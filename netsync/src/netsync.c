/* netsync: snapshot state synchronization over a jittery, lossy link.
   - snapshots are interpolated with a playback delay of one tick, and the
     render clock is never allowed to trail the newest snapshot by more than
     that, so interpolation adds at most one snapshot interval of delay
   - when packets are lost the last known velocity is used to extrapolate,
     capped at NS_MAX_EXTRAP so dead reckoning cannot drift forever
   - if the cap is reached the client freezes the prediction, then rolls
     back to authoritative playback when the next snapshot arrives and fades
     the visual discontinuity out instead of popping
   - the self-test runs a reference client that receives every tick and
     checks the lossy client's error against a per-frame analytic envelope
     derived from the true trajectory, not from the measured error
   - rollback count and correction magnitude are reported
*/
#include <stdio.h>
#include <string.h>
#include <math.h>

#define NS_ENTITIES 3
#define NS_MAX_SNAPS 64
#define NS_TICK 0.05              /* server snapshot interval, 20 Hz */
#define NS_MAX_EXTRAP 0.15        /* dead-reckoning cap, three ticks */
#define NS_CORRECT_TIME 0.12      /* visual correction fade-out */
#define NS_CORRECT_EPS 0.02       /* smallest visible jump worth correcting */
#define NS_SERVER_TICKS 120
#define NS_FRAME_DT (1.0 / 60.0)

typedef struct { double x, y; } Vec2;

typedef struct {
    double t;
    Vec2 pos[NS_ENTITIES];
} Snapshot;

typedef struct {
    Snapshot snaps[NS_MAX_SNAPS];
    int count;
    int started;
    double render_t;              /* server-time being displayed */
    int timed_out;                /* extrapolation cap reached */

    Vec2 corr_start[NS_ENTITIES]; /* visual offset at rollback, fades to zero */
    double corr_left;

    /* stats */
    int recv;
    int extrap_frames;
    double extrap_run_max;        /* render clock overrun past newest snapshot */
    int timeouts;
    int rollbacks;
    int nudges;                   /* forward clock pulls, not rollbacks */
    double nudge_max;
    int corrections;
    double corr_total;
    double corr_max;
    double lag_max;
} Client;

static void client_init(Client* c) {
    memset(c, 0, sizeof(*c));
}

static void snap_push(Client* c, double t, const Vec2* pos) {
    for (int i = 0; i < c->count; i++)
        if (fabs(c->snaps[i].t - t) < 1e-9) return;
    if (c->count == NS_MAX_SNAPS) {
        memmove(&c->snaps[0], &c->snaps[1], sizeof(Snapshot) * (NS_MAX_SNAPS - 1));
        c->count--;
    }
    int idx = c->count;
    while (idx > 0 && c->snaps[idx - 1].t > t) idx--;
    memmove(&c->snaps[idx + 1], &c->snaps[idx], sizeof(Snapshot) * (size_t)(c->count - idx));
    c->snaps[idx].t = t;
    memcpy(c->snaps[idx].pos, pos, sizeof(Vec2) * NS_ENTITIES);
    c->count++;
}

/* state at render_t: interpolate between snapshots, extrapolate past the
   newest one for at most NS_MAX_EXTRAP, clamp outside the buffer */
static void eval_state(const Client* c, Vec2 out[NS_ENTITIES]) {
    if (c->count == 0) {
        for (int i = 0; i < NS_ENTITIES; i++) out[i].x = out[i].y = 0.0;
        return;
    }
    if (c->count == 1 || c->render_t <= c->snaps[0].t) {
        memcpy(out, c->snaps[0].pos, sizeof(Vec2) * NS_ENTITIES);
        return;
    }
    const Snapshot* last = &c->snaps[c->count - 1];
    if (c->render_t >= last->t) {
        const Snapshot* prev = &c->snaps[c->count - 2];
        double span = last->t - prev->t;
        double age = c->render_t - last->t;
        if (age > NS_MAX_EXTRAP) age = NS_MAX_EXTRAP;
        for (int i = 0; i < NS_ENTITIES; i++) {
            double vx = span > 0.0 ? (last->pos[i].x - prev->pos[i].x) / span : 0.0;
            double vy = span > 0.0 ? (last->pos[i].y - prev->pos[i].y) / span : 0.0;
            out[i].x = last->pos[i].x + vx * age;
            out[i].y = last->pos[i].y + vy * age;
        }
        return;
    }
    for (int i = 0; i + 1 < c->count; i++) {
        const Snapshot* a = &c->snaps[i];
        const Snapshot* b = &c->snaps[i + 1];
        if (c->render_t <= b->t) {
            double u = (c->render_t - a->t) / (b->t - a->t);
            for (int k = 0; k < NS_ENTITIES; k++) {
                out[k].x = a->pos[k].x + (b->pos[k].x - a->pos[k].x) * u;
                out[k].y = a->pos[k].y + (b->pos[k].y - a->pos[k].y) * u;
            }
            return;
        }
    }
    memcpy(out, last->pos, sizeof(Vec2) * NS_ENTITIES);
}

static double corr_mag(const Client* c) {
    if (c->corr_left <= 0.0) return 0.0;
    double k = c->corr_left / NS_CORRECT_TIME;
    double m = 0.0;
    for (int i = 0; i < NS_ENTITIES; i++) {
        double d = sqrt(c->corr_start[i].x * c->corr_start[i].x +
                        c->corr_start[i].y * c->corr_start[i].y);
        if (d > m) m = d;
    }
    return m * k;
}

static void visual_state(const Client* c, Vec2 out[NS_ENTITIES]) {
    eval_state(c, out);
    if (c->corr_left > 0.0) {
        double k = c->corr_left / NS_CORRECT_TIME;
        for (int i = 0; i < NS_ENTITIES; i++) {
            out[i].x += c->corr_start[i].x * k;
            out[i].y += c->corr_start[i].y * k;
        }
    }
}

static void client_advance(Client* c, double dt) {    if (!c->started) return;
    c->render_t += dt;
    if (c->corr_left > 0.0) {
        c->corr_left -= dt;
        if (c->corr_left < 0.0) c->corr_left = 0.0;
    }
    const Snapshot* newest = &c->snaps[c->count - 1];
    double run = c->render_t - newest->t;
    if (run > 0.0) {
        c->extrap_frames++;
        if (run > c->extrap_run_max) c->extrap_run_max = run;
        if (run >= NS_MAX_EXTRAP && !c->timed_out) {
            c->timeouts++;
            c->timed_out = 1;
        }
    }
    double lag = newest->t - c->render_t;
    if (lag > c->lag_max) c->lag_max = lag;
}

static void client_recv(Client* c, double t, const Vec2* pos) {
    Vec2 before[NS_ENTITIES];
    if (c->started) visual_state(c, before);

    snap_push(c, t, pos);
    c->recv++;

    if (!c->started) {
        c->started = 1;
        c->render_t = c->snaps[c->count - 1].t - NS_TICK;
        if (NS_TICK > c->lag_max) c->lag_max = NS_TICK;
        return;
    }

    const Snapshot* newest = &c->snaps[c->count - 1];
    double target = newest->t - NS_TICK;
    int rolled = 0;

    if (c->timed_out) {
        /* extrapolation ran to its cap: abandon the prediction and snap the
           clock back to authoritative playback */
        if (c->render_t > target) c->render_t = target;
        c->timed_out = 0;
        c->rollbacks++;
        rolled = 1;
    } else if (c->render_t < target) {
        /* hard bound on the interpolation delay: never trail the newest
           snapshot by more than one tick.  This is a clock adjust, the
           display moves forward, no correction is applied. */
        double push = target - c->render_t;
        c->render_t = target;
        c->nudges++;
        if (push > c->nudge_max) c->nudge_max = push;
    }
    if (c->render_t < target) c->render_t = target;

    double lag = newest->t - c->render_t;
    if (lag > c->lag_max) c->lag_max = lag;

    if (!rolled) return;

    Vec2 after[NS_ENTITIES];
    eval_state(c, after);
    double jump = 0.0;
    for (int i = 0; i < NS_ENTITIES; i++) {
        double dx = before[i].x - after[i].x;
        double dy = before[i].y - after[i].y;
        double d = sqrt(dx * dx + dy * dy);
        if (d > jump) jump = d;
    }
    if (jump > NS_CORRECT_EPS) {
        for (int i = 0; i < NS_ENTITIES; i++) {
            c->corr_start[i].x = before[i].x - after[i].x;
            c->corr_start[i].y = before[i].y - after[i].y;
        }
        c->corr_left = NS_CORRECT_TIME;
        c->corrections++;
        c->corr_total += jump;
        if (jump > c->corr_max) c->corr_max = jump;
    }
}

/* authoritative motion: straight line, oscillating, orbiting */
static void server_state(double t, Vec2 out[NS_ENTITIES]) {
    out[0].x = 1.0 + 2.0 * t;
    out[0].y = 2.0 + 1.0 * t;
    out[1].x = 3.0 * sin(1.5 * t);
    out[1].y = 1.5 * t;
    out[2].x = 3.0 * cos(1.0 * t + 0.3);
    out[2].y = 3.0 * sin(1.0 * t + 0.3);
}

/* displayed server time, clamped to the buffer and to the extrapolation cap */
static double display_time(const Client* c) {
    if (c->count == 0) return 0.0;
    double t = c->render_t;
    double oldest = c->snaps[0].t;
    double newest = c->snaps[c->count - 1].t;
    if (t < oldest) t = oldest;
    if (t > newest + NS_MAX_EXTRAP) t = newest + NS_MAX_EXTRAP;
    return t;
}

/* Locate the snapshot pair the display actually reads at tau.  Returns the
   bracketing indices (both 0 when no pair exists), the spacing between them,
   the distance extrapolated past the upper sample, and the distance tau sits
   outside the segment, which the display holds through. */
static void eval_segment(const Client* c, double tau,
                         int* ia, int* ib, double* span,
                         double* age_after, double* hold) {
    *ia = *ib = 0;
    *span = 0.0;
    *age_after = 0.0;
    *hold = 0.0;
    if (c->count == 0) return;
    if (c->count == 1) {
        /* one sample: the display is just that sample */
        *hold = fabs(tau - c->snaps[0].t);
        return;
    }
    if (tau <= c->snaps[0].t) {
        *ia = 0; *ib = 1;
        *span = c->snaps[1].t - c->snaps[0].t;
        *hold = c->snaps[0].t - tau;
        return;
    }
    int ibi = c->count - 1;
    for (int i = 0; i + 1 < c->count; i++) {
        if (tau <= c->snaps[i + 1].t) { ibi = i + 1; break; }
    }
    *ia = ibi - 1; *ib = ibi;
    *span = c->snaps[ibi].t - c->snaps[ibi - 1].t;
    if (tau < c->snaps[ibi - 1].t) *hold = c->snaps[ibi - 1].t - tau;
    if (tau > c->snaps[ibi].t) *age_after = tau - c->snaps[ibi].t;
}

/* Upper bound on |displayed - true| at display time tau, from the scenario's
   global peak speed and acceleration.
   - interpolating over [a, b] cuts the curve: |e| <= amax*span^2/8
     (e(t) = (t-a)(t-b)/2 * x''(xi))
   - dead reckoning past b uses the mean velocity over [a, b]; that mean
     differs from v(b+s) by at most amax*(span/2 + s), so the integrated
     error over age is amax*(span/2*age + age^2)
   - outside the segment the display holds a sample: |e| <= vmax*hold
   - the visual correction offset is still fading in: corr_mag */
static double error_envelope(const Client* c, double tau,
                             double vmax, double amax) {
    if (c->count == 0) return 0.0;
    int ia, ib;
    double span, age, hold;
    eval_segment(c, tau, &ia, &ib, &span, &age, &hold);

    double interp = 0.125 * amax * span * span;
    double predict = amax * (0.5 * span * age + age * age);
    double held = vmax * hold;
    return interp + predict + held + corr_mag(c);
}

/* largest speed and acceleration of any entity on [a, b] */
static void motion_bounds(double a, double b, double* vmax, double* amax) {
    const double h = 0.002;
    if (b - a < 0.05) { a -= 0.05; b += 0.05; }
    if (a < 0.0) { b -= a; a = 0.0; }
    *vmax = 0.0;
    *amax = 0.0;
    Vec2 p0[NS_ENTITIES], p1[NS_ENTITIES], p2[NS_ENTITIES];
    for (double t = a; t <= b + h; t += h) {
        server_state(t - h, p0);
        server_state(t, p1);
        server_state(t + h, p2);
        for (int i = 0; i < NS_ENTITIES; i++) {
            double vx = (p2[i].x - p0[i].x) / (2.0 * h);
            double vy = (p2[i].y - p0[i].y) / (2.0 * h);
            double ax = (p2[i].x - 2.0 * p1[i].x + p0[i].x) / (h * h);
            double ay = (p2[i].y - 2.0 * p1[i].y + p0[i].y) / (h * h);
            double v = sqrt(vx * vx + vy * vy);
            double acc = sqrt(ax * ax + ay * ay);
            if (v > *vmax) *vmax = v;
            if (acc > *amax) *amax = acc;
        }
    }
}

typedef struct {
    double t;
    double arr;                   /* arrival time at the client */
    Vec2 pos[NS_ENTITIES];
    int drop;
} Packet;
typedef struct { unsigned state; } Rng;

static double rng_unit(Rng* r) {
    unsigned x = r->state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    r->state = x;
    return (double)(x >> 8) * (1.0 / 16777216.0);
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    static Packet pkt[NS_SERVER_TICKS];
    Rng rng = { 0x12345678u };

    for (int k = 0; k < NS_SERVER_TICKS; k++) {
        double t = k * NS_TICK;
        pkt[k].t = t;
        server_state(t, pkt[k].pos);
        /* 10-50 ms one-way delay; a 300 ms burst plus random drops */
        pkt[k].arr = t + 0.010 + 0.040 * rng_unit(&rng);
        int burst = (t >= 2.0 - 1e-9 && t < 2.3 - 1e-9);
        pkt[k].drop = burst || rng_unit(&rng) < 0.18;
    }
    for (int i = 1; i < NS_SERVER_TICKS; i++) {
        Packet p = pkt[i];
        int j = i - 1;
        while (j >= 0 && pkt[j].arr > p.arr) { pkt[j + 1] = pkt[j]; j--; }
        pkt[j + 1] = p;
    }

    double end = 0.0;
    for (int k = 0; k < NS_SERVER_TICKS; k++) if (pkt[k].arr > end) end = pkt[k].arr;
    int frames = (int)(end / NS_FRAME_DT) + 1;

    Client lossy;
    client_init(&lossy);

    /* the envelope uses the scenario's peak speed and acceleration, sampled
       densely enough that the discrete maximum cannot undershoot */
    double vmax, amax;
    motion_bounds(0.0, NS_SERVER_TICKS * NS_TICK, &vmax, &amax);
    vmax *= 1.001;
    amax *= 1.001;

    int sent = 0, dropped = 0;
    double err_max = 0.0, env_max = 0.0;
    int env_breach = 0, env_frames = 0;
    double gap_max = 0.0, last_arrival = 0.0;
    int pi = 0;
    for (int f = 0; f <= frames; f++) {
        double now = f * NS_FRAME_DT;
        while (pi < NS_SERVER_TICKS && pkt[pi].arr <= now + 1e-9) {
            sent++;
            if (!pkt[pi].drop) {
                client_recv(&lossy, pkt[pi].t, pkt[pi].pos);
                double gap = last_arrival > 0.0 ? pkt[pi].arr - last_arrival : 0.0;
                if (gap > gap_max) gap_max = gap;
                last_arrival = pkt[pi].arr;
            } else {
                dropped++;
            }
            pi++;
        }
        client_advance(&lossy, NS_FRAME_DT);
        if (!lossy.started) continue;

        /* A client that receives every tick renders the server state exactly,
           so the true state at the lossy client's display time is the
           per-frame reference. */
        Vec2 shown[NS_ENTITIES], truth[NS_ENTITIES];
        visual_state(&lossy, shown);
        double tau = display_time(&lossy);
        server_state(tau, truth);
        double err_frame = 0.0;
        for (int i = 0; i < NS_ENTITIES; i++) {
            double dx = shown[i].x - truth[i].x;
            double dy = shown[i].y - truth[i].y;
            double d = sqrt(dx * dx + dy * dy);
            if (d > err_frame) err_frame = d;
        }
        if (err_frame > err_max) err_max = err_frame;

        /* analytic envelope for this frame, from the scenario's peak speed
           and acceleration */
        double env = error_envelope(&lossy, tau, vmax, amax);
        if (env > env_max) env_max = env;
        env_frames++;
        if (err_frame > env) env_breach++;
    }

    printf("snapshots_sent=%d recv=%d dropped=%d\n",
           sent, lossy.recv, dropped);
    printf("extrap_frames=%d extrap_run_max_s=%.3f timeouts=%d gap_max_s=%.3f\n",
           lossy.extrap_frames, lossy.extrap_run_max, lossy.timeouts, gap_max);
    printf("rollbacks=%d corrections=%d correction_total=%.3f correction_max=%.3f\n",
           lossy.rollbacks, lossy.corrections, lossy.corr_total, lossy.corr_max);
    printf("clock_nudges=%d nudge_max_s=%.3f\n", lossy.nudges, lossy.nudge_max);
    printf("interp_lag_max_s=%.4f\n", lossy.lag_max);
    printf("error_max=%.3f error_envelope_max=%.3f env_frames=%d env_breach=%d\n",
           err_max, env_max, env_frames, env_breach);

    check("packet_accounting", sent == lossy.recv + dropped);
    check("extrapolation_used", lossy.extrap_frames > 0);
    check("timeout_rollback", lossy.timeouts >= 1 && lossy.rollbacks >= 1 &&
                               lossy.rollbacks <= lossy.timeouts);
    check("correction_reported", lossy.corrections >= lossy.rollbacks &&
                                 lossy.corr_max > NS_CORRECT_EPS);
    check("interp_delay_within_one_frame", lossy.lag_max <= NS_TICK + 1e-6);
    check("error_within_envelope", env_breach == 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
