
/* mini-perception: guard perception for stealth games.
   - vision: cone stimulus attenuated by distance and angle, zeroed when a
     blocked tile breaks line of sight
   - hearing: one-shot events with radius-based falloff, omnidirectional and
     unaffected by walls (impulses applied directly to awareness)
   - awareness meter: piecewise-constant ODE da/dt = rate * s, integrated with
     exact Euler steps, so the detection frame at any frame rate matches the
     analytic detection time (threshold - a) / (rate * s)
   - memory: last-known position held on a timer; when the timer expires the
     meter hard-resets, so re-detection after expiry pays the full delay again
     while reacquisition before expiry is faster thanks to residual awareness */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define GRID_MAX 64
#define MAX_EVENTS 16

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct { float x, y; } Vec2;

typedef struct { int w, h; unsigned char blocked[GRID_MAX][GRID_MAX]; } Grid;

typedef enum {
    PS_UNAWARE = 0,    /* no stimulus, no memory */
    PS_SUSPICIOUS,     /* perceiving, awareness below threshold */
    PS_DETECTED,       /* awareness crossed the threshold */
    PS_MEMORY          /* perception lost, holding last-known position */
} PercState;

typedef struct {
    Vec2 pos;
    float yaw;                 /* facing direction, radians, 0 = +x */
    /* tuning */
    float view_range;          /* tiles */
    float fov;                 /* full cone angle, radians */
    float rise_rate;           /* awareness per second at stimulus s = 1 */
    float decay_rate;          /* awareness drained per second without stimulus */
    float detect_threshold;    /* awareness at which the target is discovered */
    float memory_duration;     /* seconds the last-known position is retained */
    float hear_gain;           /* impulse multiplier for heard events */
    /* runtime state */
    float awareness;           /* 0..1 */
    float memory_timer;        /* counts down while nothing is perceived */
    Vec2 last_known;
    int has_lkp;
    PercState state;
    float sim_time;
    float detect_time;         /* first threshold crossing, -1 after full reset */
    int perceiving;            /* last tick had any stimulus */
    float last_vis;            /* raw visual stimulus of the last tick */
} Guard;

typedef struct { Vec2 pos; float radius, loudness; } SoundEvent;

typedef struct { SoundEvent items[MAX_EVENTS]; int count; } SoundBus;

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float wrap_pi(float a) {
    while (a >  M_PI) a -= 2.0f * (float)M_PI;
    while (a < -M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static int grid_load(Grid* g, const char* const* rows, int h) {
    int w = (int)strlen(rows[0]);
    if (w <= 0 || w > GRID_MAX || h <= 0 || h > GRID_MAX) return -1;
    g->w = w; g->h = h;
    for (int y = 0; y < h; y++) {
        if ((int)strlen(rows[y]) != w) return -1;
        for (int x = 0; x < w; x++) g->blocked[y][x] = (rows[y][x] == '#');
    }
    return 0;
}

/* Amanatides-Woo DDA: returns 1 if any tile crossed by segment a->b blocks. */
static int grid_occluded(const Grid* g, Vec2 a, Vec2 b) {
    int x = (int)floorf(a.x), y = (int)floorf(a.y);
    int tx = (int)floorf(b.x), ty = (int)floorf(b.y);
    if (x < 0 || y < 0 || x >= g->w || y >= g->h) return 1;
    if (g->blocked[y][x]) return 1;
    if (x == tx && y == ty) return 0;
    float dx = b.x - a.x, dy = b.y - a.y;
    int sx = dx > 0.0f ? 1 : (dx < 0.0f ? -1 : 0);
    int sy = dy > 0.0f ? 1 : (dy < 0.0f ? -1 : 0);
    float tmax_x = INFINITY, tmax_y = INFINITY;
    if (sx != 0) {
        float edge = sx > 0 ? ((float)x + 1.0f - a.x) : (a.x - (float)x);
        tmax_x = edge / fabsf(dx);
    }
    if (sy != 0) {
        float edge = sy > 0 ? ((float)y + 1.0f - a.y) : (a.y - (float)y);
        tmax_y = edge / fabsf(dy);
    }
    float tdelta_x = sx != 0 ? fabsf(1.0f / dx) : INFINITY;
    float tdelta_y = sy != 0 ? fabsf(1.0f / dy) : INFINITY;
    for (;;) {
        if (tmax_x < tmax_y) { x += sx; tmax_x += tdelta_x; }
        else                 { y += sy; tmax_y += tdelta_y; }
        if (x < 0 || y < 0 || x >= g->w || y >= g->h) return 1;
        if (g->blocked[y][x]) return 1;
        if (x == tx && y == ty) return 0;
    }
}

static void guard_init(Guard* g, Vec2 pos, float yaw) {
    memset(g, 0, sizeof(*g));
    g->pos = pos;
    g->yaw = yaw;
    g->view_range = 10.0f;
    g->fov = (float)M_PI / 2.0f;       /* 90 degrees, half-angle 45 */
    g->rise_rate = 1.0f;
    g->decay_rate = 0.5f;
    g->detect_threshold = 0.8f;
    g->memory_duration = 4.0f;
    g->hear_gain = 0.6f;
    g->detect_time = -1.0f;
}

/* Visual stimulus in [0,1]: distance falloff * angle falloff * conspicuousness.
   Out of range, outside the cone, or occluded => exactly 0. */
static float visual_stimulus(const Guard* g, const Grid* grid,
                             Vec2 target, float conspicuous) {
    float dx = target.x - g->pos.x, dy = target.y - g->pos.y;
    float d = hypotf(dx, dy);
    if (d > g->view_range) return 0.0f;
    float dist_f = 1.0f - d / g->view_range;            /* 1 on top, 0 at edge */
    if (d == 0.0f) return clampf(conspicuous, 0.0f, 1.0f);
    float ang = fabsf(wrap_pi(atan2f(dy, dx) - g->yaw));
    float half = g->fov * 0.5f;
    if (ang > half) return 0.0f;
    float ang_f = 1.0f - ang / half;                    /* 1 centered, 0 at edge */
    if (grid_occluded(grid, g->pos, target)) return 0.0f;
    return clampf(dist_f * ang_f * conspicuous, 0.0f, 1.0f);
}

static int bus_emit(SoundBus* bus, Vec2 pos, float radius, float loudness) {
    if (bus->count >= MAX_EVENTS) return -1;
    bus->items[bus->count++] = (SoundEvent){ pos, radius, loudness };
    return 0;
}

/* Analytic discovery time for a constant visual stimulus, starting now. */
static float time_to_detect(const Guard* g, float stimulus) {
    if (stimulus <= 0.0f) return INFINITY;
    if (g->awareness >= g->detect_threshold) return 0.0f;
    return (g->detect_threshold - g->awareness) / (g->rise_rate * stimulus);
}

/* Core state machine. vis is a precomputed visual stimulus; the bus is consumed. */
static void guard_tick(Guard* g, float dt, float vis,
                       Vec2 target_pos, SoundBus* bus) {
    g->sim_time += dt;
    float hear = 0.0f;
    float loudest = 0.0f;
    Vec2 hear_pos = {0.0f, 0.0f};
    int heard = 0;
    if (bus) {
        for (int i = 0; i < bus->count; i++) {
            SoundEvent* e = &bus->items[i];
            float d = hypotf(e->pos.x - g->pos.x, e->pos.y - g->pos.y);
            if (d <= e->radius) {                        /* radius-gated hearing */
                float h = e->loudness * (1.0f - d / e->radius);
                hear += h;                               /* simultaneous events stack */
                if (h > loudest) { loudest = h; heard = 1; hear_pos = e->pos; }
            }
        }
        bus->count = 0;                                 /* events are one-shot */
    }

    int perceive = vis > 0.0f || heard;
    if (perceive) {
        g->memory_timer = g->memory_duration;
        if (vis > 0.0f) { g->last_known = target_pos; g->has_lkp = 1; }
        else            { g->last_known = hear_pos;   g->has_lkp = 1; }
        g->awareness += g->rise_rate * vis * dt;       /* exact for constant s */
        if (heard) g->awareness += hear * g->hear_gain; /* event: frame-rate-free impulse */
        g->awareness = clampf(g->awareness, 0.0f, 1.0f);
    } else {
        g->awareness -= g->decay_rate * dt;
        if (g->awareness < 0.0f) g->awareness = 0.0f;
        g->memory_timer -= dt;
        if (g->memory_timer <= 0.0f) {                  /* memory fully expired */
            g->memory_timer = 0.0f;
            g->awareness = 0.0f;
            g->has_lkp = 0;
        }
    }

    g->perceiving = perceive;
    g->last_vis = vis;
    if (g->awareness >= g->detect_threshold) {
        if (g->state != PS_DETECTED) g->detect_time = g->sim_time;
        g->state = PS_DETECTED;
    } else if (perceive) {
        g->state = PS_SUSPICIOUS;
    } else if (g->has_lkp) {
        g->state = PS_MEMORY;
    } else {
        g->state = PS_UNAWARE;
        g->detect_time = -1.0f;
    }
}

/* High-level entry: sense the target through the grid, then run the meter. */
static void guard_update(Guard* g, const Grid* grid, Vec2 target,
                         float conspicuous, float dt, SoundBus* bus) {
    float vis = visual_stimulus(g, grid, target, conspicuous);
    guard_tick(g, dt, vis, target, bus);
}

static const char* state_name(PercState s) {
    static const char* names[] = {"UNAWARE", "SUSPICIOUS", "DETECTED", "MEMORY"};
    return names[s];
}

/* ---------------- self tests ---------------- */

static int g_fail;

static void check(int ok, const char* name) {
    printf("  [%s] %s\n", ok ? "ok" : "FAIL", name);
    if (!ok) g_fail++;
}

static float script_vis(float t) {
    if (t < 1.0f) return 0.5f;      /* visible, weak stimulus */
    if (t < 2.0f) return 0.0f;      /* lost: meter decays */
    return 0.8f;                    /* reacquired, strong stimulus */
}

static float run_script(float dt, float* final_awareness) {
    Guard g;
    guard_init(&g, (Vec2){0.0f, 0.0f}, 0.0f);
    float t = 0.0f;
    while (g.state != PS_DETECTED && t < 100.0f) {
        guard_tick(&g, dt, script_vis(t), (Vec2){1.0f, 0.0f}, NULL);
        t += dt;
    }
    *final_awareness = g.awareness;
    return g.detect_time;
}

int main(void) {
    Grid open, wall;
    const char* open_rows[] = {
        "............",
        "............",
        "............",
        "............",
        "............",
    };
    const char* wall_rows[] = {
        "............",
        "....#.......",
        "....#.......",
        "............",
        "............",
    };
    if (grid_load(&open, open_rows, 5) != 0 ||
        grid_load(&wall, wall_rows, 5) != 0) {
        printf("grid load fail\n");
        return 1;
    }

    /* 1. cone geometry: distance and angle falloff, occlusion coupling */
    printf("vision cone:\n");
    Guard g;
    guard_init(&g, (Vec2){1.0f, 1.0f}, 0.0f);           /* facing +x */
    float s_center = visual_stimulus(&g, &open, (Vec2){6.0f, 1.0f}, 1.0f);
    printf("  center d=5  s=%.4f (expect 0.5)\n", s_center);
    check(fabsf(s_center - 0.5f) < 1e-5f, "distance falloff 1 - d/range");
    float s_on_top = visual_stimulus(&g, &open, (Vec2){1.0f, 1.0f}, 1.0f);
    check(s_on_top == 1.0f, "target on top of guard is fully seen");
    float s_edge = visual_stimulus(&g, &open, (Vec2){5.0f, 5.0f}, 1.0f);
    check(s_edge == 0.0f, "cone edge (45 deg) stimulus is zero");
    float s_inside = visual_stimulus(&g, &open, (Vec2){5.0f, 4.9f}, 1.0f);
    check(s_inside > 0.0f && s_inside < s_center, "just inside edge: weak but nonzero");
    float s_behind = visual_stimulus(&g, &open, (Vec2){1.0f, 6.0f}, 1.0f);
    check(s_behind == 0.0f, "behind the guard is outside the cone");
    float s_range = visual_stimulus(&g, &open, (Vec2){11.0f, 1.0f}, 1.0f);
    check(s_range == 0.0f, "at/beyond view range the stimulus is zero");
    float s_crouch = visual_stimulus(&g, &open, (Vec2){6.0f, 1.0f}, 0.4f);
    check(fabsf(s_crouch - 0.2f) < 1e-5f, "conspicuousness scales the stimulus");
    float s_blocked = visual_stimulus(&g, &wall, (Vec2){6.0f, 1.0f}, 1.0f);
    check(s_blocked == 0.0f, "line of sight blocked by wall => zero");
    float s_clear = visual_stimulus(&g, &wall, (Vec2){3.0f, 1.0f}, 1.0f);
    check(s_clear > 0.0f, "target before the wall still seen");
    check(grid_occluded(&wall, (Vec2){1.0f, 1.0f}, (Vec2){6.0f, 1.0f}) == 1,
          "DDA reports the blocking tile");
    check(grid_occluded(&open, (Vec2){1.0f, 1.0f}, (Vec2){6.0f, 1.0f}) == 0,
          "open grid has line of sight");

    /* 2. hearing: radius-gated events with distance attenuation */
    printf("hearing events:\n");
    Guard h;
    guard_init(&h, (Vec2){1.0f, 1.0f}, 0.0f);
    SoundBus bus = {0};
    bus_emit(&bus, (Vec2){3.5f, 1.0f}, 5.0f, 1.0f);       /* d=2.5 => atten 0.5 */
    guard_tick(&h, 0.1f, 0.0f, (Vec2){0, 0}, &bus);
    printf("  awareness=%.4f (expect 0.3)\n", h.awareness);
    check(fabsf(h.awareness - 0.3f) < 1e-5f, "event impulse = loudness * atten * gain");
    check(h.has_lkp && h.last_known.x == 3.5f && h.last_known.y == 1.0f,
          "heard event registers last-known position");
    check(h.state == PS_SUSPICIOUS, "noise below threshold makes guard suspicious");
    check(bus.count == 0, "sound bus is consumed by the tick");
    Guard quiet;
    guard_init(&quiet, (Vec2){1.0f, 1.0f}, 0.0f);
    SoundBus bq = {0};
    bus_emit(&bq, (Vec2){3.5f, 1.0f}, 1.0f, 1.0f);       /* d=2.5 > radius */
    guard_tick(&quiet, 0.1f, 0.0f, (Vec2){0, 0}, &bq);
    check(quiet.awareness == 0.0f, "event outside radius is inaudible");

    Guard near_g, far_g;
    guard_init(&near_g, (Vec2){0, 0}, 0.0f);
    guard_init(&far_g, (Vec2){0, 0}, 0.0f);
    SoundBus bn = {0}, bf = {0};
    bus_emit(&bn, (Vec2){0, 0}, 5.0f, 1.0f);              /* d=0 => atten 1 */
    bus_emit(&bf, (Vec2){4.0f, 0}, 5.0f, 1.0f);           /* d=4 => atten 0.2 */
    guard_tick(&near_g, 0.1f, 0.0f, (Vec2){0, 0}, &bn);
    guard_tick(&far_g, 0.1f, 0.0f, (Vec2){0, 0}, &bf);
    check(near_g.awareness > far_g.awareness, "nearby event is louder than distant one");

    Guard shot;
    guard_init(&shot, (Vec2){1.0f, 1.0f}, 0.0f);
    SoundBus bs = {0};
    bus_emit(&bs, (Vec2){1.0f, 1.0f}, 8.0f, 1.0f);
    bus_emit(&bs, (Vec2){1.0f, 1.0f}, 8.0f, 1.0f);        /* two loud impulses */
    guard_tick(&shot, 0.1f, 0.0f, (Vec2){0, 0}, &bs);     /* 2 * 1.0 * 0.6 = 1.2 */
    check(shot.awareness > shot.detect_threshold && shot.state == PS_DETECTED &&
          fabsf(shot.detect_time - 0.1f) < 1e-5f,
          "strong noise discovers the target immediately");
    check(shot.has_lkp && shot.last_known.x == 1.0f, "detection stores the noise position");

    /* 3. frame-exact detection time: stepped meters vs analytic solution */
    printf("detection timing (analytic 1.6000 s at s=0.5):\n");
    Guard base;
    guard_init(&base, (Vec2){0, 0}, 0.0f);
    float analytic05 = time_to_detect(&base, 0.5f);
    const float dts[3] = {1.0f / 60.0f, 1.0f / 30.0f, 1.0f / 10.0f};
    for (int i = 0; i < 3; i++) {
        Guard m;
        guard_init(&m, (Vec2){0, 0}, 0.0f);
        float t = 0.0f;
        while (m.state != PS_DETECTED && t < 10.0f) {
            guard_tick(&m, dts[i], 0.5f, (Vec2){1, 0}, NULL);
            t += dts[i];
        }
        printf("  dt=%.4f detect=%.4f analytic=%.4f\n", dts[i], m.detect_time, analytic05);
        check(fabsf(m.detect_time - analytic05) <= dts[i] * 2.0f + 1e-5f,
              "stepped detection frame contains the analytic time");
    }
    check(fabsf(analytic05 - 1.6f) < 1e-5f,
          "analytic delay from zero awareness");

    /* 4. end-to-end occlusion, decay, memory expiry and re-detection */
    printf("occlusion + memory lifecycle:\n");
    Guard e;
    guard_init(&e, (Vec2){1.0f, 1.0f}, 0.0f);
    Vec2 target = {6.0f, 1.0f};
    float dt = 1.0f / 60.0f;
    float detected_at = -1.0f;
    for (float t = 0.0f; t < 9.0f; t += dt) {
        const Grid* gp = (t < 1.0f || t >= 3.0f) ? &open : &wall;
        guard_update(&e, gp, target, 1.0f, dt, NULL);
        if (t < 1.0f && fabsf(t - (1.0f - dt)) < 1e-6f)
            check(fabsf(e.awareness - 0.5f) < 1e-4f, "visible phase: awareness rises to 0.5");
        if (fabsf(t - (2.0f - dt)) < 1e-6f)
            check(e.awareness < 1e-4f && e.state == PS_MEMORY,
                  "occluded: meter decays to zero but memory persists");
        if (e.state == PS_DETECTED && detected_at < 0.0f) detected_at = e.detect_time;
    }
    printf("  detect_time=%.4f (expect 4.6000)\n", detected_at);
    check(fabsf(detected_at - 4.6f) <= dt * 2.0f + 1e-5f,
          "re-emergence detection matches piecewise analytic timing");

    /* hide the target again: detection lingers as memory, then fully expires */
    for (float t = 0.0f; t < 5.0f; t += dt)
        guard_update(&e, &wall, target, 1.0f, dt, NULL);
    check(e.state == PS_UNAWARE && !e.has_lkp && e.awareness == 0.0f,
          "memory expired: full reset, last-known position cleared");
    check(e.detect_time == -1.0f, "expiry clears the stale detection stamp");

    /* re-detection after expiry pays the full delay again */
    float fresh_delay, redo_delay;
    {
        Guard a; guard_init(&a, (Vec2){0, 0}, 0.0f);
        fresh_delay = time_to_detect(&a, 0.5f);
    }
    redo_delay = time_to_detect(&e, 0.5f);
    printf("  fresh=%.4f after_expiry=%.4f\n", fresh_delay, redo_delay);
    check(fabsf(redo_delay - fresh_delay) < 1e-5f,
          "re-detection after memory expiry restarts the full delay");

    /* reacquisition before expiry keeps residual awareness and is faster */
    Guard r;
    guard_init(&r, (Vec2){0, 0}, 0.0f);
    guard_tick(&r, 0.8f, 0.5f, (Vec2){1, 0}, NULL);       /* awareness 0.4 */
    guard_tick(&r, 0.2f, 0.0f, (Vec2){1, 0}, NULL);       /* briefly lost */
    check(r.state == PS_MEMORY && r.awareness > 0.0f, "short loss leaves residual memory");
    float fast = time_to_detect(&r, 0.5f);
    printf("  residual reacquire=%.4f vs full=%.4f\n", fast, fresh_delay);
    check(fast < fresh_delay, "re-perceived before expiry: shorter delay");

    /* 5. frame-rate independence on a piecewise script (analytic answer 3.0 s) */
    printf("frame-rate independence:\n");
    float prev_t = 0.0f, prev_a = 0.0f;
    for (int i = 0; i < 3; i++) {
        float aw;
        float td = run_script(dts[i], &aw);
        printf("  dt=%.4f detect=%.4f awareness=%.4f\n", dts[i], td, aw);
        check(fabsf(td - 3.0f) <= dts[i] * 2.0f + 1e-5f, "scripted detection matches analytic 3.0 s");
        if (i > 0) check(fabsf(td - prev_t) <= dts[2] + 1e-5f && fabsf(aw - prev_a) < 0.05f,
                         "different frame rates converge to the same outcome");
        prev_t = td; prev_a = aw;
    }

    /* 6. explicit expire-then-redetect sequence through the state machine */
    printf("expire then rediscover:\n");
    Guard z;
    guard_init(&z, (Vec2){0, 0}, 0.0f);
    guard_tick(&z, 0.4f, 0.5f, (Vec2){1, 0}, NULL);
    check(z.has_lkp && z.state == PS_SUSPICIOUS, "perception creates a memory");
    guard_tick(&z, 3.9f, 0.0f, (Vec2){1, 0}, NULL);
    check(z.state == PS_MEMORY, "still inside memory window: MEMORY state");
    guard_tick(&z, 0.2f + dt, 0.0f, (Vec2){1, 0}, NULL);  /* timer crosses 4.0 s */
    check(z.state == PS_UNAWARE && !z.has_lkp && z.awareness == 0.0f,
          "past the window: UNAWARE with everything reset");
    float rediscover_start = z.sim_time;
    float t = 0.0f;
    while (z.state != PS_DETECTED && t < 9.85f) {
        guard_tick(&z, dt, 0.5f, (Vec2){1, 0}, NULL);
        t += dt;
    }
    check(fabsf((z.detect_time - rediscover_start) - 1.6f) <= dt * 2.0f + 1e-5f,
          "rediscovery after expiry counts the delay from zero again");
    printf("final state=%s\n", state_name(z.state));

    if (g_fail) { printf("\n%d check(s) FAILED\n", g_fail); return 1; }
    printf("\nALL TESTS PASSED\n");
    return 0;
}
