
/* lod-selector: screen-space-error LOD selection with adaptive hysteresis.
   The hysteresis band widens with smoothed camera speed (sudden speed
   changes ease in through an EMA), and level changes are clamped to one
   step per update so fast motion never produces a quality cliff. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <math.h>
#include <time.h>

#define LEVELS 4

typedef struct {
    float error[LEVELS];   /* geometric error per level, world units */
    float radius;          /* bounding radius */
    float tau;             /* base hysteresis ratio at rest, e.g. 1.1 */
    float tau_max;         /* hysteresis ratio at full speed, e.g. 2.0 */
    float speed_ref;       /* camera speed (units/s) mapped to tau_max */
    float smooth;          /* EMA weight for speed smoothing, 0..1 */
} MeshLod;

typedef struct {
    int level;
    int switches;
    float speed_ema;       /* smoothed camera speed, units/s */
    float tau_eff;         /* hysteresis ratio currently in force */
    float prev_dist;
    float prev_time;
    int primed;
} State;

/* queryable view of the adaptive parameters and runtime state */
typedef struct {
    float tau_base;
    float tau_max;
    float tau_eff;
    float speed;
    float speed_ref;
    int level;
    int switches;
} LodParams;

static LodParams lod_params(const MeshLod* m, const State* st) {
    LodParams p = { m->tau, m->tau_max, st->tau_eff,
                    st->speed_ema, m->speed_ref, st->level, st->switches };
    return p;
}

static float screen_error(float geom_error, float distance, float fov_y, int screen_h) {
    if (distance <= 0.0f) return 1e9f;
    return geom_error * (float)screen_h / (2.0f * distance * tanf(fov_y * 0.5f));
}

/* pick coarsest level whose projected error is below tolerance */
static int select_level(const MeshLod* m, float distance, float now,
                        float fov_y, int screen_h, float tolerance_px, State* st) {
    /* adaptive hysteresis: scale the band by smoothed camera speed */
    if (!st->primed) {
        st->primed = 1;
        st->prev_dist = distance;
        st->prev_time = now;
        st->speed_ema = 0.0f;
        st->tau_eff = m->tau;
    } else {
        float dt = now - st->prev_time;
        if (dt > 1e-6f) {
            float v = fabsf(distance - st->prev_dist) / dt;
            /* EMA: sudden speed changes transition smoothly, not instantly */
            st->speed_ema += m->smooth * (v - st->speed_ema);
            st->prev_dist = distance;
            st->prev_time = now;
        }
        float s = st->speed_ema / m->speed_ref;
        if (s > 1.0f) s = 1.0f;
        st->tau_eff = m->tau + (m->tau_max - m->tau) * s;
    }

    int want = 0;
    for (int i = LEVELS - 1; i >= 0; i--) {
        float e = screen_error(m->error[i], distance, fov_y, screen_h);
        if (e <= tolerance_px) { want = i; break; }
    }
    int prev = st->level;
    if (want != prev) {
        /* hysteresis: dead band around each boundary. Coarsen only when the
           coarser level's error is clearly below tol (<= tol/tau_eff), refine
           only when it is clearly above tol (> tol*tau_eff); inside the band
           keep the current level. tau_eff widens the band with camera speed. */
        float tau = st->tau_eff;
        if (want > prev) {
            float e_b = screen_error(m->error[prev + 1], distance, fov_y, screen_h);
            want = (e_b <= tolerance_px / tau) ? prev + 1 : prev;
        } else {
            float e_b = screen_error(m->error[prev], distance, fov_y, screen_h);
            want = (e_b > tolerance_px * tau) ? prev - 1 : prev;
        }
        /* one level step per update: no quality cliff even when fast */
        if (want != prev) {
            st->switches++;
            st->level = want;
        }
    }
    return st->level;
}

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

int main(void) {
    MeshLod m = { {0.0f, 1.0f, 4.0f, 16.0f}, 2.0f, 1.1f, 2.0f, 5.0f, 0.2f };
    float fov = 1.0f; int h = 1080; float tol = 50.0f;
    int ok = 1;

    /* 1) receding walk: hysteresis keeps switches well below sample count */
    {
        State st = {0};
        int hist[LEVELS] = {0};
        for (int i = 0; i < 40; i++) {
            float d = 5.0f + 5.0f * i;
            int lv = select_level(&m, d, i * 0.1f, fov, h, tol, &st);
            hist[lv]++;
        }
        printf("levels=%d,%d,%d,%d switches=%d\n",
               hist[0], hist[1], hist[2], hist[3], st.switches);
        int pass = st.switches <= 8;
        printf("hysteresis_ok=%d\n", pass);
        ok &= pass;
    }

    /* 2) slow oscillation near a threshold: adaptive switches at most half
          of a fixed-threshold selector */
    {
        MeshLod fixed = m;
        fixed.tau = 1.0f; fixed.tau_max = 1.0f;   /* no hysteresis */
        State sa = {0}, sf = {0};
        for (int i = 0; i < 200; i++) {
            float t = i * 0.1f;
            float d = 79.09f + 3.0f * sinf(i * 0.35f);  /* straddles a boundary */
            select_level(&m, d, t, fov, h, tol, &sa);
            select_level(&fixed, d, t, fov, h, tol, &sf);
        }
        printf("slow: adaptive=%d fixed=%d\n", sa.switches, sf.switches);
        int pass = sa.switches * 2 <= sf.switches;
        printf("slow_half_ok=%d\n", pass);
        ok &= pass;
    }

    /* 3) fast sweep: level changes at most one step per update (no cliff) */
    {
        State st = {0};
        int max_step = 0, prev_lv = 0;
        for (int i = 0; i < 60; i++) {
            float d = 5.0f + 4.0f * i;            /* ~250 units/s */
            int lv = select_level(&m, d, i * 0.016f, fov, h, tol, &st);
            int delta = lv - prev_lv;
            if (delta < 0) delta = -delta;
            if (delta > max_step) max_step = delta;
            prev_lv = lv;
        }
        printf("fast: max_level_step=%d\n", max_step);
        int pass = max_step <= 1;
        printf("no_cliff_ok=%d\n", pass);
        ok &= pass;

        /* 4) parameters are queryable */
        LodParams p = lod_params(&m, &st);
        printf("params: tau_base=%.2f tau_max=%.2f tau_eff=%.2f "
               "speed=%.1f speed_ref=%.1f level=%d switches=%d\n",
               p.tau_base, p.tau_max, p.tau_eff, p.speed, p.speed_ref,
               p.level, p.switches);
    }

    /* 5) selection cost must stay under 100ns */
    {
        State st = {0};
        volatile int sink = 0;
        const int n = 1000000;
        double t0 = now_sec();
        for (int i = 0; i < n; i++) {
            float d = 5.0f + (float)(i % 300);
            sink += select_level(&m, d, i * 0.016f, fov, h, tol, &st);
        }
        double ns = (now_sec() - t0) * 1e9 / (double)n;
        printf("select_ns=%.1f sink=%d\n", ns, sink);
        int pass = ns < 100.0;
        printf("perf_ok=%d\n", pass);
        ok &= pass;
    }

    printf("all_ok=%d\n", ok);
    return ok ? 0 : 1;
}
