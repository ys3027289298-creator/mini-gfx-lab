
/* lod-selector: screen-space-error LOD selection with hysteresis to avoid
   level thrashing, plus a frame-rate closed loop that adapts the pixel
   budget to a target fps: per-frame step clamp, deadband freeze once
   stable, and gain damping on overshoot so the budget settles without
   oscillation. Per-frame selection stays O(objects * LEVELS). */
#include <stdio.h>
#include <math.h>

#define LEVELS 4
#define OBJECTS 64
#define FRAMES 300

typedef struct {
    float error[LEVELS];   /* geometric error per level, world units */
    float radius;          /* bounding radius */
    float tau;             /* hysteresis ratio, e.g. 1.1 */
} MeshLod;

typedef struct {
    int level;
    int switches;
    int thrash;            /* switches within one unit of distance change */
} State;

static long g_evals;       /* screen_error evaluations, for the linearity check */

static float screen_error(float geom_error, float distance, float fov_y, int screen_h) {
    g_evals++;
    if (distance <= 0.0f) return 1e9f;
    return geom_error * (float)screen_h / (2.0f * distance * tanf(fov_y * 0.5f));
}

/* pick coarsest level whose projected error is below tolerance */
static int select_level(const MeshLod* m, float distance, float fov_y, int screen_h,
                        float tolerance_px, State* st) {
    int want = 0;
    for (int i = LEVELS - 1; i >= 0; i--) {
        float e = screen_error(m->error[i], distance, fov_y, screen_h);
        if (e <= tolerance_px) { want = i; break; }
    }
    int prev = st->level;
    if (want != prev) {
        /* hysteresis: require the candidate to beat the threshold by tau */
        float e_new = screen_error(m->error[want], distance, fov_y, screen_h);
        float e_old = screen_error(m->error[prev], distance, fov_y, screen_h);
        if (want > prev && e_new > tolerance_px * m->tau) return prev;
        if (want < prev && e_old < tolerance_px / m->tau) return prev;
        st->switches++;
        st->level = want;
    }
    return st->level;
}

/* ---- frame-rate closed loop -------------------------------------------
   Adjusts the pixel-tolerance budget so the measured fps holds the target.
   - step is clamped to max_step per frame (rate limit)
   - inside target +- deadband the budget is frozen (no hunting)
   - if the fps error flips sign (overshoot), the gain is halved
   - the fps/budget slope is re-estimated from each step's outcome, so the
     predicted step approaches the target geometrically instead of
     overshooting it */
typedef struct {
    float target_fps;
    float deadband;          /* fps; freeze inside target +- deadband */
    float budget;            /* current tolerance_px */
    float min_budget, max_budget;
    float max_step;          /* per-frame budget change limit */
    float gain;              /* fraction of predicted need applied per frame */
    float slope;             /* estimated d(fps)/d(budget), > 0 */
    float prev_budget;
    float prev_fps;
    int   primed;            /* have a previous sample to compare against */
    int   overshoots;        /* times the error sign flipped */
} FpsLoop;

static float fps_loop_update(FpsLoop* l, float fps) {
    float err = fps - l->target_fps;
    if (fabsf(err) <= l->deadband) {
        /* stable: freeze the budget so the loop does not hunt */
        l->prev_fps = fps;
        l->prev_budget = l->budget;
        return l->budget;
    }
    if (l->primed) {
        float db = l->budget - l->prev_budget;
        float df = fps - l->prev_fps;
        /* refine the slope estimate from the outcome of the last step */
        if (fabsf(db) > 1e-6f) {
            float observed = df / db;
            if (observed > 0.0f)
                l->slope = 0.5f * l->slope + 0.5f * observed;
        }
        /* overshoot: the error flipped sign -> damp future steps */
        float prev_err = l->prev_fps - l->target_fps;
        if (prev_err * err < 0.0f && fabsf(db) > 1e-6f) {
            l->gain *= 0.5f;
            if (l->gain < 0.05f) l->gain = 0.05f;
            l->overshoots++;
        }
    }
    float need = (l->target_fps - fps) / l->slope;  /* budget units to target */
    float step = need * l->gain;
    if (step >  l->max_step) step =  l->max_step;
    if (step < -l->max_step) step = -l->max_step;
    l->prev_budget = l->budget;
    l->prev_fps = fps;
    l->budget += step;
    if (l->budget < l->min_budget) l->budget = l->min_budget;
    if (l->budget > l->max_budget) l->budget = l->max_budget;
    l->primed = 1;
    return l->budget;
}

/* ---- simulated scene -------------------------------------------------- */

typedef struct {
    MeshLod mesh;
    State   state;
    float   distance;
} Object;

static const float TRIS[LEVELS] = { 50000.0f, 8000.0f, 1500.0f, 200.0f };

static int all_within(const float* fps, int start, int end, float target, float band) {
    for (int i = start; i <= end; i++)
        if (fabsf(fps[i] - target) > band) return 0;
    return 1;
}

/* how far the budget travelled past its settled value, relative to travel */
static float overshoot_ratio(const float* bud, int start, int end, float settled) {
    float dir = settled - bud[start];
    float travel = fabsf(dir);
    if (travel < 1e-6f) return 0.0f;
    float over = 0.0f;
    for (int i = start; i <= end; i++) {
        float past = (dir > 0.0f) ? (bud[i] - settled) : (settled - bud[i]);
        if (past > over) over = past;
    }
    return over / travel;
}

static float mean_range(const float* v, int start, int end) {
    float s = 0.0f;
    for (int i = start; i <= end; i++) s += v[i];
    return s / (float)(end - start + 1);
}

static void sparkline(const char* label, const float* v, int n, float lo, float hi) {
    static const char* bars[] = { "▁", "▂", "▃", "▄",
                                  "▅", "▆", "▇", "█" };
    printf("%s", label);
    int cols = 100;
    for (int c = 0; c < cols; c++) {
        float s = 0.0f;
        int cnt = 0;
        for (int i = c * n / cols; i < (c + 1) * n / cols; i++) { s += v[i]; cnt++; }
        float x = (s / cnt - lo) / (hi - lo);
        if (x < 0.0f) x = 0.0f;
        if (x > 1.0f) x = 1.0f;
        printf("%s", bars[(int)(x * 7.999f)]);
    }
    printf(" [%.0f..%.0f]\n", lo, hi);
}

int main(void) {
    /* part 1: hysteresis sanity check (receding camera) */
    {
        MeshLod m = { {0.0f, 1.0f, 4.0f, 16.0f}, 2.0f, 1.1f };
        State st = {0, 0, 0};
        float fov = 1.0f; int h = 1080;
        int hist[LEVELS] = {0};
        for (int i = 0; i < 40; i++) {
            float d = 5.0f + 5.0f * i;
            int lv = select_level(&m, d, fov, h, 50.0f, &st);
            hist[lv]++;
        }
        printf("part1 levels=%d,%d,%d,%d switches=%d hysteresis_ok=%d\n",
               hist[0], hist[1], hist[2], hist[3], st.switches, st.switches <= 8);
    }

    /* part 2: frame-rate closed loop over a simulated scene */
    Object objs[OBJECTS];
    for (int i = 0; i < OBJECTS; i++) {
        float scale = 1.0f + (float)(i % 3);
        objs[i].mesh = (MeshLod){ {0.0f, 1.0f * scale, 4.0f * scale, 16.0f * scale}, 2.0f, 1.1f };
        objs[i].state = (State){0, 0, 0};
        objs[i].distance = 10.0f + 3.0f * i;
    }
    FpsLoop loop = {
        .target_fps = 60.0f, .deadband = 5.0f,
        .budget = 12.0f, .min_budget = 8.0f, .max_budget = 220.0f,
        .max_step = 12.0f, .gain = 0.5f, .slope = 4.0f,
        .prev_budget = 12.0f, .prev_fps = 0.0f, .primed = 0, .overshoots = 0,
    };
    float fov = 1.0f; int h = 1080;
    float fps_hist[FRAMES], bud_hist[FRAMES];
    long evals_per_frame = 0;

    printf("part2 frame-rate closed loop (target %.0f fps +/- %.0f)\n",
           loop.target_fps, loop.deadband);
    printf("frame,fps,budget_px\n");
    for (int f = 0; f < FRAMES; f++) {
        /* disturbance: frames 150..219 carry +8ms of external load */
        float extra_ms = (f >= 150 && f < 220) ? 8.0f : 0.0f;
        float jitter_ms = 0.15f * sinf((float)f * 1.7f);
        float budget = loop.budget;
        float tris = 0.0f;
        g_evals = 0;
        for (int i = 0; i < OBJECTS; i++) {
            int lv = select_level(&objs[i].mesh, objs[i].distance, fov, h,
                                  budget, &objs[i].state);
            tris += TRIS[lv];
        }
        evals_per_frame = g_evals;
        float frame_ms = 2.0f + extra_ms + jitter_ms + tris / 20000.0f;
        float fps = 1000.0f / frame_ms;
        fps_hist[f] = fps;
        bud_hist[f] = budget;
        fps_loop_update(&loop, fps);
        if (f % 5 == 0 || f == FRAMES - 1)
            printf("%d,%.1f,%.2f\n", f, fps, budget);
    }

    sparkline("fps   ", fps_hist, FRAMES, 0.0f, 80.0f);
    sparkline("budget", bud_hist, FRAMES, 8.0f, 220.0f);

    /* checks */
    float settled1 = mean_range(bud_hist, 110, 145);
    float settled2 = mean_range(bud_hist, 180, 215);
    float settled3 = mean_range(bud_hist, 260, 299);
    int stable1 = all_within(fps_hist, 110, 145, loop.target_fps, loop.deadband);
    int stable2 = all_within(fps_hist, 180, 215, loop.target_fps, loop.deadband);
    int stable3 = all_within(fps_hist, 260, 299, loop.target_fps, loop.deadband);
    float over1 = overshoot_ratio(bud_hist, 0, 149, settled1);
    float over2 = overshoot_ratio(bud_hist, 150, 219, settled2);
    float over3 = overshoot_ratio(bud_hist, 220, 299, settled3);
    int reversals = 0;
    for (int i = 2; i < FRAMES; i++) {
        float d1 = bud_hist[i] - bud_hist[i - 1];
        float d0 = bud_hist[i - 1] - bud_hist[i - 2];
        if (d1 * d0 < -1e-6f) reversals++;
    }
    int linear_ok = (evals_per_frame <= OBJECTS * (LEVELS + 2));
    int ok = stable1 && stable2 && stable3 &&
             over1 < 0.02f && over2 < 0.02f && over3 < 0.02f &&
             reversals <= 4 && linear_ok;

    printf("stable_phase1=%d stable_phase2=%d stable_phase3=%d\n", stable1, stable2, stable3);
    printf("overshoot_phase1=%.3f overshoot_phase2=%.3f overshoot_phase3=%.3f (of travel)\n",
           over1, over2, over3);
    printf("budget_reversals=%d loop_overshoot_events=%d\n", reversals, loop.overshoots);
    printf("select_evals_per_frame=%ld objects*levels=%d linear_ok=%d\n",
           evals_per_frame, OBJECTS * LEVELS, linear_ok);
    printf("closed_loop_ok=%d\n", ok);
    return 0;
}
