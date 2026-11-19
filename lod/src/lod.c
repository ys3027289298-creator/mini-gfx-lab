
/* lod-selector: screen-space-error LOD selection with hysteresis to avoid
   level thrashing when the camera hovers near a threshold.

   Multi-viewport mode chooses one shared level per mesh per frame. Each
   viewport proposes a level using its own (bounded) tolerance relaxation;
   the strictest -- i.e. finest -- proposal wins, so split-screen views do
   not drift apart. Every shared switch is recorded. */
#include <stdio.h>
#include <math.h>

#define LEVELS 4
#define MAX_VIEWS 4
#define MAX_SWITCH_LOG 64

/* Per-viewport tolerance multiplier: 1 = strict (same as single-viewport),
   larger values relax that viewport towards coarser levels. Held to
   [RELAX_MIN, RELAX_MAX] so relaxation can never run away. */
#define RELAX_MIN 1.0f
#define RELAX_MAX 2.0f

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

typedef struct {
    float distance;
    float fov_y;
    int screen_h;
    float relax;           /* requested tolerance multiplier */
} View;

typedef struct {
    int frame;
    int from, to;          /* shared level before / after */
    int strict;            /* strictest unrelaxed proposal at that frame */
    float relax;           /* effective relaxation of the binding viewport */
} SwitchRecord;

typedef struct {
    int level;
    int switches;
    int thrash;
    int nlog;              /* retained records, capped at MAX_SWITCH_LOG */
    SwitchRecord log[MAX_SWITCH_LOG];
} MultiState;

static float screen_error(float geom_error, float distance, float fov_y, int screen_h) {
    if (distance <= 0.0f) return 1e9f;
    return geom_error * (float)screen_h / (2.0f * distance * tanf(fov_y * 0.5f));
}

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float view_relax(const View* v) {
    return clampf(v->relax, RELAX_MIN, RELAX_MAX);
}

/* coarsest level whose projected error is below tolerance */
static int want_level(const MeshLod* m, float distance, float fov_y, int screen_h,
                      float tolerance_px) {
    for (int i = LEVELS - 1; i >= 0; i--) {
        float e = screen_error(m->error[i], distance, fov_y, screen_h);
        if (e <= tolerance_px) return i;
    }
    return 0;
}

/* original single-viewport selector; kept for behavioural parity */
static int select_level(const MeshLod* m, float distance, float fov_y, int screen_h,
                        float tolerance_px, State* st) {
    int want = want_level(m, distance, fov_y, screen_h, tolerance_px);
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

/* One shared level for all viewports: take each viewport's proposal with
   its own bounded relaxation, then keep the strictest (finest) one.
   Hysteresis is evaluated on the binding viewport, so a single viewport
   with relax == RELAX_MIN behaves exactly like select_level(). */
static int select_level_multi(const MeshLod* m, const View* views, int n,
                              float tolerance_px, int frame, MultiState* st) {
    if (n <= 0) return st->level;
    if (n > MAX_VIEWS) n = MAX_VIEWS;

    int wants[MAX_VIEWS];
    float tol_eff[MAX_VIEWS];
    int target = LEVELS - 1;
    int strict = LEVELS - 1;

    for (int i = 0; i < n; i++) {
        tol_eff[i] = tolerance_px * view_relax(&views[i]);
        wants[i] = want_level(m, views[i].distance, views[i].fov_y,
                              views[i].screen_h, tol_eff[i]);
        int sw = want_level(m, views[i].distance, views[i].fov_y,
                            views[i].screen_h, tolerance_px);
        if (wants[i] < target) target = wants[i];   /* finer == stricter */
        if (sw < strict) strict = sw;
    }

    /* binding viewport: largest normalised error at the shared level; it
       supplies the hysteresis thresholds for the shared decision */
    int bind = 0;
    float worst = -1.0f;
    for (int i = 0; i < n; i++) {
        if (wants[i] != target) continue;
        float e = screen_error(m->error[target], views[i].distance,
                               views[i].fov_y, views[i].screen_h);
        float ratio = e / tol_eff[i];
        if (ratio > worst) { worst = ratio; bind = i; }
    }

    int prev = st->level;
    if (target != prev) {
        const View* b = &views[bind];
        float e_new = screen_error(m->error[target], b->distance, b->fov_y, b->screen_h);
        float e_old = screen_error(m->error[prev], b->distance, b->fov_y, b->screen_h);
        int hold = (target > prev && e_new > tol_eff[bind] * m->tau) ||
                   (target < prev && e_old < tol_eff[bind] / m->tau);
        if (!hold) {
            SwitchRecord* r = &st->log[st->switches % MAX_SWITCH_LOG];
            r->frame = frame;
            r->from = prev;
            r->to = target;
            r->strict = strict;
            r->relax = view_relax(b);
            st->switches++;
            if (st->nlog < MAX_SWITCH_LOG) st->nlog++;
            st->level = target;
        }
    }
    return st->level;
}

/* record s (0 = first switch); returns 0 once it has fallen out of the log */
static int switch_record(const MultiState* st, int s, SwitchRecord* out) {
    int first = st->switches - st->nlog;
    if (s < first || s >= st->switches) return 0;
    *out = st->log[s % MAX_SWITCH_LOG];
    return 1;
}

int main(void) {
    MeshLod m = { {0.0f, 1.0f, 4.0f, 16.0f}, 2.0f, 1.1f };
    float fov = 1.0f;
    int h = 1080;
    float tol = 50.0f;

    /* --- 1. single viewport must match the legacy selector exactly --- */
    State legacy = {0, 0, 0};
    MultiState single = {0};
    int hist_old[LEVELS] = {0}, hist_new[LEVELS] = {0};
    int parity = 1;
    for (int i = 0; i < 40; i++) {
        float d = 5.0f + 5.0f * i;             /* receding */
        int a = select_level(&m, d, fov, h, tol, &legacy);
        View v = { d, fov, h, RELAX_MIN };
        int b = select_level_multi(&m, &v, 1, tol, i, &single);
        hist_old[a]++;
        hist_new[b]++;
        if (a != b) parity = 0;
    }
    for (int i = 0; i < LEVELS; i++) if (hist_old[i] != hist_new[i]) parity = 0;
    if (legacy.switches != single.switches) parity = 0;
    printf("single_legacy levels=%d,%d,%d,%d switches=%d\n",
           hist_old[0], hist_old[1], hist_old[2], hist_old[3], legacy.switches);
    printf("single_multi  levels=%d,%d,%d,%d switches=%d\n",
           hist_new[0], hist_new[1], hist_new[2], hist_new[3], single.switches);
    printf("single_parity=%d hysteresis_ok=%d\n", parity, legacy.switches <= 8);

    /* --- 2. split screen: quantify the difference before / after --- */
    View split[2] = {
        { 12.0f,  fov, h, RELAX_MIN },         /* near view */
        { 200.0f, fov, h, RELAX_MIN },         /* far view  */
    };
    int own[2];
    float err_own[2];
    for (int i = 0; i < 2; i++) {
        own[i] = want_level(&m, split[i].distance, split[i].fov_y,
                            split[i].screen_h, tol * view_relax(&split[i]));
        err_own[i] = screen_error(m.error[own[i]], split[i].distance,
                                  split[i].fov_y, split[i].screen_h);
    }
    int gap_before = own[0] > own[1] ? own[0] - own[1] : own[1] - own[0];
    MultiState split_st = {0};
    int shared = select_level_multi(&m, split, 2, tol, 0, &split_st);
    int levels_after[2] = { shared, shared };   /* one level, by construction */
    int gap_after = levels_after[0] > levels_after[1]
                  ? levels_after[0] - levels_after[1]
                  : levels_after[1] - levels_after[0];
    float err_shared[2];
    for (int i = 0; i < 2; i++)
        err_shared[i] = screen_error(m.error[shared], split[i].distance,
                                     split[i].fov_y, split[i].screen_h);
    printf("split_own levels=%d,%d err_px=%.2f,%.2f gap_before=%d\n",
           own[0], own[1], err_own[0], err_own[1], gap_before);
    printf("split_shared level=%d err_px=%.2f,%.2f gap_after=%d\n",
           shared, err_shared[0], err_shared[1], gap_after);

    /* --- 3. per-viewport relaxation changes the shared level --- */
    View relaxed[2] = {
        { 12.0f,  fov, h, 1.7f },              /* near view tolerates more */
        { 200.0f, fov, h, RELAX_MIN },
    };
    int strict_shared = LEVELS - 1;
    for (int i = 0; i < 2; i++) {
        int sw = want_level(&m, relaxed[i].distance, relaxed[i].fov_y,
                            relaxed[i].screen_h, tol);
        if (sw < strict_shared) strict_shared = sw;
    }
    MultiState relax_st = {0};
    int relaxed_shared = select_level_multi(&m, relaxed, 2, tol, 1, &relax_st);
    printf("relax_case strict_shared=%d relaxed_shared=%d eff=%.2f,%.2f\n",
           strict_shared, relaxed_shared,
           view_relax(&relaxed[0]), view_relax(&relaxed[1]));

    /* --- 4. requested relaxation is clamped to the bounds --- */
    View clamp_hi  = { 12.0f, fov, h, 100.0f };
    View clamp_lo  = { 12.0f, fov, h, 0.01f };
    View clamp_max = { 12.0f, fov, h, RELAX_MAX };
    MultiState hi_st = {0}, lo_st = {0}, max_st = {0};
    int lvl_hi  = select_level_multi(&m, &clamp_hi, 1, tol, 0, &hi_st);
    int lvl_lo  = select_level_multi(&m, &clamp_lo, 1, tol, 0, &lo_st);
    int lvl_max = select_level_multi(&m, &clamp_max, 1, tol, 0, &max_st);
    int bounds_ok = view_relax(&clamp_hi) == RELAX_MAX &&
                    view_relax(&clamp_lo) == RELAX_MIN &&
                    lvl_hi == lvl_max &&
                    lvl_hi > lvl_lo &&   /* relaxation only coarsens */
                    lvl_hi >= 0 && lvl_hi < LEVELS &&
                    lvl_lo >= 0 && lvl_lo < LEVELS;
    printf("relax_bounds req=100.00,0.01 eff=%.2f,%.2f levels=%d,%d,%d bounds_ok=%d\n",
           view_relax(&clamp_hi), view_relax(&clamp_lo),
           lvl_hi, lvl_lo, lvl_max, bounds_ok);

    /* --- 5. switching is recorded (split-screen receding cameras) --- */
    MultiState track = {0};
    for (int i = 0; i < 40; i++) {
        View vv[2] = {
            { 5.0f + 5.0f * i, fov, h, RELAX_MIN },
            { 7.0f + 7.0f * i, fov, h, 1.2f },
        };
        select_level_multi(&m, vv, 2, tol, i, &track);
    }
    int log_ok = track.switches > 0 && track.nlog > 0;
    int expect_from = 0;
    for (int s = 0; s < track.switches; s++) {
        SwitchRecord r;
        if (!switch_record(&track, s, &r)) { log_ok = 0; continue; }
        if (r.from != expect_from || r.from == r.to) log_ok = 0;
        if (r.to < 0 || r.to >= LEVELS) log_ok = 0;
        if (r.strict < 0 || r.strict >= LEVELS) log_ok = 0;
        if (r.relax < RELAX_MIN || r.relax > RELAX_MAX) log_ok = 0;
        expect_from = r.to;
    }
    printf("switch_records=%d switches=%d log_ok=%d\n",
           track.nlog, track.switches, log_ok);
    for (int s = track.switches - track.nlog; s < track.switches; s++) {
        SwitchRecord r = track.log[s % MAX_SWITCH_LOG];
        printf("  switch f=%d %d->%d strict=%d relax=%.2f\n",
               r.frame, r.from, r.to, r.strict, r.relax);
    }

    int ok = parity && gap_after == 0 && gap_before > 0 &&
             relaxed_shared > strict_shared && bounds_ok && log_ok;
    printf("all_ok=%d\n", ok);
    return ok ? 0 : 1;
}
