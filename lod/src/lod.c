
/* lod-selector: screen-space-error LOD selection with hysteresis to avoid
   level thrashing when the camera hovers near a threshold, plus a hard
   per-frame triangle budget.

   Budget rule: a frame's proposal is accepted only if the sum of the chosen
   levels' triangle counts fits the budget.  Otherwise the whole proposal is
   rejected and the selector rolls back to the previous committed set (or,
   when there is none / it no longer fits a shrunken budget, to the
   deterministic coarsest-level set).  Every rollback is logged, and the
   emitted total never exceeds the budget. */
#include <stdio.h>
#include <limits.h>
#include <math.h>

#define LEVELS 4
#define MAX_MESHES 16
#define MAX_LOG 128

typedef struct {
    float error[LEVELS];   /* geometric error per level, world units */
    float radius;          /* bounding radius */
    float tau;             /* hysteresis ratio, e.g. 1.1 */
    int tris[LEVELS];      /* triangles per level; level 0 finest */
} MeshLod;

typedef struct {
    int level;
    int switches;
} LodState;

typedef struct {
    int level[MAX_MESHES];
    long long tris;        /* 0 when invalid */
    int valid;             /* 0 = no feasible set could be emitted */
    int rolled_back;
    int rollback_reason;
} LodSet;

enum { RB_PREVIOUS = 0, RB_COARSEST = 1, RB_NONE = 2 };

typedef struct {
    int frame;
    long long candidate_tris;
    long long budget;
    long long used_tris;
    int reason;
} RollbackEntry;

typedef struct {
    int n;
    LodState mesh[MAX_MESHES];
    LodSet committed;      /* last emitted feasible set */
    int frame;
    int over_budget_frames;
    int rollbacks;
    int no_feasible_frames;
    RollbackEntry log[MAX_LOG];
    int log_count;
    FILE* log_out;         /* NULL disables textual logging */
} Selector;

static float screen_error(float geom_error, float distance, float fov_y, int screen_h) {
    if (distance <= 0.0f) return 1e9f;
    return geom_error * (float)screen_h / (2.0f * distance * tanf(fov_y * 0.5f));
}

/* Pure selection: no state is modified, so a rejected proposal cannot poison
   the hysteresis state.  Returns the level this mesh would use. */
static int choose_level(const MeshLod* m, float distance, float fov_y, int screen_h,
                        float tolerance_px, int prev) {
    int want = 0;
    for (int i = LEVELS - 1; i >= 0; i--) {
        float e = screen_error(m->error[i], distance, fov_y, screen_h);
        if (e <= tolerance_px) { want = i; break; }
    }
    if (want == prev) return prev;
    /* hysteresis: require the candidate to beat the threshold by tau */
    float e_new = screen_error(m->error[want], distance, fov_y, screen_h);
    float e_old = screen_error(m->error[prev], distance, fov_y, screen_h);
    if (want > prev && e_new > tolerance_px * m->tau) return prev;
    if (want < prev && e_old < tolerance_px / m->tau) return prev;
    return want;
}

static void selector_init(Selector* s, int n, FILE* log_out) {
    s->n = n;
    for (int m = 0; m < n; m++) {
        s->mesh[m].level = 0;
        s->mesh[m].switches = 0;
        s->committed.level[m] = 0;
    }
    s->committed.tris = 0;
    s->committed.valid = 0;
    s->committed.rolled_back = 0;
    s->committed.rollback_reason = -1;
    s->frame = 0;
    s->over_budget_frames = 0;
    s->rollbacks = 0;
    s->no_feasible_frames = 0;
    s->log_count = 0;
    s->log_out = log_out;
}

static void commit_set(Selector* sel, const int* levels, long long tris) {
    for (int m = 0; m < sel->n; m++) {
        if (sel->mesh[m].level != levels[m]) sel->mesh[m].switches++;
        sel->mesh[m].level = levels[m];
        sel->committed.level[m] = levels[m];
    }
    sel->committed.tris = tris;
    sel->committed.valid = 1;
}

static void record_rollback(Selector* sel, long long candidate_tris,
                            long long budget, long long used_tris, int reason) {
    RollbackEntry e = { sel->frame, candidate_tris, budget, used_tris, reason };
    if (sel->log_count < MAX_LOG) sel->log[sel->log_count] = e;
    sel->log_count++;
    if (sel->log_out) {
        const char* why = reason == RB_PREVIOUS ? "previous committed set"
                        : reason == RB_COARSEST ? "coarsest feasible set"
                        : "no feasible set";
        fprintf(sel->log_out,
                "rollback frame=%d cand_tris=%lld budget=%lld used_tris=%lld (%s)\n",
                e.frame, e.candidate_tris, e.budget, e.used_tris, why);
    }
}

/* Advance one frame.  Returns the set actually emitted: the accepted
   proposal, the rolled-back set, or an invalid set (nothing) when even the
   coarsest configuration exceeds the budget. */
static LodSet selector_step(Selector* sel, const MeshLod* meshes, const float* distances,
                            float fov_y, int screen_h, float tolerance_px,
                            long long budget) {
    if (budget < 0) budget = 0;

    int cand_levels[MAX_MESHES];
    long long cand_tris = 0;
    for (int m = 0; m < sel->n; m++) {
        cand_levels[m] = choose_level(&meshes[m], distances[m], fov_y, screen_h,
                                      tolerance_px, sel->mesh[m].level);
        cand_tris += meshes[m].tris[cand_levels[m]];
    }

    LodSet out;
    if (cand_tris <= budget) {
        commit_set(sel, cand_levels, cand_tris);
        out = sel->committed;
        out.rolled_back = 0;
        out.rollback_reason = -1;
        sel->frame++;
        return out;
    }

    /* Hard cap: reject the entire proposal, then roll back. */
    sel->over_budget_frames++;
    sel->rollbacks++;

    if (sel->committed.valid && sel->committed.tris <= budget) {
        out = sel->committed;
        out.rolled_back = 1;
        out.rollback_reason = RB_PREVIOUS;
        record_rollback(sel, cand_tris, budget, out.tris, RB_PREVIOUS);
    } else {
        /* No usable previous set: deterministically fall back to the
           coarsest level for every mesh, if that fits. */
        int coarse[MAX_MESHES];
        long long coarse_tris = 0;
        for (int m = 0; m < sel->n; m++) {
            coarse[m] = LEVELS - 1;
            coarse_tris += meshes[m].tris[LEVELS - 1];
        }
        if (coarse_tris <= budget) {
            commit_set(sel, coarse, coarse_tris);
            out = sel->committed;
            out.rolled_back = 1;
            out.rollback_reason = RB_COARSEST;
            record_rollback(sel, cand_tris, budget, out.tris, RB_COARSEST);
        } else {
            sel->no_feasible_frames++;
            sel->committed.valid = 0;
            sel->committed.tris = 0;
            for (int m = 0; m < sel->n; m++) {
                out.level[m] = LEVELS - 1;
                sel->committed.level[m] = LEVELS - 1;
            }
            out.tris = 0;
            out.valid = 0;
            out.rolled_back = 1;
            out.rollback_reason = RB_NONE;
            record_rollback(sel, cand_tris, budget, 0, RB_NONE);
        }
    }
    sel->frame++;
    return out;
}

/* ------------------------------------------------------------------ tests */

#define SCENE_MESHES 6
#define SCENE_FRAMES 40
#define SCENE_BUDGET 4000LL

static void scene_distances(float* d, int frame) {
    for (int m = 0; m < SCENE_MESHES; m++)
        d[m] = 5.0f + 5.0f * frame + 3.0f * m;
}

static void run_scene(const MeshLod* mesh, long long budget, Selector* sel,
                      LodSet* out, FILE* log_out) {
    selector_init(sel, SCENE_MESHES, log_out);
    for (int f = 0; f < SCENE_FRAMES; f++) {
        float d[SCENE_MESHES];
        scene_distances(d, f);
        out[f] = selector_step(sel, mesh, d, 1.0f, 1080, 50.0f, budget);
    }
}

static int same_set(const LodSet* a, const LodSet* b) {
    if (a->valid != b->valid || a->tris != b->tris) return 0;
    for (int m = 0; m < SCENE_MESHES; m++) if (a->level[m] != b->level[m]) return 0;
    return 1;
}

int main(void) {
    MeshLod hm = { {0.0f, 1.0f, 4.0f, 16.0f}, 2.0f, 1.1f, {4000, 1000, 250, 60} };

    /* --- existing hysteresis scenario (budget non-binding) --- */
    int hist[LEVELS] = {0};
    {
        Selector hs;
        selector_init(&hs, 1, NULL);
        for (int i = 0; i < 40; i++) {
            float d[1] = { 5.0f + 5.0f * i };   /* receding */
            LodSet o = selector_step(&hs, &hm, d, 1.0f, 1080, 50.0f, LLONG_MAX);
            hist[o.level[0]]++;
        }
        printf("levels=%d,%d,%d,%d switches=%d\n",
               hist[0], hist[1], hist[2], hist[3], hs.mesh[0].switches);
        /* 迟滞应当让切换次数远小于采样次数 */
        printf("hysteresis_ok=%d\n", hs.mesh[0].switches <= 8);
    }

    /* --- hard-budget scenario --- */
    MeshLod scene[SCENE_MESHES];
    for (int m = 0; m < SCENE_MESHES; m++) scene[m] = hm;

    Selector s1, s2;
    LodSet r1[SCENE_FRAMES], r2[SCENE_FRAMES];
    run_scene(scene, SCENE_BUDGET, &s1, r1, stdout);
    run_scene(scene, SCENE_BUDGET, &s2, r2, NULL);

    int cap_ok = 1;
    int fallback_ok = 1;
    int rollback_frames = 0;
    for (int f = 0; f < SCENE_FRAMES; f++) {
        if (r1[f].rolled_back) rollback_frames++;
        if (r1[f].valid) {
            if (r1[f].tris > SCENE_BUDGET) cap_ok = 0;      /* 任意帧都不超预算 */
        } else if (r1[f].tris != 0) {
            cap_ok = 0;
        }
        /* 回退到上一组可行解：结果必须与上一帧发射的集合完全相同 */
        if (r1[f].rolled_back && r1[f].rollback_reason == RB_PREVIOUS) {
            if (f == 0 || !r1[f - 1].valid || !same_set(&r1[f], &r1[f - 1])) fallback_ok = 0;
        }
    }

    int determinism_ok = 1;
    for (int f = 0; f < SCENE_FRAMES; f++)
        if (!same_set(&r1[f], &r2[f])) determinism_ok = 0;

    int rollback_count_ok =
        s1.rollbacks == s1.over_budget_frames &&
        rollback_frames == s1.over_budget_frames &&
        s1.log_count == s1.rollbacks;
    int log_ok = s1.log_count == s1.rollbacks && s1.rollbacks > 0;

    printf("budget frames=%d over_budget=%d rollbacks=%d\n",
           SCENE_FRAMES, s1.over_budget_frames, s1.rollbacks);
    printf("budget_cap_ok=%d\n", cap_ok);
    printf("fallback_prev_ok=%d\n", fallback_ok);
    printf("rollback_count_ok=%d\n", rollback_count_ok);
    printf("rollback_log_ok=%d\n", log_ok);
    printf("determinism_ok=%d\n", determinism_ok);

    /* --- budget sweep: every budget must respect the cap --- */
    int sweep_cap_ok = 1, sweep_count_ok = 1, sweep_det_ok = 1;
    for (long long b = 0; b <= 6000; b += 37) {
        Selector sa, sb;
        LodSet ra[SCENE_FRAMES], rb[SCENE_FRAMES];
        run_scene(scene, b, &sa, ra, NULL);
        run_scene(scene, b, &sb, rb, NULL);
        for (int f = 0; f < SCENE_FRAMES; f++) {
            if (ra[f].valid ? (ra[f].tris > b) : (ra[f].tris != 0)) sweep_cap_ok = 0;
            if (!same_set(&ra[f], &rb[f])) sweep_det_ok = 0;
        }
        if (sa.rollbacks != sa.over_budget_frames || sa.log_count != sa.rollbacks)
            sweep_count_ok = 0;
    }
    printf("budget_sweep_ok=%d\n", sweep_cap_ok);
    printf("budget_sweep_rollback_ok=%d\n", sweep_count_ok);
    printf("budget_sweep_determinism_ok=%d\n", sweep_det_ok);

    /* --- varying budget (shrink can invalidate the committed set) --- */
    int varied_cap_ok = 1;
    {
        Selector sv;
        selector_init(&sv, SCENE_MESHES, NULL);
        for (int f = 0; f < SCENE_FRAMES; f++) {
            float d[SCENE_MESHES];
            scene_distances(d, f);
            long long b = 4200 - (long long)(f * 331) % 4201;   /* deterministic */
            LodSet o = selector_step(&sv, scene, d, 1.0f, 1080, 50.0f, b);
            if (o.valid ? (o.tris > b) : (o.tris != 0)) varied_cap_ok = 0;
        }
        if (sv.rollbacks != sv.over_budget_frames) varied_cap_ok = 0;
    }
    printf("varying_budget_ok=%d\n", varied_cap_ok);
    return 0;
}
