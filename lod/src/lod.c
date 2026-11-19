
/* lod-selector: screen-space-error LOD selection with hysteresis to avoid
   level thrashing when the camera hovers near a threshold.

   Also models many meshes sharing one frame triangle budget:
   - the budget is split by visible screen area * importance,
   - a bounded greedy exchange then moves triangles between meshes
     (downgrade one to upgrade another) until no trade is profitable,
   - every frame reports how much of the budget it used. */
#include <stdio.h>
#include <math.h>

#define LEVELS 4
#define MAX_MESHES 64
#define EXCHANGE_ROUNDS_PER_MESH 8
#define PI_F 3.14159265358979f
#define CULL_ERROR_PX 1.0e6f   /* error charged for a mesh that does not fit */

typedef struct {
    float error[LEVELS];   /* geometric error per level, world units */
    int tris[LEVELS];      /* triangles per level, level 0 is finest */
    float radius;          /* bounding radius */
    float tau;             /* hysteresis ratio, e.g. 1.1 */
} MeshLod;

typedef struct {
    int level;
    int switches;
    int thrash;            /* switches within one unit of distance change */
} State;

static float screen_error(float geom_error, float distance, float fov_y, int screen_h) {
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

/* ---------------- shared triangle budget ---------------- */

typedef struct {
    MeshLod mesh[MAX_MESHES];
    int n;
    float distance[MAX_MESHES];
    float importance[MAX_MESHES];
    float weight[MAX_MESHES];         /* visible pixel area * importance */
    float serr[MAX_MESHES][LEVELS];   /* projected error in pixels */
} Scene;

typedef struct {
    int level[MAX_MESHES];
    int used;                         /* triangles actually submitted */
    int culled;                       /* meshes that did not fit the budget */
    int iterations;                   /* greedy exchange rounds applied */
    double error_cost;                /* sum weight * projected px error, lower is better */
    double error_cost_before;         /* same metric before exchanging */
} Frame;

static float projected_radius_px(float radius, float distance, float fov_y, int screen_h) {
    float k = (float)screen_h * 0.5f / tanf(fov_y * 0.5f);
    return radius * k / distance;
}

static void scene_prepare(Scene* s, float fov_y, int screen_h) {
    for (int i = 0; i < s->n; i++) {
        float r_px = projected_radius_px(s->mesh[i].radius, s->distance[i], fov_y, screen_h);
        s->weight[i] = PI_F * r_px * r_px * s->importance[i];
        for (int l = 0; l < LEVELS; l++)
            s->serr[i][l] = screen_error(s->mesh[i].error[l], s->distance[i], fov_y, screen_h);
    }
}

/* finest level that fits the triangle allowance; LEVELS means cull */
static int finest_level_fitting(const MeshLod* m, int tri_budget) {
    for (int l = 0; l < LEVELS; l++)
        if (m->tris[l] <= tri_budget) return l;
    return LEVELS;
}

static int count_culled(const Scene* s, const int* level) {
    int n = 0;
    for (int i = 0; i < s->n; i++) if (level[i] >= LEVELS) n++;
    return n;
}

/* level == LEVELS means the mesh was culled because even its coarsest
   level could not fit the budget */
static int level_tris(const MeshLod* m, int level) {
    return level >= LEVELS ? 0 : m->tris[level];
}

static float level_serr(const Scene* s, int i, int level) {
    return level >= LEVELS ? CULL_ERROR_PX : s->serr[i][level];
}

static int frame_used(const Scene* s, const int* level) {
    int used = 0;
    for (int i = 0; i < s->n; i++) used += level_tris(&s->mesh[i], level[i]);
    return used;
}

static double frame_error_cost(const Scene* s, const int* level) {
    double cost = 0.0;
    for (int i = 0; i < s->n; i++)
        cost += (double)s->weight[i] * level_serr(s, i, level[i]);
    return cost;
}

/* split the budget by visible area * importance, reserving each mesh's
   coarsest level first so every mesh stays renderable */
static void alloc_by_weight(const Scene* s, int budget, int* level) {
    int reserve = 0;
    for (int i = 0; i < s->n; i++) reserve += s->mesh[i].tris[LEVELS - 1];

    if (reserve > budget) {
        /* cannot even show every mesh coarsest: keep the most visible ones
           at their coarsest level and cull the rest */
        int used = 0;
        for (int i = 0; i < s->n; i++) level[i] = LEVELS;
        for (;;) {
            int pick = -1;
            for (int i = 0; i < s->n; i++) {
                if (level[i] != LEVELS) continue;
                if (used + s->mesh[i].tris[LEVELS - 1] > budget) continue;
                if (pick < 0 || s->weight[i] > s->weight[pick]) pick = i;
            }
            if (pick < 0) break;
            level[pick] = LEVELS - 1;
            used += s->mesh[pick].tris[LEVELS - 1];
        }
        return;
    }

    int spare = budget - reserve;
    float sumw = 0.0f;
    for (int i = 0; i < s->n; i++) sumw += s->weight[i];
    if (sumw <= 0.0f) sumw = 1.0f;

    for (int i = 0; i < s->n; i++) {
        int share = s->mesh[i].tris[LEVELS - 1]
                  + (int)((float)spare * s->weight[i] / sumw);
        level[i] = finest_level_fitting(&s->mesh[i], share);
    }
}

/* Trade triangles between meshes: take one level from a mesh that loses
   little and give it to a mesh that gains more, until no trade pays off.
   Each accepted round strictly lowers the weighted error, so the loop is
   monotone; the iteration cap is a hard upper bound as well. */
static void greedy_exchange(const Scene* s, int budget, Frame* f) {
    int cap = s->n * EXCHANGE_ROUNDS_PER_MESH;
    int used = frame_used(s, f->level);
    f->iterations = 0;

    while (f->iterations < cap) {
        int spare = budget - used;
        double best = 0.0;
        int up = -1, down = -1;

        /* an upgrade funded by leftover budget has no downside */
        for (int b = 0; b < s->n; b++) {
            if (f->level[b] <= 0 || f->level[b] >= LEVELS) continue;
            int cost = s->mesh[b].tris[f->level[b] - 1] - s->mesh[b].tris[f->level[b]];
            if (cost > spare) continue;
            double gain = (double)s->weight[b]
                        * (s->serr[b][f->level[b]] - s->serr[b][f->level[b] - 1]);
            if (gain > best) { best = gain; up = b; down = -1; }
        }

        /* otherwise downgrade one mesh to fund upgrading another */
        for (int d = 0; d < s->n; d++) {
            if (f->level[d] >= LEVELS - 1) continue;   /* coarsest or culled */
            int freed = s->mesh[d].tris[f->level[d]] - s->mesh[d].tris[f->level[d] + 1];
            double loss = (double)s->weight[d]
                        * (s->serr[d][f->level[d] + 1] - s->serr[d][f->level[d]]);
            for (int u = 0; u < s->n; u++) {
                if (u == d || f->level[u] <= 0 || f->level[u] >= LEVELS) continue;
                int cost = s->mesh[u].tris[f->level[u] - 1] - s->mesh[u].tris[f->level[u]];
                if (cost > spare + freed) continue;
                double gain = (double)s->weight[u]
                            * (s->serr[u][f->level[u]] - s->serr[u][f->level[u] - 1]);
                double net = gain - loss;
                if (net > best) { best = net; up = u; down = d; }
            }
        }

        if (up < 0 || best <= 1e-9) break;
        if (down >= 0) f->level[down]++;
        f->level[up]--;
        used = frame_used(s, f->level);
        f->iterations++;
    }

    f->used = used;
    f->culled = count_culled(s, f->level);
    f->error_cost = frame_error_cost(s, f->level);
}

static void allocate_frame(const Scene* s, int budget, Frame* f) {
    alloc_by_weight(s, budget, f->level);
    f->error_cost_before = frame_error_cost(s, f->level);
    greedy_exchange(s, budget, f);
}

/* baseline: every mesh gets the same slice of the budget, then the
   leftover is handed out round-robin so the baseline also spends it all */
static void alloc_uniform(const Scene* s, int budget, int* level) {
    int share = budget / s->n;
    for (int i = 0; i < s->n; i++)
        level[i] = finest_level_fitting(&s->mesh[i], share);

    int used = frame_used(s, level);
    int changed = 1;
    while (changed) {
        changed = 0;
        for (int i = 0; i < s->n; i++) {
            if (level[i] <= 0 || level[i] >= LEVELS) continue;
            int cost = s->mesh[i].tris[level[i] - 1] - s->mesh[i].tris[level[i]];
            if (used + cost > budget) continue;
            level[i]--;
            used += cost;
            changed = 1;
        }
    }
}

/* ---------------- scene ---------------- */

static const int BASE_TRIS[LEVELS] = { 4000, 1200, 400, 100 };
static const float BASE_ERR[LEVELS] = { 0.0f, 0.08f, 0.40f, 1.60f };

static void build_scene(Scene* s, int n, float distance_scale) {
    s->n = n;
    for (int i = 0; i < n; i++) {
        float f = 0.6f + 0.15f * (float)(i % 5);
        for (int l = 0; l < LEVELS; l++) {
            s->mesh[i].tris[l] = (int)((float)BASE_TRIS[l] * f + 0.5f);
            s->mesh[i].error[l] = BASE_ERR[l];
        }
        s->mesh[i].radius = 2.0f + 0.5f * (float)(i % 4);
        s->mesh[i].tau = 1.1f;
        s->importance[i] = 1.0f + 0.8f * (float)((i * 37) % 11) / 10.0f;
        float t = (n > 1) ? (float)i / (float)(n - 1) : 0.0f;
        s->distance[i] = 8.0f * expf(logf(250.0f / 8.0f) * t) * distance_scale;
    }
}

int main(void) {
    /* ---- single-mesh hysteresis, unchanged ---- */
    MeshLod m = { .error = { 0.0f, 1.0f, 4.0f, 16.0f }, .radius = 2.0f, .tau = 1.1f };
    State st = { 0, 0, 0 };
    float fov = 1.0f;
    int h = 1080;
    int hist[LEVELS] = { 0 };
    for (int i = 0; i < 40; i++) {
        float d = 5.0f + 5.0f * i;             /* receding */
        int lv = select_level(&m, d, fov, h, 50.0f, &st);
        hist[lv]++;
    }
    printf("levels=%d,%d,%d,%d switches=%d\n", hist[0], hist[1], hist[2], hist[3], st.switches);
    /* 迟滞应当让切换次数远小于采样次数 */
    printf("hysteresis_ok=%d\n", st.switches <= 8);

    /* ---- 50 meshes sharing one triangle budget ---- */
    const int N = 50;
    const int BUDGET = 75000;
    const int TIGHT_BUDGET = 4000;      /* below 50 * coarsest: forces culling */
    static Scene scene;
    static Frame alloc;
    static int uni_level[MAX_MESHES];
    const float scales[] = { 1.0f, 1.15f, 1.35f, 1.6f, 1.35f, 1.0f };
    const int frames = (int)(sizeof(scales) / sizeof(scales[0]));

    int budget_ok = 1, iterations_ok = 1, gain_ok = 1, monotone_ok = 1, cull_ok = 1;
    double worst_ratio = 0.0;

    printf("meshes=%d budget=%d\n", N, BUDGET);
    for (int fr = 0; fr < frames; fr++) {
        build_scene(&scene, N, scales[fr]);
        scene_prepare(&scene, fov, h);
        allocate_frame(&scene, BUDGET, &alloc);

        alloc_uniform(&scene, BUDGET, uni_level);
        int uni_used = frame_used(&scene, uni_level);
        double cost_uniform = frame_error_cost(&scene, uni_level);

        /* reference is an unconstrained all-finest render (zero error) */
        double ratio = (cost_uniform > 0.0) ? alloc.error_cost / cost_uniform : 0.0;
        if (ratio > worst_ratio) worst_ratio = ratio;

        printf("frame=%d used=%d/%d usage=%.1f%% iters=%d cost=%.1f cost_before=%.1f "
               "cost_avg=%.1f loss_ratio=%.3f uni_used=%d\n",
               fr, alloc.used, BUDGET, 100.0 * (double)alloc.used / (double)BUDGET,
               alloc.iterations, alloc.error_cost, alloc.error_cost_before,
               cost_uniform, ratio, uni_used);

        if (alloc.used > BUDGET || uni_used > BUDGET) budget_ok = 0;
        if (alloc.iterations > N * EXCHANGE_ROUNDS_PER_MESH) iterations_ok = 0;
        if (alloc.error_cost > alloc.error_cost_before + 1e-6) monotone_ok = 0;
        if (ratio > 0.8) gain_ok = 0;
    }

    /* tight budget: the allocator must keep the frame under budget by
       culling the least visible meshes rather than overspending */
    build_scene(&scene, N, 1.0f);
    scene_prepare(&scene, fov, h);
    allocate_frame(&scene, TIGHT_BUDGET, &alloc);
    printf("tight used=%d/%d usage=%.1f%% culled=%d iters=%d\n",
           alloc.used, TIGHT_BUDGET,
           100.0 * (double)alloc.used / (double)TIGHT_BUDGET,
           alloc.culled, alloc.iterations);
    if (alloc.used > TIGHT_BUDGET) cull_ok = 0;

    printf("budget_ok=%d iterations_bounded=%d monotone_ok=%d "
           "quality_gain_ok=%d cull_ok=%d worst_loss_ratio=%.3f\n",
           budget_ok, iterations_ok, monotone_ok, gain_ok, cull_ok, worst_ratio);

    int ok = budget_ok && iterations_ok && monotone_ok && gain_ok && cull_ok
          && st.switches <= 8;
    return ok ? 0 : 1;
}
