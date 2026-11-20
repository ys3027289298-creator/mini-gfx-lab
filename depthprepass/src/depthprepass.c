/* depthprepass: software rasterizer comparing naive forward drawing with a
   depth prepass. Opaque primitives first write depth only, then shade
   front-to-back against that depth, and blended primitives get their own
   back-to-front pass with depth writes disabled. The two paths are compared
   pixel by pixel; shading overdraw, prepass cost and the opaque/blended
   partition are all reported and checked. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define FB_W 320
#define FB_H 240
#define MAX_PRIMS 256
#define OPAQUE_PRIMS 120
#define BLEND_PRIMS 16

enum { MAT_OPAQUE = 0, MAT_BLEND = 1 };

typedef struct { float x, y, z, r, g, b, a; } Vert;

typedef struct {
    Vert v[3];
    int material;
    int id;
} Prim;

typedef struct {
    Prim prims[MAX_PRIMS];
    int count;
} Scene;

typedef struct {
    long tested;        /* fragments that passed coverage */
    long shaded;        /* fragments that ran the colour shader */
    long depth_fail;    /* covered fragments rejected by the depth test */
    long depth_writes;
    long color_writes;
    long blend_reads;
} Stats;

typedef struct {
    float depth[FB_H][FB_W];
    float color[FB_H][FB_W][4];
} Target;

typedef struct {
    int write_color;
    int write_depth;
    int blend;
} RasterMode;

typedef struct {
    Stats prepass;      /* depth-only pass, zero for direct drawing */
    Stats opaque;       /* opaque shading pass */
    Stats transparent;  /* blended pass */
    int opaque_prims;
    int transparent_prims;
} Frame;

enum {
    PE_COUNT = 1,
    PE_DUP = 2,
    PE_MATERIAL = 4,
    PE_ALPHA = 8,
    PE_RANGE = 16
};

static Target g_direct;
static Target g_sorted;
static Target g_prepass;

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static float edge_fn(float ax, float ay, float bx, float by, float px, float py) {
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
}

static void target_clear(Target* t) {
    memset(t->color, 0, sizeof(t->color));
    for (int y = 0; y < FB_H; y++)
        for (int x = 0; x < FB_W; x++)
            t->depth[y][x] = 1.0f;
}

/* One triangle, screen-space x/y/z with y down and z in [0,1], near = small.
   The same routine serves every pass; the mode decides depth writes, colour
   writes and blending. */
static void raster_tri(Target* t, const Vert* a, const Vert* b, const Vert* c,
                       const RasterMode* m, Stats* st) {
    float area = edge_fn(a->x, a->y, b->x, b->y, c->x, c->y);
    if (area == 0.0f) return;
    if (area < 0.0f) {
        const Vert* tmp = b; b = c; c = tmp;
        area = -area;
    }

    float minx = fminf(a->x, fminf(b->x, c->x));
    float maxx = fmaxf(a->x, fmaxf(b->x, c->x));
    float miny = fminf(a->y, fminf(b->y, c->y));
    float maxy = fmaxf(a->y, fmaxf(b->y, c->y));

    int x0 = (int)floorf(minx), x1 = (int)ceilf(maxx);
    int y0 = (int)floorf(miny), y1 = (int)ceilf(maxy);
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > FB_W) x1 = FB_W;
    if (y1 > FB_H) y1 = FB_H;

    for (int y = y0; y < y1; y++) {
        float py = (float)y + 0.5f;
        for (int x = x0; x < x1; x++) {
            float px = (float)x + 0.5f;
            float w0 = edge_fn(b->x, b->y, c->x, c->y, px, py);
            float w1 = edge_fn(c->x, c->y, a->x, a->y, px, py);
            float w2 = edge_fn(a->x, a->y, b->x, b->y, px, py);
            if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) continue;
            w0 /= area; w1 /= area; w2 /= area;

            float z = w0 * a->z + w1 * b->z + w2 * c->z;
            st->tested++;
            if (z > t->depth[y][x]) { st->depth_fail++; continue; }

            if (m->write_color) {
                float r = w0 * a->r + w1 * b->r + w2 * c->r;
                float g = w0 * a->g + w1 * b->g + w2 * c->g;
                float bl = w0 * a->b + w1 * b->b + w2 * c->b;
                float al = w0 * a->a + w1 * b->a + w2 * c->a;
                float* dst = t->color[y][x];
                if (m->blend) {
                    float inv = 1.0f - al;
                    dst[0] = r * al + dst[0] * inv;
                    dst[1] = g * al + dst[1] * inv;
                    dst[2] = bl * al + dst[2] * inv;
                    dst[3] = al + dst[3] * inv;
                    st->blend_reads++;
                } else {
                    dst[0] = r; dst[1] = g; dst[2] = bl; dst[3] = al;
                }
                st->shaded++;
                st->color_writes++;
            }
            if (m->write_depth) {
                t->depth[y][x] = z;
                st->depth_writes++;
            }
        }
    }
}

static void draw_prim(Target* t, const Scene* s, int idx, const RasterMode* m, Stats* st) {
    const Prim* p = &s->prims[idx];
    raster_tri(t, &p->v[0], &p->v[1], &p->v[2], m, st);
}

static float prim_z(const Scene* s, int idx) {
    const Prim* p = &s->prims[idx];
    return (p->v[0].z + p->v[1].z + p->v[2].z) / 3.0f;
}

/* stable insertion sort on centroid depth; front_to_back = nearest first */
static void sort_by_z(const Scene* s, int idx[], int n, int front_to_back) {
    for (int i = 1; i < n; i++) {
        int cur = idx[i];
        float zc = prim_z(s, cur);
        int j = i - 1;
        while (j >= 0) {
            float zj = prim_z(s, idx[j]);
            int out_of_order = front_to_back ? (zc < zj) : (zc > zj);
            if (!out_of_order) break;
            idx[j + 1] = idx[j];
            j--;
        }
        idx[j + 1] = cur;
    }
}

static int order_front_to_back(const Scene* s, const int idx[], int n) {
    for (int i = 1; i < n; i++)
        if (prim_z(s, idx[i]) < prim_z(s, idx[i - 1])) return 0;
    return 1;
}

static int order_back_to_front(const Scene* s, const int idx[], int n) {
    for (int i = 1; i < n; i++)
        if (prim_z(s, idx[i]) > prim_z(s, idx[i - 1])) return 0;
    return 1;
}

static int partition_scene(const Scene* s, int opaque[], int* n_op,
                           int blend[], int* n_bl) {
    *n_op = 0;
    *n_bl = 0;
    for (int i = 0; i < s->count; i++) {
        if (s->prims[i].material == MAT_OPAQUE) opaque[(*n_op)++] = i;
        else if (s->prims[i].material == MAT_BLEND) blend[(*n_bl)++] = i;
        else return -1;
    }
    return 0;
}

static int prim_alpha_is(const Prim* p, float want) {
    for (int k = 0; k < 3; k++)
        if (p->v[k].a != want) return 0;
    return 1;
}

/* Checks that the two lists are a complete, duplicate-free partition of the
   scene and that each primitive sits on the side its alpha demands. */
static int validate_partition(const Scene* s, const int opaque[], int n_op,
                              const int blend[], int n_bl) {
    int err = 0;
    int seen[MAX_PRIMS] = {0};

    if (n_op + n_bl != s->count) err |= PE_COUNT;
    for (int i = 0; i < s->count; i++) {
        int m = s->prims[i].material;
        if (m != MAT_OPAQUE && m != MAT_BLEND) err |= PE_MATERIAL;
    }

    for (int i = 0; i < n_op; i++) {
        int idx = opaque[i];
        if (idx < 0 || idx >= s->count) { err |= PE_RANGE; continue; }
        if (seen[idx]) err |= PE_DUP;
        seen[idx] = 1;
        const Prim* p = &s->prims[idx];
        if (p->material != MAT_OPAQUE) err |= PE_MATERIAL;
        if (!prim_alpha_is(p, 1.0f)) err |= PE_ALPHA;
    }
    for (int i = 0; i < n_bl; i++) {
        int idx = blend[i];
        if (idx < 0 || idx >= s->count) { err |= PE_RANGE; continue; }
        if (seen[idx]) err |= PE_DUP;
        seen[idx] = 1;
        const Prim* p = &s->prims[idx];
        if (p->material != MAT_BLEND) err |= PE_MATERIAL;
        int a = p->v[0].a > 0.0f && p->v[0].a < 1.0f;
        a = a && prim_alpha_is(p, p->v[0].a);
        if (!a) err |= PE_ALPHA;
    }
    for (int i = 0; i < s->count; i++)
        if (!seen[i]) err |= PE_COUNT;
    return err;
}

/* direct drawing: opaque in submission order with depth test/write, then
   blended primitives back-to-front without depth writes */
static void render_direct(const Scene* s, Target* t, int sort_opaque, Frame* f) {
    int op[MAX_PRIMS], bl[MAX_PRIMS], n_op, n_bl;
    partition_scene(s, op, &n_op, bl, &n_bl);
    if (sort_opaque) sort_by_z(s, op, n_op, 1);

    RasterMode opaque = { 1, 1, 0 };
    for (int i = 0; i < n_op; i++)
        draw_prim(t, s, op[i], &opaque, &f->opaque);

    sort_by_z(s, bl, n_bl, 0);
    RasterMode blend = { 1, 0, 1 };
    for (int i = 0; i < n_bl; i++)
        draw_prim(t, s, bl[i], &blend, &f->transparent);

    f->opaque_prims = n_op;
    f->transparent_prims = n_bl;
}

/* depth prepass: opaque depth only, then opaque colour front-to-back against
   that depth, then the same blended pass as direct drawing */
static void render_with_prepass(const Scene* s, Target* t, Frame* f) {
    int op[MAX_PRIMS], bl[MAX_PRIMS], n_op, n_bl;
    partition_scene(s, op, &n_op, bl, &n_bl);

    RasterMode depth_only = { 0, 1, 0 };
    for (int i = 0; i < n_op; i++)
        draw_prim(t, s, op[i], &depth_only, &f->prepass);

    sort_by_z(s, op, n_op, 1);
    RasterMode opaque = { 1, 0, 0 };
    for (int i = 0; i < n_op; i++)
        draw_prim(t, s, op[i], &opaque, &f->opaque);

    sort_by_z(s, bl, n_bl, 0);
    RasterMode blend = { 1, 0, 1 };
    for (int i = 0; i < n_bl; i++)
        draw_prim(t, s, bl[i], &blend, &f->transparent);

    f->opaque_prims = n_op;
    f->transparent_prims = n_bl;
}

static unsigned rng_state;

static float rnd_unit(void) {
    rng_state = rng_state * 1664525u + 1013904223u;
    return (float)(rng_state >> 8) / 16777216.0f;
}

/* Occlusion-heavy scene: large overlapping triangles. Submission order is
   farthest first, the worst case for naive forward drawing. */
static void scene_build(Scene* s, unsigned seed) {
    Scene tmp;
    memset(&tmp, 0, sizeof(tmp));
    tmp.count = OPAQUE_PRIMS + BLEND_PRIMS;
    rng_state = seed;
    for (int i = 0; i < tmp.count; i++) {
        Prim* p = &tmp.prims[i];
        p->id = i;
        p->material = (i < OPAQUE_PRIMS) ? MAT_OPAQUE : MAT_BLEND;

        float cx = FB_W * (0.30f + 0.40f * rnd_unit());
        float cy = FB_H * (0.30f + 0.40f * rnd_unit());
        float zc = 0.05f + 0.90f * rnd_unit();
        float base = 6.28318530718f * rnd_unit();
        float rad = (0.15f + 0.30f * rnd_unit()) * (float)FB_W;
        float alpha = (p->material == MAT_OPAQUE) ? 1.0f : 0.25f + 0.35f * rnd_unit();

        for (int k = 0; k < 3; k++) {
            float ang = base + (float)k * 2.09439510239f + (rnd_unit() - 0.5f);
            float r = rad * (0.75f + 0.5f * rnd_unit());
            Vert* v = &p->v[k];
            v->x = cx + cosf(ang) * r;
            v->y = cy + sinf(ang) * r;
            v->z = zc + (rnd_unit() - 0.5f) * 0.02f;
            v->r = 0.10f + 0.80f * rnd_unit();
            v->g = 0.10f + 0.80f * rnd_unit();
            v->b = 0.10f + 0.80f * rnd_unit();
            v->a = alpha;
        }
    }

    int order[MAX_PRIMS];
    for (int i = 0; i < tmp.count; i++) order[i] = i;
    sort_by_z(&tmp, order, tmp.count, 0);
    memset(s, 0, sizeof(*s));
    s->count = tmp.count;
    for (int i = 0; i < tmp.count; i++) s->prims[i] = tmp.prims[order[i]];
}

static int covered_pixels(const Target* t) {
    int n = 0;
    for (int y = 0; y < FB_H; y++)
        for (int x = 0; x < FB_W; x++)
            if (t->depth[y][x] < 1.0f) n++;
    return n;
}

static void compare_targets(const Target* a, const Target* b,
                            int* color_diff_px, float* max_diff,
                            int* depth_diff_px) {
    *color_diff_px = 0;
    *max_diff = 0.0f;
    *depth_diff_px = 0;
    for (int y = 0; y < FB_H; y++) {
        for (int x = 0; x < FB_W; x++) {
            int diff = 0;
            for (int c = 0; c < 4; c++) {
                float d = fabsf(a->color[y][x][c] - b->color[y][x][c]);
                if (d != 0.0f) diff = 1;
                if (d > *max_diff) *max_diff = d;
            }
            if (diff) (*color_diff_px)++;
            if (a->depth[y][x] != b->depth[y][x]) (*depth_diff_px)++;
        }
    }
}

static long frame_tests(const Frame* f) {
    return f->prepass.tested + f->opaque.tested + f->transparent.tested;
}

static long frame_shaded(const Frame* f) {
    return f->opaque.shaded + f->transparent.shaded;
}

static int stats_same(const Stats* a, const Stats* b) {
    return a->tested == b->tested && a->shaded == b->shaded &&
           a->depth_fail == b->depth_fail && a->depth_writes == b->depth_writes &&
           a->color_writes == b->color_writes && a->blend_reads == b->blend_reads;
}

int main(void) {
    static Scene sc;
    scene_build(&sc, 0x5eed1234u);

    int op[MAX_PRIMS], bl[MAX_PRIMS], n_op, n_bl;
    int part_rc = partition_scene(&sc, op, &n_op, bl, &n_bl);
    int part_err = validate_partition(&sc, op, n_op, bl, n_bl);

    target_clear(&g_direct);
    target_clear(&g_sorted);
    target_clear(&g_prepass);

    Frame direct = { {0}, {0}, {0}, 0, 0 };
    Frame sorted = { {0}, {0}, {0}, 0, 0 };
    Frame pre = { {0}, {0}, {0}, 0, 0 };

    render_direct(&sc, &g_direct, 0, &direct);
    render_direct(&sc, &g_sorted, 1, &sorted);
    render_with_prepass(&sc, &g_prepass, &pre);

    int covered = covered_pixels(&g_direct);
    int color_diff = 0, depth_diff = 0, sorted_color_diff = 0, sorted_depth_diff = 0;
    float max_diff = 0.0f, sorted_max_diff = 0.0f;
    compare_targets(&g_direct, &g_prepass, &color_diff, &max_diff, &depth_diff);
    compare_targets(&g_sorted, &g_prepass, &sorted_color_diff, &sorted_max_diff,
                    &sorted_depth_diff);

    double direct_overdraw = (double)direct.opaque.shaded / (double)covered;
    double pre_overdraw = (double)pre.opaque.shaded / (double)covered;
    double reduction_pct = 100.0 * (double)(direct.opaque.shaded - pre.opaque.shaded) /
                           (double)direct.opaque.shaded;

    long direct_tests = frame_tests(&direct);
    long pre_tests = frame_tests(&pre);
    long extra_tests = pre.prepass.tested;
    long extra_depth_writes = pre.prepass.depth_writes;
    double extra_pct = 100.0 * (double)extra_tests / (double)direct_tests;

    /* Simple cost model: one fragment test costs 1, one shaded fragment costs
       SHADE_WEIGHT. This is conservative; real shading is usually dearer. */
    const long SHADE_WEIGHT = 8;
    long direct_cost = direct_tests + SHADE_WEIGHT * frame_shaded(&direct);
    long pre_cost = pre_tests + SHADE_WEIGHT * frame_shaded(&pre);

    printf("screen_px=%d covered_px=%d\n", FB_W * FB_H, covered);
    printf("scene_prims=%d opaque=%d blend=%d\n", sc.count, n_op, n_bl);
    printf("partition_err=%d\n", part_err);
    printf("submission_far_to_near=%d\n", order_back_to_front(&sc, op, n_op));
    printf("direct_opaque_tested=%ld shaded=%ld overdraw=%.3f\n",
           direct.opaque.tested, direct.opaque.shaded, direct_overdraw);
    printf("sorted_opaque_tested=%ld shaded=%ld overdraw=%.3f\n",
           sorted.opaque.tested, sorted.opaque.shaded,
           (double)sorted.opaque.shaded / (double)covered);
    printf("prepass_depth_tested=%ld depth_writes=%ld shaded=%ld\n",
           pre.prepass.tested, pre.prepass.depth_writes, pre.prepass.shaded);
    printf("prepass_opaque_tested=%ld shaded=%ld overdraw=%.3f\n",
           pre.opaque.tested, pre.opaque.shaded, pre_overdraw);
    printf("transparent_tested=%ld shaded=%ld blend_reads=%ld\n",
           pre.transparent.tested, pre.transparent.shaded, pre.transparent.blend_reads);
    printf("shaded_reduction=%ld (%.1f%%)\n",
           direct.opaque.shaded - pre.opaque.shaded, reduction_pct);
    printf("prepass_extra_frag_tests=%ld (%.1f%% of direct tests) depth_writes=%ld\n",
           extra_tests, extra_pct, extra_depth_writes);
    printf("cost_model shade_weight=%ld direct=%ld prepass=%ld ratio=%.3f\n",
           SHADE_WEIGHT, direct_cost, pre_cost,
           (double)pre_cost / (double)direct_cost);
    printf("color_diff_px=%d max_diff=%.9f depth_diff_px=%d\n",
           color_diff, (double)max_diff, depth_diff);

    check("partition_rc_ok", part_rc == 0);
    check("partition_count_ok", (part_err & PE_COUNT) == 0 && n_op + n_bl == sc.count);
    check("partition_unique_ok", (part_err & PE_DUP) == 0);
    check("partition_material_ok", (part_err & PE_MATERIAL) == 0);
    check("partition_alpha_ok", (part_err & PE_ALPHA) == 0);
    check("opaque_count_ok", n_op == OPAQUE_PRIMS);
    check("blend_count_ok", n_bl == BLEND_PRIMS);
    check("submission_not_front_to_back", !order_front_to_back(&sc, op, n_op));
    check("opaque_shading_front_to_back",
          (sort_by_z(&sc, op, n_op, 1), order_front_to_back(&sc, op, n_op)));
    check("blend_shading_back_to_front",
          (sort_by_z(&sc, bl, n_bl, 0), order_back_to_front(&sc, bl, n_bl)));

    check("prepass_is_depth_only",
          pre.prepass.shaded == 0 && pre.prepass.color_writes == 0 &&
          pre.prepass.depth_writes > 0);
    check("prepass_prim_count_ok", pre.prepass.tested == pre.opaque.tested &&
          direct.opaque.tested == pre.opaque.tested);
    check("prepass_opaque_pass_depth_static", pre.opaque.depth_writes == 0);
    check("direct_opaque_depth_test_active",
          direct.opaque.depth_fail > 0 &&
          direct.opaque.shaded == direct.opaque.tested - direct.opaque.depth_fail);
    check("prepass_opaque_shades_nearest_once",
          pre.opaque.depth_fail == pre.opaque.tested - pre.opaque.shaded &&
          pre.opaque.shaded == covered);
    check("transparent_no_depth_write",
          direct.transparent.depth_writes == 0 && pre.transparent.depth_writes == 0);
    check("transparent_blended",
          pre.transparent.blend_reads == pre.transparent.shaded &&
          pre.transparent.shaded > 0);
    check("opaque_not_blended",
          direct.opaque.blend_reads == 0 && pre.opaque.blend_reads == 0);
    check("transparent_pass_identical",
          stats_same(&direct.transparent, &pre.transparent));
    check("direct_vs_prepass_identical", color_diff == 0 && depth_diff == 0);
    check("sorted_vs_prepass_identical",
          sorted_color_diff == 0 && sorted_depth_diff == 0);
    check("overdraw_reduced", pre.opaque.shaded * 3 < direct.opaque.shaded);
    check("prepass_cost_reported", extra_tests > 0 && extra_depth_writes > 0);

    /* grouping must not only pass its own lists: a misclassified primitive has
       to be rejected, otherwise the check proves nothing */
    Scene tampered = sc;
    int bad_idx = -1;
    for (int i = 0; i < tampered.count; i++)
        if (tampered.prims[i].material == MAT_BLEND) { bad_idx = i; break; }
    tampered.prims[bad_idx].material = MAT_OPAQUE;   /* alpha still < 1 */
    int t_op[MAX_PRIMS], t_bl[MAX_PRIMS], t_nop, t_nbl;
    partition_scene(&tampered, t_op, &t_nop, t_bl, &t_nbl);
    check("tamper_blend_as_opaque_detected",
          (validate_partition(&tampered, t_op, t_nop, t_bl, t_nbl) & PE_ALPHA) != 0);

    Scene badmat = sc;
    badmat.prims[0].material = 7;
    check("tamper_bad_material_detected",
          partition_scene(&badmat, t_op, &t_nop, t_bl, &t_nbl) == -1);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
