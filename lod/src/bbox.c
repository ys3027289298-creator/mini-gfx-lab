
/* bbox-check: conservative bounding-box verification for the four LOD levels.
 *
 * Why this exists: whole grid tiles occasionally vanish at screen edges. The
 * suspected cause is a non-conservative cull box - a box a hair smaller than
 * the real geometry gets frustum-culled while the mesh is still on screen.
 * This check builds a tight AABB per LOD level and a conservative box grown by
 * a relative plus absolute epsilon ("rather too big than too small"), then
 * proves for all four levels:
 *   1. containment: every vertex of a level sits inside that level's
 *      conservative box (all four levels fully cover their vertex sets);
 *   2. projection:  no projected vertex leaks past the projected box rect, so
 *      edge-straddling meshes are never falsely culled (zero missed draws);
 *   3. cost:        the padded box's volume overhead stays under 20%.
 * A quantized "naive" box (the quantized-pipeline failure mode: build the box
 * from rounded positions) is tested the same way and is expected to FAIL,
 * reproducing the original symptom so the check has teeth.
 */
#include <stdio.h>
#include <math.h>
#include <stdint.h>

#define LEVELS 4
#define MAX_VERTS 1100
#define FOOTPRINT_HALF 5.0f
#define PAD_REL 0.01f        /* grow each face by 1% of the axis extent ... */
#define PAD_ABS 1e-3f        /* ... plus an absolute floor for thin axes    */
#define QUANT 0.05f          /* simulated position quantization step        */
#define REDUNDANCY_LIMIT 0.20f
#define PX_EPS 1e-3f         /* float-rounding slack for projected rects    */

typedef struct { float x, y, z; } Vec3;
typedef struct { Vec3 mn, mx; } Aabb;

static const int grid_n[LEVELS] = { 33, 17, 9, 5 };

static uint32_t rng_state;
static float frand(void) {
    rng_state = rng_state * 1664525u + 1013904223u;
    return (float)(rng_state >> 8) * (1.0f / 16777216.0f);
}

static float height_at(float x, float y) {
    return 0.5f * sinf(x * 0.9f) * cosf(y * 0.7f);
}

static Vec3 vadd(Vec3 a, Vec3 b) {
    Vec3 r = { a.x + b.x, a.y + b.y, a.z + b.z };
    return r;
}

/* LOD re-sampling: coarser levels keep the same footprint but get a different
   vertex set, so each level needs its own box rather than a shared one. */
static int build_level(int lv, Vec3* out) {
    int n = grid_n[lv];
    rng_state = 0x9e3779b9u ^ (uint32_t)(lv * 2654435761u);
    int k = 0;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            float x = -FOOTPRINT_HALF + 2.0f * FOOTPRINT_HALF * j / (n - 1);
            float y = -FOOTPRINT_HALF + 2.0f * FOOTPRINT_HALF * i / (n - 1);
            out[k].x = x + (frand() - 0.5f) * 0.05f;
            out[k].y = y + (frand() - 0.5f) * 0.05f;
            out[k].z = height_at(x, y) + (frand() - 0.5f) * 0.05f;
            k++;
        }
    }
    return k;
}

static Aabb tight_box(const Vec3* v, int n) {
    Aabb b = { {1e30f, 1e30f, 1e30f}, {-1e30f, -1e30f, -1e30f} };
    for (int i = 0; i < n; i++) {
        if (v[i].x < b.mn.x) b.mn.x = v[i].x;
        if (v[i].y < b.mn.y) b.mn.y = v[i].y;
        if (v[i].z < b.mn.z) b.mn.z = v[i].z;
        if (v[i].x > b.mx.x) b.mx.x = v[i].x;
        if (v[i].y > b.mx.y) b.mx.y = v[i].y;
        if (v[i].z > b.mx.z) b.mx.z = v[i].z;
    }
    return b;
}

static float quantize_inward_min(float v) { return QUANT * ceilf(v / QUANT); }
static float quantize_inward_max(float v) { return QUANT * floorf(v / QUANT); }

/* The non-conservative box under suspicion: the CPU cull box is rebuilt from
   quantized grid coordinates with each face rounded INWARD (min up, max down),
   while the GPU keeps the displaced, unquantized vertices. Every face shrinks
   by up to a full quantization step, so edge placements exist where the box is
   frustum-culled although displaced vertices are still on screen. */
static Aabb naive_box(const Vec3* v, int n) {
    Aabb t = tight_box(v, n);
    Aabb b;
    b.mn.x = quantize_inward_min(t.mn.x);
    b.mn.y = quantize_inward_min(t.mn.y);
    b.mn.z = quantize_inward_min(t.mn.z);
    b.mx.x = quantize_inward_max(t.mx.x);
    b.mx.y = quantize_inward_max(t.mx.y);
    b.mx.z = quantize_inward_max(t.mx.z);
    return b;
}

/* Conservative expansion: every face moves outward by rel*extent + abs epsilon,
   so rounding error can never push a vertex outside the box. */
static Aabb conservative_box(const Aabb* t) {
    Aabb c;
    float ex = PAD_ABS + PAD_REL * (t->mx.x - t->mn.x);
    float ey = PAD_ABS + PAD_REL * (t->mx.y - t->mn.y);
    float ez = PAD_ABS + PAD_REL * (t->mx.z - t->mn.z);
    c.mn.x = t->mn.x - ex; c.mx.x = t->mx.x + ex;
    c.mn.y = t->mn.y - ey; c.mx.y = t->mx.y + ey;
    c.mn.z = t->mn.z - ez; c.mx.z = t->mx.z + ez;
    return c;
}

static int contains(const Aabb* b, Vec3 p) {
    return p.x >= b->mn.x && p.x <= b->mx.x &&
           p.y >= b->mn.y && p.y <= b->mx.y &&
           p.z >= b->mn.z && p.z <= b->mx.z;
}

static float box_volume(const Aabb* b) {
    return (b->mx.x - b->mn.x) * (b->mx.y - b->mn.y) * (b->mx.z - b->mn.z);
}

static Aabb translated_box(const Aabb* b, Vec3 d) {
    Aabb r = { vadd(b->mn, d), vadd(b->mx, d) };
    return r;
}

typedef struct {
    Vec3 pos;
    float fov_y, near;
    int w, h;
} Camera;

/* pinhole projection, camera looks down -z; 0 = behind the near plane */
static int project(const Camera* c, Vec3 p, float* sx, float* sy) {
    float vz = p.z - c->pos.z;
    if (vz >= -c->near) return 0;
    float t = tanf(c->fov_y * 0.5f);
    float aspect = (float)c->w / (float)c->h;
    float ndx = (p.x - c->pos.x) / (-vz * t * aspect);
    float ndy = (p.y - c->pos.y) / (-vz * t);
    *sx = (ndx + 1.0f) * 0.5f * (float)c->w;
    *sy = (1.0f - ndy) * 0.5f * (float)c->h;
    return 1;
}

/* Conservative screen rect of an AABB: with every corner in front of the
   camera, the projected box lies in the projected corners' convex hull, so
   corner min/max bounds it. Returns 0 if the whole box is behind the plane. */
static int project_box(const Camera* cam, const Aabb* b, float rect[4]) {
    float minx = 1e30f, miny = 1e30f, maxx = -1e30f, maxy = -1e30f;
    int any = 0;
    for (int iz = 0; iz < 2; iz++)
      for (int iy = 0; iy < 2; iy++)
        for (int ix = 0; ix < 2; ix++) {
            Vec3 p = { ix ? b->mx.x : b->mn.x,
                       iy ? b->mx.y : b->mn.y,
                       iz ? b->mx.z : b->mn.z };
            float sx, sy;
            if (project(cam, p, &sx, &sy)) {
                any = 1;
                if (sx < minx) minx = sx;
                if (sy < miny) miny = sy;
                if (sx > maxx) maxx = sx;
                if (sy > maxy) maxy = sy;
            }
        }
    if (!any) return 0;
    rect[0] = minx; rect[1] = miny; rect[2] = maxx; rect[3] = maxy;
    return 1;
}

static int culled(const Camera* cam, const Aabb* b) {
    float rect[4];
    if (!project_box(cam, b, rect)) return 1;
    return rect[2] < 0.0f || rect[0] > (float)cam->w ||
           rect[3] < 0.0f || rect[1] > (float)cam->h;
}

static int truly_visible(const Camera* cam, const Vec3* v, int n, Vec3 d) {
    for (int i = 0; i < n; i++) {
        float sx, sy;
        if (project(cam, vadd(v[i], d), &sx, &sy) &&
            sx >= 0.0f && sx <= (float)cam->w &&
            sy >= 0.0f && sy <= (float)cam->h)
            return 1;
    }
    return 0;
}

/* Slide a mesh across one screen edge in fine steps (the failure window for a
   shrunken box is only a quantization-step wide) and compare each cull decision
   against per-vertex ground truth. */
static void sweep_axis(int axis, float lo, float hi,
                       const Camera* cam, const Vec3* v, int n,
                       const Aabb* naive, const Aabb* cons,
                       int* fc_naive, int* fc_cons, int* false_keep,
                       int* proj_leaks, int* placements) {
    for (float off = lo; off <= hi + 1e-6f; off += 0.01f) {
        Vec3 d = { 0.0f, 0.0f, 0.0f };
        if (axis == 0) d.x = off; else d.y = off;

        int truth = truly_visible(cam, v, n, d);
        Aabb bc = translated_box(cons, d);
        Aabb bn = translated_box(naive, d);
        int cull_c = culled(cam, &bc);
        int cull_n = culled(cam, &bn);
        if (truth && cull_c) (*fc_cons)++;     /* missed draw: forbidden */
        if (truth && cull_n) (*fc_naive)++;    /* bug reproduced */
        if (!truth && !cull_c) (*false_keep)++;

        float rect[4];
        if (project_box(cam, &bc, rect)) {
            for (int i = 0; i < n; i++) {
                float sx, sy;
                if (project(cam, vadd(v[i], d), &sx, &sy) &&
                    (sx < rect[0] - PX_EPS || sx > rect[2] + PX_EPS ||
                     sy < rect[1] - PX_EPS || sy > rect[3] + PX_EPS))
                    (*proj_leaks)++;
            }
        }
        (*placements)++;
    }
}

int main(void) {
    Camera cam = { {0.0f, 0.0f, 12.0f}, 1.0f, 0.1f, 1920, 1080 };

    Vec3 verts[LEVELS][MAX_VERTS];
    int counts[LEVELS];
    Aabb tight[LEVELS], cons[LEVELS], naive[LEVELS];

    for (int lv = 0; lv < LEVELS; lv++) {
        counts[lv] = build_level(lv, verts[lv]);
        tight[lv] = tight_box(verts[lv], counts[lv]);
        naive[lv] = naive_box(verts[lv], counts[lv]);
        cons[lv] = conservative_box(&tight[lv]);
    }

    printf("-- per-level containment / volume --\n");
    int leaks_naive = 0, leaks_cons = 0;
    float max_redundancy = 0.0f;
    for (int lv = 0; lv < LEVELS; lv++) {
        int ln = 0, lc = 0;
        for (int i = 0; i < counts[lv]; i++) {
            if (!contains(&naive[lv], verts[lv][i])) ln++;
            if (!contains(&cons[lv], verts[lv][i])) lc++;
        }
        float redundancy = box_volume(&cons[lv]) / box_volume(&tight[lv]) - 1.0f;
        if (redundancy > max_redundancy) max_redundancy = redundancy;
        printf("lv=%d verts=%d leaks_naive=%d leaks_cons=%d redundancy=%.4f\n",
               lv, counts[lv], ln, lc, redundancy);
        leaks_naive += ln;
        leaks_cons += lc;
    }

    printf("-- edge sweep: cull decision vs per-vertex ground truth --\n");
    float t = tanf(cam.fov_y * 0.5f);
    float aspect = (float)cam.w / (float)cam.h;
    float half_h = cam.pos.z * t;
    float half_w = half_h * aspect;
    int fc_naive = 0, fc_cons = 0, false_keep = 0, proj_leaks = 0, placements = 0;
    /* range must cover the full on-screen -> fully-off transition for a box
       with its own footprint, not just a zero-size point at the frustum edge */
    float span_y = half_h + FOOTPRINT_HALF + 1.0f;
    float span_x = half_w + FOOTPRINT_HALF + 1.0f;
    for (int lv = 0; lv < LEVELS; lv++) {
        sweep_axis(1, -span_y, span_y, &cam,
                   verts[lv], counts[lv], &naive[lv], &cons[lv],
                   &fc_naive, &fc_cons, &false_keep, &proj_leaks, &placements);
        sweep_axis(0, -span_x, span_x, &cam,
                   verts[lv], counts[lv], &naive[lv], &cons[lv],
                   &fc_naive, &fc_cons, &false_keep, &proj_leaks, &placements);
    }
    printf("placements=%d false_culls_naive=%d false_culls_cons=%d "
           "over_keeps=%d projected_leaks=%d\n",
           placements, fc_naive, fc_cons, false_keep, proj_leaks);

    int coverage_ok = (leaks_cons == 0);
    int no_missed_draws = (fc_cons == 0 && proj_leaks == 0);
    int redundancy_ok = (max_redundancy <= REDUNDANCY_LIMIT);
    /* the test only has teeth if the un-padded box actually demonstrates the bug */
    int naive_fails = (leaks_naive > 0 || fc_naive > 0);
    int ok = coverage_ok && no_missed_draws && redundancy_ok && naive_fails;

    printf("coverage_ok=%d\n", coverage_ok);
    printf("naive_box_fails=%d (leaks=%d false_culls=%d)\n",
           naive_fails, leaks_naive, fc_naive);
    printf("no_missed_draws=%d (over_keeps=%d, allowed: rather big than small)\n",
           no_missed_draws, false_keep);
    printf("max_redundancy=%.4f limit=%.2f redundancy_ok=%d\n",
           max_redundancy, REDUNDANCY_LIMIT, redundancy_ok);
    printf("bbox_conservative_ok=%d\n", ok);
    return ok ? 0 : 1;
}
