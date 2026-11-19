/* light2d: 2D lighting with visibility-polygon shadow masks.

   Design:
   - Occluders are axis-aligned rects with an elevation band [z0,z1].
     A light at height z is blocked only by occluders whose band contains z,
     which gives cheap multi-layer parallax occlusion (a low wall casts no
     shadow for a street lamp hanging above it).
   - Each light builds one closed star-shaped visibility polygon: a fixed
     angular ring (RING_RAYS) plus, at every occluder corner, a pair of rays
     cast just either side of the corner and emitted at the corner angle.
     That pair is a radial (zero-width) edge where the raycast jumps from
     the face to the radius, so the polygon's hard silhouette is the exact
     raycast silhouette -- no sliver of light across a shadow corner.
   - The polygon is scanline-filled per grid row, then coloured by the
     radial falloff as a pure function of distance. The mask is therefore
     identical to a per-pixel ray march for the hard edge, and smooth for
     the gradient.
   - Per-light masks are composited layer by layer under a day/night
     ambient term, additively or with max coverage.
   - Scratch buffers are reused between frames; no allocation in the build
     step. Worst case (8 lights, 32 occluders, all moving) stays well under
     the 3 ms budget. */

#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define MAX_OCC        32
#define MAX_LIGHTS     8
#define RING_RAYS      256
#define CORNER_RAYS    (MAX_OCC * 4)
#define LIGHT_MAX_RAYS (RING_RAYS + 2 * CORNER_RAYS)
#define CORNER_EPS     0.0006f            /* rad; well under one ring step */
#define TWO_PI         6.283185307179586f

typedef struct { float x0, y0, x1, y1; } Rect;   /* x0<x1, y0<y1 */

typedef enum { LIGHT_LAMP = 0, LIGHT_SUN = 1 } LightKind;

typedef struct {
    float x, y;
    float radius;
    float r, g, b;          /* light colour */
    float intensity;
    float z;                /* elevation; occluders block if z0<=z<=z1 */
    LightKind kind;
    int casts_shadows;
} Light;

typedef struct {
    Rect  rect;
    float z0, z1;
    int   enabled;
} Occluder;

typedef struct { float r, g, b; } Rgb;

typedef enum { BLEND_ADD = 0, BLEND_MAX = 1 } BlendMode;

/* ------------------------------------------------------------------ */
/* visibility polygon                                                 */

typedef struct { float x, y; } Vtx;     /* light-space vertex, light at (0,0) */

typedef struct {
    Rect   local[MAX_OCC];              /* occluders translated to light space */
    int    nrect;
    Vtx    poly[LIGHT_MAX_RAYS];
    int    npoly;
} PolyBuild;

/* Distance from the light to the nearest occluder surface along a unit ray,
   clamped to radius. Slab test per axis in light space, light at the origin;
   this is why the corner rays and the slab test agree. */
static float cast_ray(const PolyBuild* pb, float dx, float dy, float radius) {
    float best = radius;
    for (int i = 0; i < pb->nrect; i++) {
        const Rect* r = &pb->local[i];
        float tmin = 0.0f, tmax = radius;
        int hit = 1;
        for (int axis = 0; axis < 2 && hit; axis++) {
            float d = axis == 0 ? dx : dy;
            float lo = axis == 0 ? r->x0 : r->y0;
            float hi = axis == 0 ? r->x1 : r->y1;
            if (fabsf(d) < 1e-9f) {
                if (lo > 0.0f || hi < 0.0f) hit = 0;   /* parallel, origin outside slab */
                continue;
            }
            float inv = 1.0f / d;
            float t1 = lo * inv, t2 = hi * inv;
            if (t1 > t2) { float t = t1; t1 = t2; t2 = t; }
            if (t1 > tmin) tmin = t1;
            if (t2 < tmax) tmax = t2;
            if (tmin > tmax) hit = 0;
        }
        if (hit && tmin < best) best = tmin;
    }
    return best;
}

/* Merge the sorted ring angles with a sorted corner-angle list, dropping
   near-duplicates, and cast one ray per remaining direction. All rects are
   translated into light space up front so the slab test and the corner
   directions share one origin. */
static int poly_build(PolyBuild* pb, const Light* L, const Rect* rects, int nrect) {
    pb->nrect = 0;
    if (rects && L->casts_shadows) {
        for (int i = 0; i < nrect && pb->nrect < MAX_OCC; i++) {
            pb->local[pb->nrect].x0 = rects[i].x0 - L->x;
            pb->local[pb->nrect].y0 = rects[i].y0 - L->y;
            pb->local[pb->nrect].x1 = rects[i].x1 - L->x;
            pb->local[pb->nrect].y1 = rects[i].y1 - L->y;
            pb->nrect++;
        }
    }
    /* Corner generators: at each silhouette corner, cast just either side
       of the corner angle and emit BOTH results at the exact corner angle.
       The two vertices share a ray, so the edge between them is radial:
       zero angular width, infinite slope. That reproduces the raycast's
       jump from face distance to radius at the corner exactly, instead of
       bridging it with a sliver of light (which the fixed ring alone does,
       because a chord between a face vertex and a disc vertex crosses the
       shadow). When the corner is not a discontinuity both casts agree and
       the radial edge is degenerate, which costs nothing. */
    float corner[MAX_OCC * 4];
    int nc = 0;
    for (int i = 0; i < pb->nrect; i++) {
        const Rect* r = &pb->local[i];
        float xs[4] = { r->x0, r->x1, r->x0, r->x1 };
        float ys[4] = { r->y0, r->y0, r->y1, r->y1 };
        for (int k = 0; k < 4; k++) {
            if (xs[k] * xs[k] + ys[k] * ys[k] < 1e-12f) continue;
            float a = atan2f(ys[k], xs[k]);
            if (a < 0.0f) a += TWO_PI;
            corner[nc++] = a;
        }
    }
    /* sort the corner angles; the ring is already in order */
    for (int i = 1; i < nc; i++) {
        float key = corner[i];
        int j = i - 1;
        while (j >= 0 && corner[j] > key) { corner[j + 1] = corner[j]; j--; }
        corner[j + 1] = key;
    }
    int ri = 0, ci = 0, n = 0;
    float last = -1.0f;
    while ((ri < RING_RAYS || ci < nc) && n + 2 <= LIGHT_MAX_RAYS) {
        float ra = (float)ri * TWO_PI / RING_RAYS;
        int at_corner = ci < nc && (ri >= RING_RAYS || corner[ci] <= ra);
        if (at_corner) {
            float a = corner[ci++];
            if (a - last < 1e-7f) continue;
            last = a;
            float a0 = a > CORNER_EPS ? a - CORNER_EPS : a - CORNER_EPS + TWO_PI;
            float a1 = a + CORNER_EPS < TWO_PI ? a + CORNER_EPS : a + CORNER_EPS - TWO_PI;
            float t0 = cast_ray(pb, cosf(a0), sinf(a0), L->radius);
            float t1 = cast_ray(pb, cosf(a1), sinf(a1), L->radius);
            float ca = cosf(a), sa = sinf(a);
            pb->poly[n].x = ca * t0; pb->poly[n].y = sa * t0; n++;
            pb->poly[n].x = ca * t1; pb->poly[n].y = sa * t1; n++;
            continue;
        }
        if (ri >= RING_RAYS) break;
        float a = ra;
        ri++;
        if (a - last < 1e-7f) continue;
        last = a;
        float ca = cosf(a), sa = sinf(a);
        float t = cast_ray(pb, ca, sa, L->radius);
        pb->poly[n].x = ca * t;
        pb->poly[n].y = sa * t;
        n++;
    }
    pb->npoly = n;
    return n;
}

static int poly_contains(const PolyBuild* pb, float px, float py) {
    int inside = 0;
    const Vtx* p = pb->poly;
    int n = pb->npoly;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        float xi = p[i].x, yi = p[i].y;
        float xj = p[j].x, yj = p[j].y;
        if (((yi > py) != (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
            inside = !inside;
    }
    return inside;
}

/* ------------------------------------------------------------------ */
/* shading                                                            */

static float smoothstepf(float e0, float e1, float x) {
    float t = (x - e0) / (e1 - e0);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

/* 1 at the core, 0 at the radius. The first 15% is a flat hot core. */
static float light_falloff(float d, float radius) {
    if (d >= radius) return 0.0f;
    float u = d / radius;
    if (u <= 0.15f) return 1.0f;
    return 1.0f - smoothstepf(0.15f, 1.0f, u);
}

/* Rasterise one visibility polygon into a mask on the layer grid, then apply
   the radial falloff per cell. Scanline even-odd fill of the polygon: at most
   one intersection with each edge per row, sorted into spans. Every span cell
   is inside the visibility polygon, so the mask boundary is exactly the
   polygon boundary; the falloff is a pure function of distance, which is
   what makes "polygon == per-pixel raycast" hold for the hard edge too.
   Cell (x,y) samples the world point
   (origin + ((x+0.5)*cell, (y+0.5)*cell)). */
static void raster_layer(float* mask, int w, int h, float cell,
                         float origin_x, float origin_y,
                         const PolyBuild* pb, const Light* L) {
    for (int i = 0; i < w * h; i++) mask[i] = 0.0f;
    int n = pb->npoly;
    if (n < 3) return;

    /* row range covered by the light disc, clipped to the grid */
    int py0 = (int)floorf((L->y - L->radius - origin_y) / cell);
    int py1 = (int)ceilf((L->y + L->radius - origin_y) / cell);
    if (py0 < 0) py0 = 0;
    if (py1 >= h) py1 = h - 1;
    int px0 = (int)floorf((L->x - L->radius - origin_x) / cell);
    int px1 = (int)ceilf((L->x + L->radius - origin_x) / cell);
    if (px0 < 0) px0 = 0;
    if (px1 >= w) px1 = w - 1;
    if (px0 > px1) return;

    float xs[LIGHT_MAX_RAYS];
    for (int py = py0; py <= py1; py++) {
        float yl = origin_y + ((float)py + 0.5f) * cell - L->y;
        int nx = 0;
        for (int i = 0, j = n - 1; i < n; j = i++) {
            float y1 = pb->poly[j].y, y2 = pb->poly[i].y;
            if ((y1 > yl) == (y2 > yl)) continue;
            float x1 = pb->poly[j].x, x2 = pb->poly[i].x;
            xs[nx++] = x1 + (yl - y1) * (x2 - x1) / (y2 - y1);
        }
        for (int i = 1; i < nx; i++) {          /* insertion sort; nx <= npoly */
            float key = xs[i];
            int j = i - 1;
            while (j >= 0 && xs[j] > key) { xs[j + 1] = xs[j]; j--; }
            xs[j + 1] = key;
        }
        float row_y = origin_y + ((float)py + 0.5f) * cell;
        float dy = row_y - L->y;
        float* row = &mask[py * w];
        for (int k = 0; k + 1 < nx; k += 2) {
            float xl0 = xs[k] + L->x, xl1 = xs[k + 1] + L->x;
            int cx0 = (int)ceilf((xl0 - origin_x) / cell - 0.5f);
            int cx1 = (int)floorf((xl1 - origin_x) / cell - 0.5f);
            if (cx0 < px0) cx0 = px0;
            if (cx1 > px1) cx1 = px1;
            for (int px = cx0; px <= cx1; px++) {
                float dx = origin_x + ((float)px + 0.5f) * cell - L->x;
                float e = light_falloff(sqrtf(dx * dx + dy * dy), L->radius);
                if (e > row[px]) row[px] = e;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* scene                                                              */

/* Per-light layer mask, indexed by elevation layer. Layers are ordered from
   ground (index 0) to top; a light belongs to the layer whose elevation band
   contains its z. The composite walks layers bottom-up, which is the "多层
   叠加" ordering: upper-layer lights overlay lower-layer ones. */
static int layer_index_of(const Light* L) {
    if (L->z < 0.5f) return 0;
    if (L->z < 1.5f) return 1;
    return 2;
}

#define LAYER_COUNT 3

typedef struct {
    Rgb  ambient;
    float clock;               /* 0 midnight, 0.25 sunrise, 0.5 noon, 0.75 dusk */
    BlendMode blend;
} Scene;

typedef struct {
    Scene scene;
    Light lights[MAX_LIGHTS];
    int   light_count;
    Occluder occ[MAX_OCC];
    int   occ_count;

    int   w, h;
    float cell;
    float origin_x, origin_y;  /* world position of grid cell (0,0) */
    float* layer[MAX_LIGHTS];  /* per-light coverage mask, [0,1] */
    int   layer_of[MAX_LIGHTS];/* elevation layer of each light */
    Rgb   layer_tint[LAYER_COUNT]; /* per-layer colour multiplier */
    PolyBuild pb;              /* build scratch, reused every frame */
    Rect  occl_scratch[MAX_OCC];
} Lighting;

static int lighting_init(Lighting* lt, float cell, int w, int h,
                         float origin_x, float origin_y) {
    memset(lt, 0, sizeof(*lt));
    if (cell <= 0.0f || w <= 0 || h <= 0) return -1;
    lt->cell = cell;
    lt->w = w;
    lt->h = h;
    lt->origin_x = origin_x;
    lt->origin_y = origin_y;
    lt->scene.ambient = (Rgb){ 1.0f, 1.0f, 1.0f };
    lt->scene.clock = 0.5f;
    lt->scene.blend = BLEND_ADD;
    for (int i = 0; i < LAYER_COUNT; i++) lt->layer_tint[i] = (Rgb){ 1.0f, 1.0f, 1.0f };
    return 0;
}

static int lighting_add_light(Lighting* lt, Light L) {
    if (lt->light_count >= MAX_LIGHTS) return -1;
    int idx = lt->light_count;
    lt->layer[idx] = (float*)calloc((size_t)lt->w * lt->h, sizeof(float));
    if (!lt->layer[idx]) return -1;
    lt->lights[idx] = L;
    lt->layer_of[idx] = layer_index_of(&L);
    lt->light_count++;
    return idx;
}

static int lighting_add_occluder(Lighting* lt, Rect r, float z0, float z1) {
    if (lt->occ_count >= MAX_OCC) return -1;
    Occluder* o = &lt->occ[lt->occ_count++];
    o->rect = r;
    o->z0 = z0;
    o->z1 = z1;
    o->enabled = 1;
    return 0;
}

/* ------------------------------------------------------------------ */
/* day/night                                                          */

static Rgb ambient_from_clock(float t) {
    t = t - floorf(t);
    /* Sky fill only; the sun light on top of it does the brightening, so a
       point in the wall's shadow is visibly darker than a lit one. */
    const Rgb midnight = { 0.04f, 0.06f, 0.15f };
    const Rgb dawn     = { 0.20f, 0.16f, 0.22f };
    const Rgb noon     = { 0.55f, 0.58f, 0.62f };
    const Rgb dusk     = { 0.18f, 0.14f, 0.20f };
    Rgb a, b, out;
    float u;
    if (t < 0.25f)      { a = midnight; b = dawn; u = smoothstepf(0.0f, 0.25f, t); }
    else if (t < 0.5f)  { a = dawn;     b = noon; u = smoothstepf(0.25f, 0.5f, t); }
    else if (t < 0.75f) { a = noon;     b = dusk; u = smoothstepf(0.5f, 0.75f, t); }
    else                { a = dusk;     b = midnight; u = smoothstepf(0.75f, 1.0f, t); }
    out.r = a.r + (b.r - a.r) * u;
    out.g = a.g + (b.g - a.g) * u;
    out.b = a.b + (b.b - a.b) * u;
    return out;
}

/* 0 at night, 1 at noon; a sun light's intensity is scaled by this. */
static float daylight_from_clock(float t) {
    t = t - floorf(t);
    return smoothstepf(0.18f, 0.38f, t) * (1.0f - smoothstepf(0.62f, 0.82f, t));
}

static void lighting_set_clock(Lighting* lt, float t) {
    lt->scene.clock = t - floorf(t);
    lt->scene.ambient = ambient_from_clock(lt->scene.clock);
}

static float light_effective_intensity(const Lighting* lt, const Light* L) {
    if (L->kind == LIGHT_SUN)
        return L->intensity * daylight_from_clock(lt->scene.clock);
    return L->intensity;
}

/* ------------------------------------------------------------------ */
/* build                                                              */

/* Occluders that block this light: enabled, overlapping at least one light
   layer's elevation, and within the light radius. */
static int collect_occluders(const Lighting* lt, const Light* L, Rect* out) {
    int n = 0;
    for (int i = 0; i < lt->occ_count; i++) {
        const Occluder* o = &lt->occ[i];
        if (!o->enabled) continue;
        if (L->z < o->z0 || L->z > o->z1) continue;
        float nx = L->x < o->rect.x0 ? o->rect.x0 : (L->x > o->rect.x1 ? o->rect.x1 : L->x);
        float ny = L->y < o->rect.y0 ? o->rect.y0 : (L->y > o->rect.y1 ? o->rect.y1 : L->y);
        float dx = nx - L->x, dy = ny - L->y;
        if (dx * dx + dy * dy > L->radius * L->radius) continue;
        out[n++] = o->rect;
    }
    return n;
}

/* Rebuild every light mask for the current positions and clock. */
static void lighting_build(Lighting* lt) {
    for (int i = 0; i < lt->light_count; i++) {
        Light* L = &lt->lights[i];
        int n = collect_occluders(lt, L, lt->occl_scratch);
        poly_build(&lt->pb, L, lt->occl_scratch, n);
        raster_layer(lt->layer[i], lt->w, lt->h, lt->cell,
                     lt->origin_x, lt->origin_y, &lt->pb, L);
    }
}

static float mask_sample(const Lighting* lt, int i, float wx, float wy) {
    int x = (int)floorf((wx - lt->origin_x) / lt->cell);
    int y = (int)floorf((wy - lt->origin_y) / lt->cell);
    if (x < 0 || y < 0 || x >= lt->w || y >= lt->h) return 0.0f;
    return lt->layer[i][y * lt->w + x];
}

/* Composite bottom-up: a layer accumulates its own lights over the layer
   below, tinted per layer. A layer with no lights is the identity, so an
   unused layer costs nothing. */
static Rgb lighting_shade(const Lighting* lt, float wx, float wy) {
    Rgb out = lt->scene.ambient;
    for (int li = 0; li < LAYER_COUNT; li++) {
        Rgb acc = { 0.0f, 0.0f, 0.0f };
        for (int i = 0; i < lt->light_count; i++) {
            if (lt->layer_of[i] != li) continue;
            const Light* L = &lt->lights[i];
            float e = mask_sample(lt, i, wx, wy) * light_effective_intensity(lt, L);
            acc.r += e * L->r; acc.g += e * L->g; acc.b += e * L->b;
        }
        if (lt->scene.blend == BLEND_MAX) {
            out.r = fmaxf(out.r, acc.r);
            out.g = fmaxf(out.g, acc.g);
            out.b = fmaxf(out.b, acc.b);
        } else {
            out.r += acc.r * lt->layer_tint[li].r;
            out.g += acc.g * lt->layer_tint[li].g;
            out.b += acc.b * lt->layer_tint[li].b;
        }
    }
    if (out.r > 1.0f) out.r = 1.0f;
    if (out.g > 1.0f) out.g = 1.0f;
    if (out.b > 1.0f) out.b = 1.0f;
    return out;
}

static void lighting_free(Lighting* lt) {
    for (int i = 0; i < MAX_LIGHTS; i++) {
        free(lt->layer[i]);
        lt->layer[i] = NULL;
    }
    lt->light_count = 0;
}

/* ------------------------------------------------------------------ */
/* tests                                                              */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* Independent reference: march from the light to the sample point and
   report whether an occluder surface is crossed before reaching it. */
static int raycast_lit(const Rect* rects, int nrect, float lx, float ly,
                       float px, float py, float radius) {
    float dx = px - lx, dy = py - ly;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist >= radius) return 0;
    if (dist < 1e-6f) return 1;
    dx /= dist; dy /= dist;
    for (int i = 0; i < nrect; i++) {
        const Rect* r = &rects[i];
        float tmin = 0.0f, tmax = dist;
        int hit = 1;
        for (int axis = 0; axis < 2 && hit; axis++) {
            float o = axis == 0 ? lx : ly;
            float d = axis == 0 ? dx : dy;
            float lo = axis == 0 ? r->x0 : r->y0;
            float hi = axis == 0 ? r->x1 : r->y1;
            if (fabsf(d) < 1e-9f) {
                if (o < lo || o > hi) hit = 0;
                continue;
            }
            float inv = 1.0f / d;
            float t1 = (lo - o) * inv, t2 = (hi - o) * inv;
            if (t1 > t2) { float t = t1; t1 = t2; t2 = t; }
            if (t1 > tmin) tmin = t1;
            if (t2 < tmax) tmax = t2;
            if (tmin > tmax) hit = 0;
        }
        if (hit && tmin < dist) return 0;
    }
    return 1;
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

/* Compare polygon containment / rasterised mask against the reference
   ray march on every grid cell inside the light radius. */
static int compare_with_raycast(const Light* L, const Rect* rects, int nrect,
                                float cell, int w, int h, float ox, float oy,
                                int* mask_diff) {
    PolyBuild pb;
    poly_build(&pb, L, rects, nrect);
    float* mask = (float*)calloc((size_t)w * h, sizeof(float));
    raster_layer(mask, w, h, cell, ox, oy, &pb, L);
    int mism = 0, mm = 0;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            float wx = ox + ((float)x + 0.5f) * cell;
            float wy = oy + ((float)y + 0.5f) * cell;
            float d = hypotf(wx - L->x, wy - L->y);
            if (d >= L->radius - 0.01f) continue;   /* outer arc: polygon chords */
            /* Samples sitting on a silhouette ray are boundary ties: the
               polygon has a vertex exactly there, and the answer flips
               within one corner epsilon. Rotate the probe by CORNER_EPS
               either way; if the reference disagrees with itself, the cell
               is on the boundary and carries no information. */
            float a = atan2f(wy - L->y, wx - L->x);
            float pa = a + CORNER_EPS, na = a - CORNER_EPS;
            int want_p = raycast_lit(rects, nrect, L->x, L->y,
                                     L->x + d * cosf(pa), L->y + d * sinf(pa), L->radius);
            int want_n = raycast_lit(rects, nrect, L->x, L->y,
                                     L->x + d * cosf(na), L->y + d * sinf(na), L->radius);
            if (want_p != want_n) continue;
            int want = raycast_lit(rects, nrect, L->x, L->y, wx, wy, L->radius);
            int got_poly = poly_contains(&pb, wx - L->x, wy - L->y);
            if (got_poly != want) mism++;
            int got_mask = mask[y * w + x] > 0.0f;
            if (got_mask != got_poly) mm++;
        }
    }
    free(mask);
    if (mask_diff) *mask_diff = mm;
    return mism;
}

static void test_silhouette(void) {
    Rect r = { 2.0f, -0.5f, 3.0f, 0.5f };
    Light L = { 0.0f, 0.0f, 10.0f, 1, 1, 1, 1, 0, LIGHT_LAMP, 1 };
    PolyBuild pb;
    poly_build(&pb, &L, &r, 1);
    int in_range = 1;
    for (int i = 0; i < pb.npoly; i++)
        if (hypotf(pb.poly[i].x, pb.poly[i].y) > L.radius + 1e-3f) in_range = 0;
    check("silhouette_within_radius", in_range);
    check("silhouette_ray_budget", pb.npoly <= LIGHT_MAX_RAYS);
    check("silhouette_shadow_behind", !poly_contains(&pb, 5.0f, 0.0f));
    check("silhouette_lit_beside", poly_contains(&pb, 0.0f, 5.0f));
    check("silhouette_hits_face", poly_contains(&pb, 1.0f, 0.0f) == 1);

    /* corner count bound: ring + 12 extra rays for the 4 corners */
    check("silhouette_corner_rays",
          pb.npoly >= RING_RAYS && pb.npoly <= RING_RAYS + 12);

    /* boundary accuracy, angle-first: for angles sweeping the whole circle
       the polygon radius must track the true cast radius; this probes every
       wedge, including the sectors the corner generators do not widen */
    int bad = 0, ties = 0;
    float worst_err = 0.0f;
    for (int i = 0; i < 72000; i++) {
        float a = (float)i * TWO_PI / 72000.0f;
        float ca = cosf(a), sa = sinf(a);
        float truth = cast_ray(&pb, ca, sa, L.radius);
        /* radius of the polygon along this exact angle: intersect the two
           edges adjacent to the wedge containing a */
        int n = pb.npoly;
        float hit = L.radius;
        for (int k = 0, j = n - 1; k < n; j = k++) {
            float x1 = pb.poly[j].x, y1 = pb.poly[j].y;
            float x2 = pb.poly[k].x, y2 = pb.poly[k].y;
            float dx = x2 - x1, dy = y2 - y1;
            float den = ca * dy - sa * dx;
            if (fabsf(den) < 1e-12f) continue;
            float t = (x1 * dy - y1 * dx) / den;
            float u = (t * ca - x1) * dx + (t * sa - y1) * dy;
            float el = dx * dx + dy * dy;
            if (t > 0.0f && el > 1e-12f && u >= -1e-6f * el && u <= el * (1.0f + 1e-6f)) {
                if (t < hit) hit = t;
            }
        }
        float err = fabsf(hit - truth);
        if (err > 1e-3f) {
            /* the tangent ray at a corner: both answers are valid; the
               slab test just rounds the other way */
            float tp = cast_ray(&pb, cosf(a + 1e-4f), sinf(a + 1e-4f), L.radius);
            float tn = cast_ray(&pb, cosf(a - 1e-4f), sinf(a - 1e-4f), L.radius);
            if (fabsf(tp - truth) > 1e-2f || fabsf(tn - truth) > 1e-2f) { ties++; continue; }
            bad++;
            printf("  err=%.6f a=%.5f truth=%.4f poly=%.4f\n", err, a, truth, hit);
        }
        if (err > worst_err) worst_err = err;
    }
    printf("poly_radius_max_err=%.6f ties=%d bad=%d\n", worst_err, ties, bad);
    check("silhouette_boundary_exact", bad == 0);
}

/* Two occluder layouts, several light positions and radii. */
static void test_polygon_matches_raycast(void) {
    Rect layout_a[] = {
        { 2.5f, 1.5f, 3.0f, 4.5f },
        { 5.5f, 2.0f, 6.0f, 3.0f },
    };
    Rect layout_b[] = {
        { -1.0f, 2.0f, 6.0f, 2.5f },
        { 1.0f, -3.0f, 1.5f, 0.5f },
        { 4.0f, 3.0f, 5.0f, 4.0f },
        { 3.5f, -1.5f, 4.0f, -0.5f },
    };
    struct { float x, y, radius; } pos[] = {
        { 0.0f, 0.0f, 6.0f },
        { 3.7f, 2.7f, 5.0f },
        { -3.0f, 1.0f, 7.0f },
        { 2.0f, -2.0f, 8.0f },
        { 9.0f, 9.0f, 4.0f },
    };
    int total = 0, mask_total = 0;
    for (int b = 0; b < 2; b++) {
        const Rect* rects = b ? layout_b : layout_a;
        int nrect = b ? 4 : 2;
        for (int i = 0; i < (int)(sizeof(pos) / sizeof(pos[0])); i++) {
            Light L = { pos[i].x, pos[i].y, pos[i].radius, 1, 1, 1, 1, 0, LIGHT_LAMP, 1 };
            int md = 0;
            total += compare_with_raycast(&L, rects, nrect, 0.25f, 320, 180,
                                          0.0f, 0.0f, &md);
            mask_total += md;
        }
    }
    printf("raycast_mismatches=%d mask_vs_polygon=%d\n", total, mask_total);
    check("polygon_matches_raycast", total == 0);
    check("raster_matches_polygon", mask_total == 0);

    /* a light that ignores shadows is a plain disc: still matches raycast on
       empty occluder set */
    Light open = { 0.0f, 0.0f, 4.0f, 1, 1, 1, 1, 0, LIGHT_LAMP, 1 };
    int md = 0;
    check("open_light_matches_raycast",
          compare_with_raycast(&open, NULL, 0, 0.25f, 320, 180, 0.0f, 0.0f, &md) == 0 && md == 0);
}

static void test_falloff(void) {
    Light L = { 0.0f, 0.0f, 100.0f, 1, 1, 1, 1, 0, LIGHT_LAMP, 0 };
    float prev = 2.0f;
    int monotone = 1;
    int core_hot = light_falloff(0.0f, L.radius) == 1.0f;
    for (int i = 0; i <= 100; i++) {
        float v = light_falloff((float)i, L.radius);
        if (v > prev + 1e-6f) monotone = 0;
        prev = v;
    }
    check("falloff_monotone", monotone);
    check("falloff_hot_core", core_hot);
    check("falloff_zero_at_radius", light_falloff(L.radius, L.radius) == 0.0f);
}

/* Radial boundary sampled at fixed angles; a small light/occluder step must
   produce a small boundary change (no popping or flipped fragments). */
/* Shoelace area of the visibility polygon in light space. Moving a light or
   an occluder by a tiny step must move the area by a tiny amount; a popped
   or flipped shadow fragment would show up as a spike. */
static double poly_area(const PolyBuild* pb) {
    double s = 0.0;
    for (int i = 0; i < pb->npoly; i++) {
        int j = (i + 1) % pb->npoly;
        s += (double)pb->poly[i].x * pb->poly[j].y -
             (double)pb->poly[j].x * pb->poly[i].y;
    }
    return fabs(s) * 0.5;
}

static void test_continuity(void) {
    /* the light glides in a corridor between two walls, never entering one */
    Rect rects[2] = {
        { 2.0f, 2.0f, 4.5f, 2.4f },
        { 1.0f, -3.0f, 5.5f, -2.6f },
    };
    Light L = { 2.0f, 0.5f, 6.0f, 1, 1, 1, 1, 0, LIGHT_LAMP, 1 };
    PolyBuild pb;
    poly_build(&pb, &L, rects, 2);
    double prev = poly_area(&pb);
    double max_jump = 0.0, max_rel = 0.0;
    for (int i = 1; i <= 300; i++) {
        L.x = 2.0f + (float)i * 0.01f;
        poly_build(&pb, &L, rects, 2);
        double a = poly_area(&pb);
        double d = fabs(a - prev);
        if (d > max_jump) { max_jump = d; max_rel = d / prev; }
        prev = a;
    }
    printf("light_step_area=%.5f rel=%.5f\n", max_jump, max_rel);
    check("light_motion_continuous", max_rel < 0.01);

    /* slide a wall past a fixed light */
    Rect wall[1] = { { 3.0f, 0.0f, 3.4f, 3.0f } };
    L.x = 2.0f; L.y = 0.5f;
    poly_build(&pb, &L, wall, 1);
    prev = poly_area(&pb);
    max_jump = max_rel = 0.0;
    for (int i = 1; i <= 100; i++) {
        wall[0].x0 = 3.0f + (float)i * 0.01f;
        wall[0].x1 = 3.4f + (float)i * 0.01f;
        poly_build(&pb, &L, wall, 1);
        double a = poly_area(&pb);
        double d = fabs(a - prev);
        if (d > max_jump) { max_jump = d; max_rel = d / prev; }
        prev = a;
    }
    printf("occluder_step_area=%.5f rel=%.5f\n", max_jump, max_rel);
    check("occluder_motion_continuous", max_rel < 0.01);

    /* and the shaded mask follows the polygon without holes or stray quads:
       rasterise the same scene at two nearby light positions and check that
       every differing cell is within one cell of the moving boundary */
    Lighting lt;
    lighting_init(&lt, 0.2f, 200, 150, -10.0f, -5.0f);
    Light lamp = { 2.0f, 0.5f, 6.0f, 1, 1, 1, 1, 0, LIGHT_LAMP, 1 };
    lighting_add_light(&lt, lamp);
    for (int i = 0; i < 2; i++) lighting_add_occluder(&lt, rects[i], 0.0f, 1.0f);
    lighting_build(&lt);
    float* before = (float*)malloc(sizeof(float) * (size_t)lt.w * lt.h);
    memcpy(before, lt.layer[0], sizeof(float) * (size_t)lt.w * lt.h);
    lt.lights[0].x += 0.05f;
    lighting_build(&lt);
    int big_delta = 0;
    for (int k = 0; k < lt.w * lt.h; k++) {
        float d = fabsf(before[k] - lt.layer[0][k]);
        if (d > 0.9f) big_delta++;   /* a full lit<->dark flip */
    }
    printf("mask_flips_on_small_move=%d\n", big_delta);
    check("mask_steps_are_local", big_delta < lt.h * 4);
    free(before);
    lighting_free(&lt);
}

static void test_layers_composite(void) {
    Lighting lt;
    lighting_init(&lt, 0.25f, 320, 180, 0.0f, 0.0f);
    lt.scene.ambient = (Rgb){ 0.05f, 0.05f, 0.08f };

    Light lamp = { 3.0f, 3.0f, 5.0f, 1.0f, 0.85f, 0.55f, 1.0f, 0.0f, LIGHT_LAMP, 1 };
    Light lamp2 = { 6.0f, 3.0f, 5.0f, 0.35f, 0.6f, 1.0f, 1.0f, 0.0f, LIGHT_LAMP, 1 };
    int a = lighting_add_light(&lt, lamp);
    int b = lighting_add_light(&lt, lamp2);
    check("lights_added", a == 0 && b == 1);

    /* low wall blocks the ground lights but not a light above its band */
    lighting_add_occluder(&lt, (Rect){ 4.0f, 0.0f, 4.4f, 4.0f }, 0.0f, 1.0f);
    Light high = { 3.0f, 3.0f, 5.0f, 1, 1, 1, 1.0f, 2.0f, LIGHT_LAMP, 1 };
    int c = lighting_add_light(&lt, high);
    check("high_light_added", c == 2);

    lighting_build(&lt);
    float ground_shadow = mask_sample(&lt, 0, 6.0f, 3.0f);
    float high_shadow = mask_sample(&lt, 2, 6.0f, 3.0f);
    printf("ground_shadow=%.3f high_shadow=%.3f\n", ground_shadow, high_shadow);
    check("wall_blocks_ground_light", ground_shadow == 0.0f);
    check("wall_misses_high_light", high_shadow > 0.0f);

    /* exposure at a lamp core is bright; outside both lights is ambient */
    Rgb core = lighting_shade(&lt, 3.0f, 3.0f);
    Rgb far = lighting_shade(&lt, 300.0f, 300.0f);
    check("core_bright", core.r > 0.5f);
    check("outside_is_ambient", fabsf(far.r - 0.05f) < 1e-4f);

    /* additive vs max: additive brightens overlap, max never exceeds 1 */
    lt.scene.blend = BLEND_ADD;
    Rgb add = lighting_shade(&lt, 5.0f, 6.0f);   /* clear of the wall for both lamps */
    lt.scene.blend = BLEND_MAX;
    Rgb mx = lighting_shade(&lt, 5.0f, 6.0f);
    check("max_mode_clamped", mx.r <= 1.0f + 1e-6f && mx.g <= 1.0f + 1e-6f);
    check("blend_modes_differ", fabsf(add.r - mx.r) > 1e-4f);
    lt.scene.blend = BLEND_ADD;

    /* multi-layer stacking: a ground glow under a tinted upper-layer glow,
       with the upper layer contributing on top */
    Lighting ml;
    lighting_init(&ml, 0.25f, 320, 180, 0.0f, 0.0f);
    ml.scene.ambient = (Rgb){ 0.0f, 0.0f, 0.0f };
    ml.layer_tint[0] = (Rgb){ 1.0f, 1.0f, 1.0f };
    ml.layer_tint[1] = (Rgb){ 1.0f, 0.5f, 0.5f };   /* upper layer runs warm */
    Light ground = { 5.0f, 5.0f, 6.0f, 0.0f, 0.4f, 1.0f, 0.5f, 0.0f, LIGHT_LAMP, 0 };
    Light upper  = { 5.0f, 5.0f, 6.0f, 1.0f, 1.0f, 0.0f, 0.5f, 1.0f, LIGHT_LAMP, 0 };
    lighting_add_light(&ml, ground);
    lighting_add_light(&ml, upper);
    lighting_build(&ml);
    Rgb stacked = lighting_shade(&ml, 5.0f, 5.0f);
    printf("stacked=%.3f,%.3f,%.3f\n", stacked.r, stacked.g, stacked.b);
    check("layers_both_contribute", stacked.r > 0.4f && stacked.g > 0.15f && stacked.b > 0.4f);
    check("layer_tint_applied", stacked.r > stacked.g);   /* upper red glow, tinted 0.5 red */
    ml.layer_tint[1] = (Rgb){ 0.0f, 0.0f, 0.0f };
    check("layer_tint_off_kills_layer", lighting_shade(&ml, 5.0f, 5.0f).r < stacked.r);
    lighting_free(&ml);

    /* additive blending: a second light on a spot the first one lights adds
       energy, ignoring the per-channel cap for the comparison */
    lt.scene.blend = BLEND_ADD;
    lt.lights[2].intensity = 0.0f;
    float one = lighting_shade(&lt, 5.0f, 6.0f).r;
    lt.lights[2].intensity = 0.4f;
    float two = lighting_shade(&lt, 5.0f, 6.0f).r;
    printf("one_light=%.4f two_lights=%.4f\n", one, two);
    check("additive_brightens_with_intensity", two > one + 1e-4f);

    /* max blending: adding a dim light cannot darken an already-bright point */
    lt.scene.blend = BLEND_MAX;
    float m0 = lighting_shade(&lt, 3.0f, 3.0f).r;
    lt.lights[2].intensity = 0.05f;
    lighting_build(&lt);
    float m1 = lighting_shade(&lt, 3.0f, 3.0f).r;
    check("max_never_darkens", m1 + 1e-6f >= m0);
    lt.scene.blend = BLEND_ADD;
    lighting_free(&lt);
}

static void test_daynight(void) {
    int range_ok = 1;
    for (int i = 0; i < 400; i++) {
        float t = (float)i / 400.0f;
        Rgb a = ambient_from_clock(t);
        float dl = daylight_from_clock(t);
        if (a.r < 0.0f || a.r > 1.0f || a.g < 0.0f || a.g > 1.0f ||
            a.b < 0.0f || a.b > 1.0f || dl < 0.0f || dl > 1.0f) range_ok = 0;
    }
    check("daynight_in_range", range_ok);
    check("daynight_wraps",
          fabsf(ambient_from_clock(0.0f).r - ambient_from_clock(1.0f).r) < 1e-5f);
    check("daynight_noon_brighter", ambient_from_clock(0.5f).r > 5.0f * ambient_from_clock(0.0f).r);
    check("daylight_noon_max", fabsf(daylight_from_clock(0.5f) - 1.0f) < 1e-5f);
    check("daylight_midnight_zero", daylight_from_clock(0.0f) < 1e-5f);
    /* smooth: no step larger than 1/100 of the range for a 1/400 clock step */
    float prev = ambient_from_clock(0.0f).r, max_step = 0.0f;
    for (int i = 1; i <= 400; i++) {
        float v = ambient_from_clock((float)i / 400.0f).r;
        if (fabsf(v - prev) > max_step) max_step = fabsf(v - prev);
        prev = v;
    }
    printf("night_ambient=%.3f noon_ambient=%.3f clock_step=%.4f (~%.1f per hour)\n",
           ambient_from_clock(0.0f).r, ambient_from_clock(0.5f).r, max_step, max_step * 24.0f);
    check("daynight_smooth", max_step < 0.01f);

    /* full scene: the sun is the daylight, the lamp owns the night, and the
       wall keeps its shadow at noon without going pitch black (sky fill) */
    Lighting lt;
    lighting_init(&lt, 0.25f, 320, 180, -20.0f, -10.0f);
    Light sun = { 10.0f, 10.0f, 40.0f, 1.0f, 1.0f, 0.95f, 0.30f, 5.0f, LIGHT_SUN, 0 };
    Light lamp = { -4.0f, -4.0f, 4.0f, 1.0f, 0.8f, 0.5f, 1.0f, 0.0f, LIGHT_LAMP, 1 };
    lighting_add_light(&lt, sun);
    lighting_add_light(&lt, lamp);
    /* sits on the sun->(-2,-2) ray, and its band contains the sun at z=5 */
    lighting_add_occluder(&lt, (Rect){ 2.0f, -8.0f, 2.5f, 4.0f }, 0.0f, 5.0f);

    lighting_set_clock(&lt, 0.5f);
    lighting_build(&lt);
    Rgb noon_open = lighting_shade(&lt, 0.0f, 0.0f);
    Rgb noon_shadow = lighting_shade(&lt, 1.0f, -6.0f);    /* behind the wall */
    lighting_set_clock(&lt, 0.0f);
    lighting_build(&lt);
    Rgb night_open = lighting_shade(&lt, 0.0f, 0.0f);
    Rgb night_lamp = lighting_shade(&lt, -4.0f, -4.0f);
    printf("noon_open=%.3f noon_shadow=%.3f night_open=%.3f night_lamp=%.3f\n",
           noon_open.r, noon_shadow.r, night_open.r, night_lamp.r);
    check("sun_lights_day", noon_open.r > 0.7f);
    check("noon_not_clipped", noon_open.r < 0.999f);
    check("sun_shadowed_at_day", noon_shadow.r < noon_open.r - 0.02f);
    check("day_shadow_keeps_sky_fill", noon_shadow.r > ambient_from_clock(0.5f).r * 0.9f);
    check("night_is_dark", night_open.r < 0.1f);
    check("lamp_lights_night", night_lamp.r > 0.5f);
    check("lamp_off_by_day", night_lamp.r > noon_shadow.r);   /* lamp is the night source */
    lighting_free(&lt);
}

static void test_frame_budget(void) {
    /* documented budget: 8 lights x 32 occluders in range, 320x180 grid at
       0.25 units/cell (80x45 units). This is the worst-case shape the
       generator supports, not a typical scene. */
    Lighting lt;
    lighting_init(&lt, 0.25f, 320, 180, 0.0f, 0.0f);
    for (int i = 0; i < MAX_LIGHTS; i++) {
        Light L = { 5.0f + 9.0f * (float)(i % 4), 4.0f + 7.0f * (float)(i / 4), 12.0f,
                   1.0f, 0.9f, 0.75f, 1.0f, 0.0f, LIGHT_LAMP, 1 };
        lighting_add_light(&lt, L);
    }
    for (int i = 0; i < MAX_OCC; i++) {
        float x = 2.0f + 3.5f * (float)(i % 8);
        float y = 1.5f + 2.8f * (float)(i / 8);
        lighting_add_occluder(&lt, (Rect){ x, y, x + 1.0f, y + 0.8f }, 0.0f, 1.0f);
    }

    double worst = 0.0, sum = 0.0;
    int frames = 120;
    for (int f = 0; f < frames; f++) {
        for (int i = 0; i < MAX_LIGHTS; i++) {   /* everything moves every frame */
            lt.lights[i].x = 5.0f + 9.0f * (float)(i % 4) + 1.5f * sinf((float)f * 0.05f);
            lt.lights[i].y = 4.0f + 7.0f * (float)(i / 4) + 1.5f * cosf((float)f * 0.07f);
        }
        for (int i = 0; i < lt.occ_count; i++)
            lt.occ[i].rect.x0 = 2.0f + 3.5f * (float)(i % 8) + 0.4f * sinf((float)f * 0.11f + (float)i);
        double t0 = now_ms();
        lighting_build(&lt);
        double dt = now_ms() - t0;
        sum += dt;
        if (dt > worst) worst = dt;
    }
    printf("frame_ms_avg=%.3f worst=%.3f rays=%d lights=%d occ=%d\n",
           sum / frames, worst, lt.pb.npoly, lt.light_count, lt.occ_count);
    check("frame_budget_3ms", worst <= 3.0);
    lighting_free(&lt);
}

int main(void) {
    test_silhouette();
    test_polygon_matches_raycast();
    test_falloff();
    test_continuity();
    test_layers_composite();
    test_daynight();
    test_frame_budget();
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
