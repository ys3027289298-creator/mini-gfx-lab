/* raster: software triangle rasteriser.

   Pipeline: viewport transform -> near-plane clip (Sutherland-Hodgman in
   homogeneous clip space) -> backface cull -> snapped screen vertices ->
   edge-function raster (exact double edges, top-left fill rule) ->
   perspective-correct barycentrics -> depth test.

   The reference ("analytic") renderer is independent of the raster path:
   for a pixel it casts the ray through that pixel's centre into the
   triangle's plane and intersects it.  Coverage, depth and attributes are
   compared pixel by pixel.

   `make test` checks:
     - coverage, depth and interpolated attributes match the analytic ray
       result for every pixel of the screen
     - a quad split into two triangles has no gap and no pixel claimed twice
     - the nearer of two overlapping triangles wins every contested pixel
     - near-plane clipping leaves exactly the pixels whose ray hits in front
       of the near plane: no seam, no double coverage
     - back faces are rejected, winding is not ambiguous
     - fills 10,000 triangles and reports the time */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define MAX_VERTICES 4          /* triangle clipped against one plane */
#define BENCH_W 640
#define BENCH_H 360
#define BENCH_TRIS 10000

typedef struct { float x, y, z, w; } Vec4;      /* clip space */
typedef struct { float x, y; } Vec2;
typedef struct { float x, y, z; } Vec3;
typedef struct { float u, v; } Vec2uv;

typedef struct { Vec4 clip; Vec2uv uv; } ClipVert;

/* screen vertex after viewport transform */
typedef struct { float x, y; float w; Vec2uv uv; } SVert;

typedef struct {
    float near_plane;
    int width, height;
    int cull_backface;
    float fov_y, aspect;     /* only needed to build rays in the reference */
} Rasterizer;

typedef struct {
    int width, height;
    float* depth;            /* view distance w, +inf = empty */
    float* u;
    float* v;
    unsigned char* claims;   /* fill-rule ownership count, NULL if untracked */
    int covered;
    int pixels_written;
} Framebuffer;

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* ---- framebuffer ---------------------------------------------------- */

static int fb_init(Framebuffer* fb, int w, int h, int track_claims) {
    memset(fb, 0, sizeof(*fb));
    fb->width = w; fb->height = h;
    fb->depth = malloc((size_t)w*h*sizeof(float));
    fb->u = malloc((size_t)w*h*sizeof(float));
    fb->v = malloc((size_t)w*h*sizeof(float));
    fb->claims = track_claims ? malloc((size_t)w*h) : NULL;
    return fb->depth && fb->u && fb->v && (!track_claims || fb->claims);
}

static void fb_free(Framebuffer* fb) {
    free(fb->depth); free(fb->u); free(fb->v); free(fb->claims);
    memset(fb, 0, sizeof(*fb));
}

static void fb_clear(Framebuffer* fb) {
    int n = fb->width * fb->height;
    for (int i = 0; i < n; i++) fb->depth[i] = INFINITY;
    memset(fb->u, 0, (size_t)n*sizeof(float));
    memset(fb->v, 0, (size_t)n*sizeof(float));
    if (fb->claims) memset(fb->claims, 0, (size_t)n);
    fb->covered = 0; fb->pixels_written = 0;
}

/* ---- matrices / transform ------------------------------------------- */

static Vec4 mat4_mul(const float m[16], Vec4 p) {
    Vec4 r;
    r.x = m[0]*p.x + m[4]*p.y + m[8]*p.z  + m[12]*p.w;
    r.y = m[1]*p.x + m[5]*p.y + m[9]*p.z  + m[13]*p.w;
    r.z = m[2]*p.x + m[6]*p.y + m[10]*p.z + m[14]*p.w;
    r.w = m[3]*p.x + m[7]*p.y + m[11]*p.z + m[15]*p.w;
    return r;
}

/* symmetric perspective, z_ndc in [-1,1]; w_clip = -z_view */
static void perspective(float fov_y, float aspect, float zn, float zf, float m[16]) {
    float t = 1.0f / tanf(fov_y * 0.5f);
    memset(m, 0, 16*sizeof(float));
    m[0] = t/aspect; m[5] = t;
    m[10] = (zf+zn)/(zn-zf); m[14] = 2.0f*zf*zn/(zn-zf);
    m[11] = -1.0f;
}

/* view space (camera at origin, -z forward) -> clip space */
static Vec4 view_to_clip(const float p[16], Vec3 v) {
    return mat4_mul(p, (Vec4){ v.x, v.y, v.z, 1.0f });
}

/* ---- clipping ------------------------------------------------------- */

static ClipVert clip_lerp(const ClipVert* a, const ClipVert* b, float near) {
    float t = (near - a->clip.w) / (b->clip.w - a->clip.w);
    ClipVert o;
    o.clip.x = a->clip.x + t*(b->clip.x - a->clip.x);
    o.clip.y = a->clip.y + t*(b->clip.y - a->clip.y);
    o.clip.z = a->clip.z + t*(b->clip.z - a->clip.z);
    o.clip.w = a->clip.w + t*(b->clip.w - a->clip.w);
    o.uv.u   = a->uv.u   + t*(b->uv.u   - a->uv.u);
    o.uv.v   = a->uv.v   + t*(b->uv.v   - a->uv.v);
    return o;
}

/* keep the part with w >= near; a triangle cut by one plane gives <= 4 verts */
static int clip_near(const ClipVert in[], int n, float near, ClipVert out[]) {
    int m = 0;
    for (int i = 0; i < n; i++) {
        const ClipVert* a = &in[i];
        const ClipVert* b = &in[(i+1) % n];
        int ina = a->clip.w >= near, inb = b->clip.w >= near;
        if (ina) out[m++] = *a;
        if (ina != inb) out[m++] = clip_lerp(a, b, near);
    }
    return m;
}

/* clip + viewport.  Returns the screen polygon, or 0 if fully clipped. */
static int project_poly(const Rasterizer* r, const Vec4 clip[3], const Vec2uv uv[3],
                        SVert out[MAX_VERTICES]) {
    ClipVert in[3], poly[MAX_VERTICES];
    for (int i = 0; i < 3; i++) { in[i].clip = clip[i]; in[i].uv = uv[i]; }
    int n = clip_near(in, 3, r->near_plane, poly);
    if (n < 3) return 0;

    for (int i = 0; i < n; i++) {
        float inv_w = 1.0f / poly[i].clip.w;
        float ndc_x = poly[i].clip.x * inv_w, ndc_y = poly[i].clip.y * inv_w;
        float sx = (ndc_x * 0.5f + 0.5f) * (float)r->width;
        float sy = (1.0f - (ndc_y * 0.5f + 0.5f)) * (float)r->height;
        out[i] = (SVert){ sx, sy, poly[i].clip.w, poly[i].uv };
    }
    return n;
}

/* ---- raster --------------------------------------------------------- */

/* Screen vertices are held in float, but edge functions are evaluated in
   double.  A float->double conversion is exact, so for any two vertices the
   difference (b.x-a.x) is exact and each product below is exact; two
   triangles that share an edge evaluate the same three edge functions with
   opposite signs.  Watertightness is therefore arithmetic, not a tolerance. */
static double edge_sd(const SVert* a, const SVert* b, double px, double py) {
    return ((double)b->x - (double)a->x) * (py - (double)a->y) -
           ((double)b->y - (double)a->y) * (px - (double)a->x);
}

/* exactly one owner for an edge shared by two triangles */
static int is_top_left(const SVert* a, const SVert* b) {
    double dx = (double)b->x - (double)a->x;
    double dy = (double)b->y - (double)a->y;
    return dy < 0.0 || (dy == 0.0 && dx > 0.0);
}

static int inside_edge(double e, int top_left) {
    return e > 0.0 || (e == 0.0 && top_left);
}

/* rasterise one screen triangle.  Returns the number of pixels whose centre
   passes the fill rule (whether or not they then lose the depth test). */
static int raster_tri(Rasterizer* r, Framebuffer* fb, const SVert v[3]) {
    double area = edge_sd(&v[0], &v[1], (double)v[2].x, (double)v[2].y);
    if (area == 0.0) return 0;
    /* viewport flips y, so front-facing (CCW in world seen from the camera)
       triangles come out with negative signed screen area */
    if (r->cull_backface && area > 0.0) return 0;
    SVert sv[3] = { v[0], v[1], v[2] };
    if (area < 0.0) {
        sv[1] = v[2]; sv[2] = v[1];
        area = -area;
    }

    float minx = fminf(fminf(sv[0].x, sv[1].x), sv[2].x);
    float maxx = fmaxf(fmaxf(sv[0].x, sv[1].x), sv[2].x);
    float miny = fminf(fminf(sv[0].y, sv[1].y), sv[2].y);
    float maxy = fmaxf(fmaxf(sv[0].y, sv[1].y), sv[2].y);

    int x0 = (int)floorf(minx), x1 = (int)ceilf(maxx);
    int y0 = (int)floorf(miny), y1 = (int)ceilf(maxy);
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > r->width - 1)  x1 = r->width - 1;
    if (y1 > r->height - 1) y1 = r->height - 1;
    if (x0 > x1 || y0 > y1) return 0;

    int tl0 = is_top_left(&sv[1], &sv[2]);
    int tl1 = is_top_left(&sv[2], &sv[0]);
    int tl2 = is_top_left(&sv[0], &sv[1]);

    double inv_area = 1.0 / area;
    float iw0 = 1.0f / sv[0].w, iw1 = 1.0f / sv[1].w, iw2 = 1.0f / sv[2].w;
    int claimed = 0;

    for (int y = y0; y <= y1; y++) {
        double py = (double)y + 0.5;
        size_t row = (size_t)y * (size_t)fb->width;
        for (int x = x0; x <= x1; x++) {
            double px = (double)x + 0.5;
            double e0 = edge_sd(&sv[1], &sv[2], px, py);
            double e1 = edge_sd(&sv[2], &sv[0], px, py);
            double e2 = edge_sd(&sv[0], &sv[1], px, py);
            if (!inside_edge(e0, tl0) || !inside_edge(e1, tl1) || !inside_edge(e2, tl2))
                continue;

            size_t idx = row + (size_t)x;
            claimed++;
            if (fb->claims) fb->claims[idx]++;

            float b0 = (float)(e0 * inv_area);
            float b1 = (float)(e1 * inv_area);
            float b2 = (float)(e2 * inv_area);

            /* 1/w is affine in screen space; interpolate it, not w */
            float iw = b0*iw0 + b1*iw1 + b2*iw2;
            float z = 1.0f / iw;
            if (z >= fb->depth[idx]) continue;

            float u = (b0*sv[0].uv.u*iw0 + b1*sv[1].uv.u*iw1 + b2*sv[2].uv.u*iw2) / iw;
            float vv = (b0*sv[0].uv.v*iw0 + b1*sv[1].uv.v*iw1 + b2*sv[2].uv.v*iw2) / iw;

            if (fb->depth[idx] == INFINITY) fb->covered++;
            fb->depth[idx] = z;
            fb->u[idx] = u;
            fb->v[idx] = vv;
            fb->pixels_written++;
        }
    }
    return claimed;
}

/* transform, clip and rasterise one triangle; returns claims and the
   clipped screen polygon via out (for the reference renderer). */
static int draw_tri(Rasterizer* r, Framebuffer* fb,
                    const Vec4 clip[3], const Vec2uv uv[3],
                    SVert out[MAX_VERTICES], int* out_n) {
    SVert poly[MAX_VERTICES];
    int n = project_poly(r, clip, uv, poly);
    if (out_n) *out_n = n;
    if (out && n) memcpy(out, poly, (size_t)n * sizeof(SVert));
    int claimed = 0;
    for (int i = 1; i + 1 < n; i++) {
        SVert tri[3] = { poly[0], poly[i], poly[i+1] };
        claimed += raster_tri(r, fb, tri);
    }
    return claimed;
}

/* ---- analytic reference --------------------------------------------- */

/* Ray through the centre of pixel (px,py), from the eye at the origin.
   Uses the camera parameters, not the rasteriser's matrices. */
static Vec3 pixel_ray(const Rasterizer* r, int px, int py) {
    float ty = tanf(r->fov_y * 0.5f);
    float ndc_x = 2.0f * ((float)px + 0.5f) / (float)r->width - 1.0f;
    float ndc_y = 1.0f - 2.0f * ((float)py + 0.5f) / (float)r->height;
    return (Vec3){ ndc_x * ty * r->aspect, ndc_y * ty, -1.0f };
}

/* Möller–Trumbore against the unclipped triangle, ray origin = eye.
   Returns the hit distance (view-space w) and interpolated attributes. */
static int ray_tri(const Vec3 tri[3], const Vec2uv uv[3], Vec3 dir, float near,
                   float* out_w, Vec2uv* out_uv) {
    Vec3 e1 = { tri[1].x-tri[0].x, tri[1].y-tri[0].y, tri[1].z-tri[0].z };
    Vec3 e2 = { tri[2].x-tri[0].x, tri[2].y-tri[0].y, tri[2].z-tri[0].z };
    Vec3 pv = { dir.y*e2.z - dir.z*e2.y, dir.z*e2.x - dir.x*e2.z, dir.x*e2.y - dir.y*e2.x };
    float det = e1.x*pv.x + e1.y*pv.y + e1.z*pv.z;
    if (fabsf(det) < 1e-12f) return 0;
    float inv = 1.0f / det;
    Vec3 tv = { -tri[0].x, -tri[0].y, -tri[0].z };   /* origin - v0, origin = eye */
    float u = (tv.x*pv.x + tv.y*pv.y + tv.z*pv.z) * inv;
    Vec3 qv = { tv.y*e1.z - tv.z*e1.y, tv.z*e1.x - tv.x*e1.z, tv.x*e1.y - tv.y*e1.x };
    float v = (dir.x*qv.x + dir.y*qv.y + dir.z*qv.z) * inv;
    if (u < -1e-6f || v < -1e-6f || u + v > 1.0f + 1e-6f) return 0;
    float t = (e2.x*qv.x + e2.y*qv.y + e2.z*qv.z) * inv;
    if (t < near) return 0;          /* clipped away by the near plane */
    *out_w = t;
    float b0 = 1.0f - u - v;
    out_uv->u = b0*uv[0].u + u*uv[1].u + v*uv[2].u;
    out_uv->v = b0*uv[0].v + u*uv[1].v + v*uv[2].v;
    return 1;
}

/* Analytic result for a screen-space polygon: it *is* the projected
   geometry, so its barycentrics at a pixel centre are the exact
   perspective-correct barycentrics.  This is the closed-form ground truth
   the raster must match, and it shares no code path with the edge test. */
static int poly_sample(const SVert poly[], int n, int px, int py,
                       float* out_w, Vec2uv* out_uv) {
    float best = INFINITY;
    int hit = 0;
    double cx = px + 0.5, cy = py + 0.5;
    for (int i = 1; i + 1 < n; i++) {
        const SVert* a = &poly[0];
        const SVert* b = &poly[i];
        const SVert* c = &poly[i+1];
        double area = ((double)b->x-a->x)*((double)c->y-a->y) -
                      ((double)b->y-a->y)*((double)c->x-a->x);
        if (area == 0.0) continue;
        double b0 = (((double)b->x-cx)*((double)c->y-cy) -
                     ((double)b->y-cy)*((double)c->x-cx)) / area;
        double b1 = (((double)c->x-cx)*((double)a->y-cy) -
                     ((double)c->y-cy)*((double)a->x-cx)) / area;
        double b2 = 1.0 - b0 - b1;
        if (b0 < 0.0 || b1 < 0.0 || b2 < 0.0) continue;

        double iw = b0/a->w + b1/b->w + b2/c->w;
        if (iw <= 0.0) continue;
        float w = (float)(1.0 / iw);
        if (w < best) {
            best = w;
            Vec2uv uv;
            uv.u = (float)((b0*a->uv.u/a->w + b1*b->uv.u/b->w + b2*c->uv.u/c->w) / iw);
            uv.v = (float)((b0*a->uv.v/a->w + b1*b->uv.v/b->w + b2*c->uv.v/c->w) / iw);
            *out_uv = uv;
            hit = 1;
        }
    }
    *out_w = best;
    return hit;
}

/* ---- tests ---------------------------------------------------------- */

static void setup_camera(Rasterizer* r, float fov, int w, int h, float near) {
    r->near_plane = near;
    r->width = w; r->height = h;
    r->cull_backface = 0;
    r->fov_y = fov;
    r->aspect = (float)w / (float)h;
}

/* 1. viewport: a known point lands on the expected pixel */
static void test_viewport(void) {
    Rasterizer r; setup_camera(&r, 1.0f, 200, 160, 0.1f);
    float proj[16]; perspective(r.fov_y, r.aspect, 0.1f, 100.0f, proj);
    float ty = tanf(r.fov_y * 0.5f);

    SVert p[MAX_VERTICES];
    Vec2uv uv[3] = { {0,0}, {0,0}, {0,0} };
    Vec4 clip[3] = {
        view_to_clip(proj, (Vec3){0, 0, -4}),
        view_to_clip(proj, (Vec3){ty*r.aspect*4*0.5f, 0, -4}),
        view_to_clip(proj, (Vec3){0, ty*4*0.5f, -4}),
    };
    int n = project_poly(&r, clip, uv, p);
    check("viewport_center", n == 3 &&
          fabsf(p[0].x - 100.0f) < 1e-3f && fabsf(p[0].y - 80.0f) < 1e-3f);
    check("viewport_right", fabsf(p[1].x - 150.0f) < 1e-3f);
    check("viewport_up", p[2].y < 80.0f && fabsf(p[2].y - 40.0f) < 1e-3f);
    check("viewport_depth", fabsf(p[0].w - 4.0f) < 1e-4f);
}

/* 2. every pixel of an unclipped triangle: coverage, depth and attributes
      must equal the analytic closed form */
static void test_pixel_parity(void) {
    Rasterizer r; setup_camera(&r, 1.0f, 200, 160, 0.1f);
    Framebuffer fb; fb_init(&fb, r.width, r.height, 0); fb_clear(&fb);
    float proj[16]; perspective(r.fov_y, r.aspect, 0.1f, 100.0f, proj);

    Vec3 verts[3] = {
        { -1.5f, -1.0f, -4.0f },
        {  2.1f, -0.7f, -6.3f },
        {  0.3f,  1.7f, -5.1f },
    };
    Vec2uv uv[3] = { {0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f} };
    Vec4 clip[3];
    for (int i = 0; i < 3; i++) clip[i] = view_to_clip(proj, verts[i]);

    SVert poly[MAX_VERTICES];
    int n = 0;
    draw_tri(&r, &fb, clip, uv, poly, &n);
    check("parity_clip_none", n == 3);

    int cov_mismatch = 0, depth_mismatch = 0, uv_mismatch = 0, hits = 0;
    float worst_depth = 0.0f, worst_uv = 0.0f;
    for (int y = 0; y < r.height; y++) {
        for (int x = 0; x < r.width; x++) {
            float aw; Vec2uv auv;
            int a = poly_sample(poly, n, x, y, &aw, &auv);
            float fz = fb.depth[y*r.width + x];
            int f = fz != INFINITY;
            if (a != f) { cov_mismatch++; continue; }
            if (!a) continue;
            hits++;
            float dz = fabsf(fz - aw) / aw;
            float du = fmaxf(fabsf(fb.u[y*r.width + x] - auv.u),
                             fabsf(fb.v[y*r.width + x] - auv.v));
            if (dz > worst_depth) worst_depth = dz;
            if (du > worst_uv) worst_uv = du;
            if (dz > 1e-5f) depth_mismatch++;
            if (du > 1e-5f) uv_mismatch++;
        }
    }
    printf("parity_pixels=%d worst_depth_rel=%.2e worst_uv=%.2e\n",
           hits, worst_depth, worst_uv);
    check("parity_coverage", cov_mismatch == 0 && hits > 1000);
    check("parity_depth", depth_mismatch == 0);
    check("parity_uv", uv_mismatch == 0);
    fb_free(&fb);
}

/* 3. quad split along a diagonal: no hole, no pixel claimed twice */
static void test_watertight(void) {
    Rasterizer r; setup_camera(&r, 1.0f, 200, 160, 0.1f);
    Framebuffer fb; fb_init(&fb, r.width, r.height, 1); fb_clear(&fb);
    float proj[16]; perspective(r.fov_y, r.aspect, 0.1f, 100.0f, proj);

    Vec3 q[4] = { {-2.0f,-2.0f,-5.0f}, {2.0f,-2.0f,-4.2f}, {2.0f,2.0f,-4.2f}, {-2.0f,2.0f,-5.0f} };
    Vec2uv uv[4] = { {0,0}, {1,0}, {1,1}, {0,1} };
    int tri_idx[2][3] = { {0,1,2}, {0,2,3} };
    SVert polys[2][MAX_VERTICES];
    int pn[2];
    for (int t = 0; t < 2; t++) {
        Vec4 clip[3]; Vec2uv tuv[3];
        for (int i = 0; i < 3; i++) {
            clip[i] = view_to_clip(proj, q[tri_idx[t][i]]);
            tuv[i] = uv[tri_idx[t][i]];
        }
        draw_tri(&r, &fb, clip, tuv, polys[t], &pn[t]);
    }

    int gap = 0, outside = 0, multi = 0, hits = 0;
    for (int y = 0; y < r.height; y++) {
        for (int x = 0; x < r.width; x++) {
            float w0, w1; Vec2uv u0, u1;
            int a0 = poly_sample(polys[0], pn[0], x, y, &w0, &u0);
            int a1 = poly_sample(polys[1], pn[1], x, y, &w1, &u1);
            int inside = a0 || a1;
            int covered = fb.depth[y*r.width + x] != INFINITY;
            unsigned char c = fb.claims[y*r.width + x];
            if (inside) hits++;
            if (inside && !covered) gap++;
            if (!inside && covered) outside++;
            if (c > 1) multi++;
        }
    }
    printf("watertight_pixels=%d\n", hits);
    check("watertight_no_gap", gap == 0);
    check("watertight_no_outside", outside == 0);
    check("watertight_single_owner", multi == 0);
    check("watertight_writes", fb.pixels_written == fb.covered);
    fb_free(&fb);
}

/* 4. occlusion: the nearer triangle wins every contested pixel */
static void test_occlusion(void) {
    Rasterizer r; setup_camera(&r, 1.0f, 200, 160, 0.1f);
    Framebuffer fb; fb_init(&fb, r.width, r.height, 0); fb_clear(&fb);
    float proj[16]; perspective(r.fov_y, r.aspect, 0.1f, 100.0f, proj);

    Vec3 near_tri[3] = { {-1.7f,-1.3f,-3.0f}, {1.3f,-1.1f,-3.1f}, {-0.1f,1.4f,-3.0f} };
    Vec3 far_tri[3]  = { {-0.3f,-2.1f,-7.0f}, {2.2f,-0.4f,-6.8f}, {0.5f,1.7f,-7.2f} };
    Vec2uv nuv[3] = { {0.1f,0.2f}, {0.9f,0.3f}, {0.4f,0.8f} };
    Vec2uv fuv[3] = { {0.0f,0.0f}, {1.0f,0.0f}, {0.0f,1.0f} };

    Vec4 nclip[3], fclip[3];
    for (int i = 0; i < 3; i++) {
        nclip[i] = view_to_clip(proj, near_tri[i]);
        fclip[i] = view_to_clip(proj, far_tri[i]);
    }
    SVert np[MAX_VERTICES], fp[MAX_VERTICES];
    int nn, fn;

    /* far-then-near and near-then-far must agree: a depth test that is
       missing (or that only wins because of draw order) shows up here */
    Framebuffer fb2;
    fb_init(&fb2, r.width, r.height, 0); fb_clear(&fb2);
    draw_tri(&r, &fb, fclip, fuv, fp, &fn);
    draw_tri(&r, &fb, nclip, nuv, np, &nn);
    draw_tri(&r, &fb2, nclip, nuv, NULL, NULL);
    draw_tri(&r, &fb2, fclip, fuv, NULL, NULL);

    int order_diff = 0;
    for (int y = 0; y < r.height; y++)
        for (int x = 0; x < r.width; x++) {
            int i = y*r.width + x;
            if (fb.depth[i] != fb2.depth[i] || fb.u[i] != fb2.u[i] || fb.v[i] != fb2.v[i])
                order_diff++;
        }

    int mism = 0, contested = 0;
    for (int y = 0; y < r.height; y++) {
        for (int x = 0; x < r.width; x++) {
            float wn, wf; Vec2uv un, uf;
            int hn = poly_sample(np, nn, x, y, &wn, &un);
            int hf = poly_sample(fp, fn, x, y, &wf, &uf);
            if (!hn && !hf) {
                if (fb.depth[y*r.width + x] != INFINITY) mism++;
                continue;
            }
            int use_near = hn && (!hf || wn <= wf);
            float wz = use_near ? wn : wf;
            Vec2uv uz = use_near ? un : uf;
            if (hn && hf) contested++;
            float fz = fb.depth[y*r.width + x];
            if (fz == INFINITY) { mism++; continue; }
            if (fabsf(fz - wz) / wz > 1e-5f) mism++;
            if (fabsf(fb.u[y*r.width + x] - uz.u) > 1e-5f ||
                fabsf(fb.v[y*r.width + x] - uz.v) > 1e-5f) mism++;
        }
    }
    check("occlusion_order_independent", order_diff == 0);
    printf("occlusion_contested=%d\n", contested);
    check("occlusion_nearest_wins", mism == 0 && contested > 100);
    fb_free(&fb2);
    fb_free(&fb);
}

/* 5. near clipping: coverage must equal the set of rays that hit the
      original triangle in front of the near plane; the cut must not leave a
      seam or cover a pixel twice */
static void test_near_clip(void) {
    Rasterizer r; setup_camera(&r, 1.0f, 200, 160, 0.35f);
    Framebuffer fb; fb_init(&fb, r.width, r.height, 1); fb_clear(&fb);
    float proj[16]; perspective(r.fov_y, r.aspect, 0.35f, 100.0f, proj);

    /* one vertex behind the eye, two in front: crosses the near plane */
    Vec3 verts[3] = {
        { -0.6f, -0.5f, -2.0f },
        {  0.6f, -0.5f, -2.0f },
        {  0.0f,  0.0f, -0.2f },      /* inside the near plane */
    };
    Vec2uv uv[3] = { {0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f} };
    Vec4 clip[3];
    for (int i = 0; i < 3; i++) clip[i] = view_to_clip(proj, verts[i]);
    SVert poly[MAX_VERTICES];
    int n = 0;
    draw_tri(&r, &fb, clip, uv, poly, &n);
    check("clip_polygon_quad", n == 4);

    int cov_mismatch = 0, depth_mismatch = 0, uv_mismatch = 0;
    int hits = 0, multi = 0;
    float worst_depth = 0.0f, worst_uv = 0.0f;
    for (int y = 0; y < r.height; y++) {
        for (int x = 0; x < r.width; x++) {
            Vec3 dir = pixel_ray(&r, x, y);
            float aw; Vec2uv auv;
            int a = ray_tri(verts, uv, dir, r.near_plane, &aw, &auv);
            float fz = fb.depth[y*r.width + x];
            int f = fz != INFINITY;
            if (a != f) { cov_mismatch++; continue; }
            if (!a) continue;
            hits++;
            float dz = fabsf(fz - aw) / aw;
            float du = fmaxf(fabsf(fb.u[y*r.width + x] - auv.u),
                             fabsf(fb.v[y*r.width + x] - auv.v));
            if (dz > worst_depth) worst_depth = dz;
            if (du > worst_uv) worst_uv = du;
            if (dz > 1e-5f) depth_mismatch++;
            if (du > 1e-5f) uv_mismatch++;
            if (fb.claims[y*r.width + x] > 1) multi++;
        }
    }
    /* second, independent reference: the clipped polygon's own
       barycentrics.  Both references must agree with each other and with
       the framebuffer. */
    int ref_mismatch = 0;
    for (int y = 0; y < r.height; y++) {
        for (int x = 0; x < r.width; x++) {
            Vec3 dir = pixel_ray(&r, x, y);
            float aw, bw; Vec2uv auv, buv;
            int a = ray_tri(verts, uv, dir, r.near_plane, &aw, &auv);
            int b = poly_sample(poly, n, x, y, &bw, &buv);
            if (a != b) { ref_mismatch++; continue; }
            if (!a) continue;
            if (fabsf(aw - bw) / aw > 1e-5f ||
                fabsf(auv.u - buv.u) > 1e-5f || fabsf(auv.v - buv.v) > 1e-5f)
                ref_mismatch++;
        }
    }
    printf("clip_pixels=%d worst_depth_rel=%.2e worst_uv=%.2e\n",
           hits, worst_depth, worst_uv);
    check("clip_references_agree", ref_mismatch == 0);
    check("clip_coverage_matches_raycast", cov_mismatch == 0 && hits > 500);
    check("clip_depth_matches_raycast", depth_mismatch == 0);
    check("clip_uv_matches_raycast", uv_mismatch == 0);
    check("clip_no_double_cover", multi == 0);

    /* a triangle entirely behind the near plane must be dropped whole */
    fb_clear(&fb);
    Vec3 behind[3] = { {-1,-1,0.2f}, {1,-1,0.15f}, {0,1,0.05f} };
    Vec4 bclip[3];
    for (int i = 0; i < 3; i++) bclip[i] = view_to_clip(proj, behind[i]);
    int bn = 0;
    draw_tri(&r, &fb, bclip, uv, poly, &bn);
    check("clip_fully_behind", bn == 0 && fb.covered == 0);
    fb_free(&fb);
}

/* 6. back-face culling: reversing the winding hides the triangle */
static void test_backface(void) {
    Rasterizer r; setup_camera(&r, 1.0f, 200, 160, 0.1f);
    Framebuffer fb; fb_init(&fb, r.width, r.height, 0);
    float proj[16]; perspective(r.fov_y, r.aspect, 0.1f, 100.0f, proj);
    Vec3 v[3] = { {-1,-1,-5}, {1,-0.8f,-5.3f}, {0,1.2f,-5} };
    Vec2uv uv[3] = { {0,0}, {1,0}, {0,1} };

    fb_clear(&fb);
    Vec4 c[3]; for (int i = 0; i < 3; i++) c[i] = view_to_clip(proj, v[i]);
    draw_tri(&r, &fb, c, uv, NULL, NULL);
    int front = fb.covered;

    fb_clear(&fb);
    Vec3 rv[3] = { v[0], v[2], v[1] };
    for (int i = 0; i < 3; i++) c[i] = view_to_clip(proj, rv[i]);
    draw_tri(&r, &fb, c, uv, NULL, NULL);
    int back = fb.covered;

    fb_clear(&fb);
    r.cull_backface = 1;
    for (int i = 0; i < 3; i++) c[i] = view_to_clip(proj, v[i]);
    draw_tri(&r, &fb, c, uv, NULL, NULL);
    int front_culled = fb.covered;

    r.cull_backface = 1;
    for (int i = 0; i < 3; i++) c[i] = view_to_clip(proj, rv[i]);
    fb_clear(&fb);
    draw_tri(&r, &fb, c, uv, NULL, NULL);
    int back_culled = fb.covered;

    printf("backface_front=%d back=%d\n", front, back);
    check("backface_front_drawn", front > 1000);
    check("backface_back_drawn_without_cull", back > 1000);
    check("backface_front_survives_cull", front_culled == front);
    check("backface_back_removed_by_cull", back_culled == 0);
    fb_free(&fb);
}

/* 6b. fill rule: shared edges that pass exactly through pixel centres.
       Each such pixel must be claimed by exactly one of the two triangles.
       Without the top-left rule both claim it (double write), or neither
       does (a one-pixel seam). */
static void test_edge_ownership(void) {
    Rasterizer r; setup_camera(&r, 1.0f, 64, 64, 0.1f);
    Framebuffer fb; fb_init(&fb, 64, 64, 1);

    int bad_multi = 0, bad_gap = 0, cases = 0;
    /* squares whose diagonals run through pixel centres: y = x + c and
       y = -x + c, both with c an integer */
    for (int c = -8; c <= 8; c++) {
        for (int sgn = -1; sgn <= 1; sgn += 2) {
            float cx = 32.0f, cy = 32.0f, s = 12.0f;
            SVert q[4] = {
                { cx-s, cy-s, 5.0f, {0,0} }, { cx+s, cy-s, 5.0f, {1,0} },
                { cx+s, cy+s, 5.0f, {1,1} }, { cx-s, cy+s, 5.0f, {0,1} },
            };
            if (sgn < 0) { SVert t = q[1]; q[1] = q[3]; q[3] = t; }
            /* diagonal: q0-q2 passes through (32,32); shift the whole quad
               by c pixels so it passes through a pixel centre for c != 0 */
            for (int i = 0; i < 4; i++) { q[i].x += (float)c; q[i].y += (float)c; }

            fb_clear(&fb);
            int idx[2][3] = { {0,1,2}, {0,2,3} };
            for (int t = 0; t < 2; t++) {
                SVert tri[3] = { q[idx[t][0]], q[idx[t][1]], q[idx[t][2]] };
                raster_tri(&r, &fb, tri);
            }
            int on_edge = 0;
            for (int y = 0; y < 64; y++)
                for (int x = 0; x < 64; x++) {
                    unsigned char cl = fb.claims[y*64 + x];
                    if (cl > 1) bad_multi++;
                    if (cl == 0) continue;
                    float px = (float)x + 0.5f, py = (float)y + 0.5f;
                    /* pixel centre exactly on the shared diagonal? */
                    if (fabsf((py - q[0].y) - (px - q[0].x)) < 1e-6f) on_edge++;
                }
            if (on_edge < 8) bad_gap++;   /* case did not exercise the rule */
            cases++;
        }
    }
    printf("edge_ownership_cases=%d\n", cases);
    check("edge_ownership_exercised", bad_gap == 0);
    check("edge_ownership_single_claim", bad_multi == 0);
    fb_free(&fb);
}

/* 6c. a triangle half off-screen must not write outside the framebuffer,
       and its on-screen half must still match the analytic result */
static void test_boundary(void) {
    Rasterizer r; setup_camera(&r, 1.0f, 64, 48, 0.1f);
    Framebuffer fb; fb_init(&fb, 64, 48, 0);
    float proj[16]; perspective(r.fov_y, r.aspect, 0.1f, 100.0f, proj);

    Vec3 v[3] = { {-40.0f, -30.0f, -5.0f}, {40.0f, -28.0f, -5.0f}, {0.0f, 30.0f, -5.0f} };
    Vec2uv uv[3] = { {0,0}, {1,0}, {0,1} };
    Vec4 clip[3];
    for (int i = 0; i < 3; i++) clip[i] = view_to_clip(proj, v[i]);
    fb_clear(&fb);
    SVert poly[MAX_VERTICES];
    int n = 0;
    draw_tri(&r, &fb, clip, uv, poly, &n);

    int mism = 0, hits = 0;
    for (int y = 0; y < r.height; y++)
        for (int x = 0; x < r.width; x++) {
            float aw; Vec2uv auv;
            int a = poly_sample(poly, n, x, y, &aw, &auv);
            int f = fb.depth[y*r.width + x] != INFINITY;
            if (a != f) mism++;
            if (a) hits++;
        }
    printf("boundary_pixels=%d\n", hits);
    /* the x=0 and x=width-1 columns are reachable by the clipped triangle,
       so a write at x<0/x>=w would show up as a coverage mismatch */
    check("boundary_clipped_coverage", mism == 0 && hits > 1000);
    fb_free(&fb);
}

/* 7. timing: 10k triangles */
static void bench_10k(void) {
    Rasterizer r; setup_camera(&r, 1.0f, BENCH_W, BENCH_H, 0.1f);
    r.cull_backface = 1;
    Framebuffer fb; fb_init(&fb, r.width, r.height, 0);
    float proj[16]; perspective(r.fov_y, r.aspect, 0.1f, 200.0f, proj);

    enum { COLS = 100, ROWS = 50, NTRIS = COLS*ROWS*2 };
    Vec3 tri[NTRIS][3];
    Vec2uv uvs[NTRIS][3];
    Vec4 clip[NTRIS][3];

    unsigned seed = 12345;
    for (int j = 0; j < ROWS; j++) {
        for (int i = 0; i < COLS; i++) {
            seed = seed * 1103515245u + 12345u;
            float jx = ((seed >> 16) & 0xff) / 255.0f - 0.5f;
            float jy = ((seed >> 8)  & 0xff) / 255.0f - 0.5f;
            float x = -9.0f + 18.0f * (i + 0.5f) / COLS + jx * 0.02f;
            float y = -4.5f +  9.0f * (j + 0.5f) / ROWS + jy * 0.02f;
            float z = -1.5f - 14.0f * ((i + j) % 17) / 17.0f;
            float s = 0.12f;
            Vec3 q[4] = {
                { x-s, y-s, z }, { x+s, y-s, z+0.05f },
                { x+s, y+s, z+0.05f }, { x-s, y+s, z },
            };
            Vec2uv quv[4] = { {0,0}, {1,0}, {1,1}, {0,1} };
            int t = (j*COLS + i) * 2;
            int idx[2][3] = { {0,1,2}, {0,2,3} };
            for (int k = 0; k < 2; k++) {
                for (int m = 0; m < 3; m++) {
                    tri[t+k][m] = q[idx[k][m]];
                    uvs[t+k][m] = quv[idx[k][m]];
                }
            }
        }
    }
    for (int t = 0; t < NTRIS; t++)
        for (int m = 0; m < 3; m++)
            clip[t][m] = view_to_clip(proj, tri[t][m]);

    /* warm up, then measure several runs and keep the median */
    long long best_ns = -1, times[7];
    int covered = 0;
    for (int run = 0; run < 7; run++) {
        fb_clear(&fb);
        struct timespec a, b;
        clock_gettime(CLOCK_MONOTONIC, &a);
        for (int t = 0; t < NTRIS; t++)
            draw_tri(&r, &fb, clip[t], uvs[t], NULL, NULL);
        clock_gettime(CLOCK_MONOTONIC, &b);
        long long ns = (long long)(b.tv_sec - a.tv_sec)*1000000000LL +
                       (b.tv_nsec - a.tv_nsec);
        times[run] = ns;
        if (run > 0) {  /* first run pays page faults */
            if (best_ns < 0 || ns < best_ns) best_ns = ns;
        }
        covered = fb.covered;
    }
    for (int i = 0; i < 7; i++)
        for (int j = i + 1; j < 7; j++)
            if (times[j] < times[i]) { long long t = times[i]; times[i] = times[j]; times[j] = t; }
    long long median_ns = times[3];

    double ms_best = (double)best_ns / 1e6;
    double ms_med = (double)median_ns / 1e6;
    double mtri_s = NTRIS / ((double)best_ns / 1e9) / 1e6;
    printf("fill_10k_screen=%dx%d\n", r.width, r.height);
    printf("fill_10k_triangles=%d\n", NTRIS);
    printf("fill_10k_pixels_covered=%d\n", covered);
    printf("fill_10k_ms_best=%.2f\n", ms_best);
    printf("fill_10k_ms_median=%.2f\n", ms_med);
    printf("fill_10k_mtris_per_s=%.2f\n", mtri_s);
    check("fill_10k_ran", best_ns > 0 && covered > 0);
    fb_free(&fb);
}

int main(void) {
    test_viewport();
    test_pixel_parity();
    test_watertight();
    test_occlusion();
    test_near_clip();
    test_backface();
    test_edge_ownership();
    test_boundary();
    bench_10k();

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
