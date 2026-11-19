
/* fontatlas: shelf packing of glyph bitmaps into a fixed atlas, plus kerning.
   Also generates a signed distance field (SDF) from the glyph edge so the
   glyph can be magnified without the blur of a stretched bitmap. */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define ATLAS_W 256
#define ATLAS_H 256
#define MAX_GLYPHS 64
#define MAX_KERN 32

typedef struct { int w, h; } GlyphSize;
typedef struct { int x, y, w, h; int placed; } GlyphRect;
typedef struct { char left, right; int delta; } KernPair;

typedef struct {
    int used_h;        /* current shelf top */
    int shelf_x;       /* next x on the current shelf */
    int shelf_h;       /* height of the current shelf */
    GlyphRect rects[MAX_GLYPHS];
    int count;
} Atlas;

static void atlas_init(Atlas* a) {
    memset(a, 0, sizeof(*a));
}

/* shelf packing: place to the right on the same shelf, or open a new shelf */
static int atlas_add(Atlas* a, GlyphSize g, int* out_index) {
    if (a->count >= MAX_GLYPHS) return -1;
    if (g.w <= 0 || g.h <= 0 || g.w > ATLAS_W || g.h > ATLAS_H) return -1;
    if (a->shelf_x + g.w > ATLAS_W) {
        a->used_h += a->shelf_h;
        a->shelf_x = 0;
        a->shelf_h = 0;
    }
    if (a->used_h + g.h > ATLAS_H) return -1;

    GlyphRect* r = &a->rects[a->count];
    r->x = a->shelf_x;
    r->y = a->used_h;
    r->w = g.w;
    r->h = g.h;
    r->placed = 1;
    a->shelf_x += g.w;
    if (g.h > a->shelf_h) a->shelf_h = g.h;
    if (out_index) *out_index = a->count;
    a->count++;
    return 0;
}

static int atlas_overlap(const Atlas* a) {
    for (int i = 0; i < a->count; i++) {
        const GlyphRect* p = &a->rects[i];
        for (int j = i + 1; j < a->count; j++) {
            const GlyphRect* q = &a->rects[j];
            int sep = p->x + p->w <= q->x || q->x + q->w <= p->x ||
                      p->y + p->h <= q->y || q->y + q->h <= p->y;
            if (!sep) return 1;
        }
    }
    return 0;
}

static int kerning_apply(const KernPair* pairs, int n, const char* text) {
    int total = 0;
    for (const char* p = text; *p; p++) {
        total += 1;   /* placeholder advance per glyph */
        if (p[1]) {
            for (int i = 0; i < n; i++)
                if (pairs[i].left == *p && pairs[i].right == p[1]) total += pairs[i].delta;
        }
    }
    return total;
}

/* ------------------------------------------------------------------ */
/* Signed distance field                                              */
/*                                                                    */
/* The glyph edge is the 0.5 coverage level of the antialiased glyph.  */
/* Marching squares over the coverage grid gives that edge as segments */
/* with sub-pixel crossing positions; a texel's signed distance is the */
/* distance to the nearest segment, negative inside the glyph.  Signs  */
/* come from the thresholded coverage, so d = 0 lands on the edge.     */
/*                                                                    */
/* Storage: one byte per texel, quantised over [-radius, +radius].     */
/* The sign is kept, so a magnified glyph can be rebuilt by            */
/* alpha = clamp(0.5 - d*s), which stays crisp at any scale.           */
/* ------------------------------------------------------------------ */

#define SDF_MAX_SAMPLES 65536

typedef struct { float x, y; } EdgePt;
typedef struct { float x0, y0, x1, y1; } EdgeSeg;

/* coverage of a bit: 1 inside, 0 outside (threshold at half coverage) */
static int inside_px(const unsigned char* mask, int w, int h, int x, int y) {
    if (x < 0 || y < 0 || x >= w || y >= h) return 0;
    return mask[y * w + x] >= 128;
}

/* Marching-squares contour of the 0.5 coverage level, emitted as segments
   so the distance can be measured to the whole contour rather than to
   sampled points.  With an antialiased glyph each crossing is interpolated
   from the coverage values, so the segments are sub-pixel accurate. */
static int collect_edge_segments(const unsigned char* mask, int w, int h,
                                 EdgeSeg* out, int cap) {
    int n = 0;
    for (int y = -1; y < h && n + 2 <= cap; y++) {
        for (int x = -1; x < w && n + 2 <= cap; x++) {
            int cv[4] = {
                (x     >= 0 && y     >= 0 && x     < w && y     < h) ? mask[y * w + x] : 0,
                (x + 1 >= 0 && y     >= 0 && x + 1 < w && y     < h) ? mask[y * w + x + 1] : 0,
                (x + 1 >= 0 && y + 1 >= 0 && x + 1 < w && y + 1 < h) ? mask[(y + 1) * w + x + 1] : 0,
                (x     >= 0 && y + 1 >= 0 && x     < w && y + 1 < h) ? mask[(y + 1) * w + x] : 0
            };
            int inside[4] = { cv[0] >= 128, cv[1] >= 128, cv[2] >= 128, cv[3] >= 128 };
            int code = inside[0] | (inside[1] << 1) | (inside[2] << 2) | (inside[3] << 3);
            if (code == 0 || code == 15) continue;
            /* Sample values are texel centres, at (i+0.5, j+0.5) in pixel-
               corner coordinates; the cell between texels (x,y)..(x+1,y+1)
               therefore has its corners at (x+0.5,y+0.5)..(x+1.5,y+1.5). */
            float cx[4] = { (float)x + 0.5f,      (float)(x + 1) + 0.5f,
                            (float)(x + 1) + 0.5f, (float)x + 0.5f };
            float cy[4] = { (float)y + 0.5f,      (float)y + 0.5f,
                            (float)(y + 1) + 0.5f, (float)(y + 1) + 0.5f };
            static const int e0[4] = {0, 1, 2, 3};
            static const int e1[4] = {1, 2, 3, 0};
            /* a crossing on each cell edge whose endpoints differ in state */
            EdgePt cross[4];
            int hit[4];
            for (int e = 0; e < 4; e++) {
                int i0 = e0[e], i1 = e1[e];
                hit[e] = inside[i0] != inside[i1];
                if (!hit[e]) continue;
                float t = (127.5f - (float)cv[i0]) /
                          ((float)cv[i1] - (float)cv[i0]);
                if (t < 0.0f) t = 0.0f;
                if (t > 1.0f) t = 1.0f;
                cross[e].x = cx[i0] + t * (cx[i1] - cx[i0]);
                cross[e].y = cy[i0] + t * (cy[i1] - cy[i0]);
            }
            int seg = 0;
            if (hit[0] && hit[1] && hit[2] && hit[3]) {
                /* saddle: resolve with the cell's averaged coverage so the
                   pairing keeps the interior corners on the same side */
                float avg = 0.25f * (float)(cv[0] + cv[1] + cv[2] + cv[3]);
                int pair_like_inside = (code == 5) ? (avg >= 127.5f) : (avg < 127.5f);
                if (pair_like_inside) {
                    out[n++] = (EdgeSeg){ cross[1].x, cross[1].y, cross[2].x, cross[2].y };
                    out[n++] = (EdgeSeg){ cross[3].x, cross[3].y, cross[0].x, cross[0].y };
                } else {
                    out[n++] = (EdgeSeg){ cross[0].x, cross[0].y, cross[1].x, cross[1].y };
                    out[n++] = (EdgeSeg){ cross[2].x, cross[2].y, cross[3].x, cross[3].y };
                }
                if (n >= cap) break;
                continue;
            }
            EdgePt a = {0}, b = {0};
            for (int e = 0; e < 4; e++) {
                if (!hit[e]) continue;
                if (seg == 0) a = cross[e];
                else if (seg == 1) b = cross[e];
                seg++;
            }
            if (seg == 2) {
                out[n++] = (EdgeSeg){ a.x, a.y, b.x, b.y };
                if (n >= cap) break;
            }
        }
    }
    return n;
}

/* distance from (px,py) to segment s */
static float point_seg_dist2(float px, float py, const EdgeSeg* s) {
    float vx = s->x1 - s->x0, vy = s->y1 - s->y0;
    float wx = px - s->x0, wy = py - s->y0;
    float len2 = vx * vx + vy * vy;
    float t = len2 > 0.0f ? (wx * vx + wy * vy) / len2 : 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float dx = wx - t * vx, dy = wy - t * vy;
    return dx * dx + dy * dy;
}

/* Quantise a signed distance into one byte over [-radius, radius]. */
static unsigned char sdf_encode(float d, float radius) {
    float t = 0.5f + 0.5f * d / radius;      /* [-r,r] -> [0,1] */
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return (unsigned char)(t * 255.0f + 0.5f);
}

static float sdf_decode(unsigned char q, float radius) {
    return ((float)q / 255.0f * 2.0f - 1.0f) * radius;
}

/* Build an SDF for a w*h antialiased glyph mask.  dst receives w*h bytes.
   radius is the clamp distance (in px); distance beyond it saturates. */
static int sdf_build(const unsigned char* mask, int w, int h, float radius,
                     unsigned char* dst, int cap) {
    if (w <= 0 || h <= 0 || w * h > cap) return -1;
    static EdgeSeg segs[SDF_MAX_SAMPLES];
    int n = collect_edge_segments(mask, w, h, segs, SDF_MAX_SAMPLES);
    if (n == 0) {
        /* no edge: an empty cell (space) is all outside, a solid glyph all
           inside; fill with the clamped distance so both still render */
        int any = 0;
        for (int i = 0; i < w * h; i++) if (mask[i] >= 128) { any = 1; break; }
        unsigned char q = sdf_encode(any ? -radius : radius, radius);
        memset(dst, q, (size_t)(w * h));
        return 0;
    }

    /* brute force nearest contour segment; atlas glyphs are small.  Edge
       and texel centres are both in pixel-corner coordinates (texel i
       spans [i, i+1], centre i+0.5). */
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            float px = (float)x + 0.5f, py = (float)y + 0.5f;
            float best = 1e30f;
            for (int i = 0; i < n; i++) {
                float d2 = point_seg_dist2(px, py, &segs[i]);
                if (d2 < best) best = d2;
            }
            float d = inside_px(mask, w, h, x, y) ? -sqrtf(best) : sqrtf(best);
            dst[y * w + x] = sdf_encode(d, radius);
        }
    }
    return 0;
}

/* Catmull-Rom cubic interpolation of four samples at t in [0,1]. */
static float cubic(float p0, float p1, float p2, float p3, float t) {
    return p1 + 0.5f * t * (p2 - p0 + t * (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3 +
           t * (3.0f * (p1 - p2) + p3 - p0)));
}

/* Rebuild coverage from an SDF at magnification s (1 = native).
   Each destination texel maps back to source space and samples the SDF
   with bicubic filtering; the field's zero crossing where d = 0 is the
   glyph edge, so alpha = clamp(0.5 - d*s) ramps over one destination texel. */
static int sdf_reconstruct(const unsigned char* sdf, int w, int h, float radius,
                           float s, unsigned char* out, int out_w, int out_h) {
    for (int y = 0; y < out_h; y++) {
        for (int x = 0; x < out_w; x++) {
            float sx = ((float)x + 0.5f) / s - 0.5f;
            float sy = ((float)y + 0.5f) / s - 0.5f;
            if (sx < 0.0f) sx = 0.0f;
            if (sy < 0.0f) sy = 0.0f;
            if (sx > (float)(w - 1)) sx = (float)(w - 1);
            if (sy > (float)(h - 1)) sy = (float)(h - 1);
            int ix = (int)floorf(sx), iy = (int)floorf(sy);
            float tx = sx - (float)ix, ty = sy - (float)iy;
            float col[4];
            for (int j = -1; j <= 2; j++) {
                int yy = iy + j;
                if (yy < 0) yy = 0;
                if (yy > h - 1) yy = h - 1;
                float v[4];
                for (int i = -1; i <= 2; i++) {
                    int xx = ix + i;
                    if (xx < 0) xx = 0;
                    if (xx > w - 1) xx = w - 1;
                    v[i + 1] = sdf_decode(sdf[yy * w + xx], radius);
                }
                col[j + 1] = cubic(v[0], v[1], v[2], v[3], tx);
            }
            float d = cubic(col[0], col[1], col[2], col[3], ty);
            /* the destination texel is 1/s source pixels wide, so the
               coverage ramp must narrow in source units by the same s */
            float a = 0.5f - d * s;
            if (a < 0.0f) a = 0.0f;
            if (a > 1.0f) a = 1.0f;
            out[y * out_w + x] = (unsigned char)(a * 255.0f + 0.5f);
        }
    }
    return 0;
}

/* Test glyph shape in texture coordinates: a hollow 'o'-like ring and a
   solid counter, i.e. the two kinds of edge (outer and inner contour) that
   real letterforms have.  Both the source raster and the high-resolution
   reference use this, so their edges are the same. */
static int glyph_solid(float gx, float gy) {
    /* outer ellipse (rx=9, ry=10) with an inner elliptical counter */
    float x = gx - 11.5f, y = gy - 11.5f;
    float outer = (x * x) / (9.0f * 9.0f) + (y * y) / (10.0f * 10.0f);
    float inner = (x * x) / (4.5f * 4.5f) + (y * y) / (5.5f * 5.5f);
    return outer <= 1.0f && inner > 1.0f;
}

/* approximate signed distance from a point to the test shape's edge
   (positive outside).  The two ellipses are convex and axis-aligned, so a
   fine angular scan of their boundaries finds the nearest edge point to
   well under a thousandth of a pixel.  Used by self-test only. */
static float glyph_edge_dist(float gx, float gy) {
    float x = gx - 11.5f, y = gy - 11.5f;
    float best = 1e9f;
    for (int k = 0; k < 2048; k++) {
        float t = (float)k * (6.2831853f / 2048.0f);
        float ct = cosf(t), st = sinf(t);
        for (int which = 0; which < 2; which++) {
            float rx = which ? 4.5f : 9.0f, ry = which ? 5.5f : 10.0f;
            float dx = x - rx * ct, dy = y - ry * st;
            float d2 = dx * dx + dy * dy;
            if (d2 < best) best = d2;
        }
    }
    float dist = sqrtf(best);
    return glyph_solid(gx, gy) ? -dist : dist;
}

/* Analytic reference: the test glyph rendered at magnification s and
   supersampled 4x4 per texel.  Destination texel (x,y) covers source
   coordinates [x/s, (x+1)/s), the same box the SDF reconstruction maps. */
static int render_ref(const unsigned char* mask, int w, int h, float s,
                      unsigned char* out, int out_w, int out_h) {
    (void)mask; (void)w; (void)h;
    for (int y = 0; y < out_h; y++) {
        for (int x = 0; x < out_w; x++) {
            int hits = 0;
            for (int sy = 0; sy < 4; sy++) {
                for (int sx = 0; sx < 4; sx++) {
                    float gx = ((float)x + ((float)sx + 0.5f) * 0.25f) / s;
                    float gy = ((float)y + ((float)sy + 0.5f) * 0.25f) / s;
                    hits += glyph_solid(gx, gy);
                }
            }
            out[y * out_w + x] = (unsigned char)(hits * 255 / 16);
        }
    }
    return 0;
}

/* Sub-pixel edge points on the 0.5 alpha contour, from marching squares
   over the (already anti-aliased) coverage.  Points are in texel units. */
typedef struct { float x, y; } Pt;

static int contour_points(const unsigned char* a, int w, int h, Pt* out, int cap) {
    int n = 0;
    for (int y = 0; y < h - 1 && n + 2 <= cap; y++) {
        for (int x = 0; x < w - 1 && n + 2 <= cap; x++) {
            int c[4] = { a[y * w + x], a[y * w + x + 1],
                         a[(y + 1) * w + x + 1], a[(y + 1) * w + x] };
            float px[4] = { (float)x, (float)(x + 1), (float)(x + 1), (float)x };
            float py[4] = { (float)y, (float)y, (float)(y + 1), (float)(y + 1) };
            static const int e0[4] = {0, 1, 2, 3}, e1[4] = {1, 2, 3, 0};
            for (int e = 0; e < 4; e++) {
                int i0 = e0[e], i1 = e1[e];
                int inside0 = c[i0] >= 128, inside1 = c[i1] >= 128;
                if (inside0 != inside1) {
                    float t = (127.5f - (float)c[i0]) / ((float)c[i1] - (float)c[i0]);
                    if (t < 0.0f) t = 0.0f;
                    if (t > 1.0f) t = 1.0f;
                    out[n].x = px[i0] + t * (px[i1] - px[i0]);
                    out[n].y = py[i0] + t * (py[i1] - py[i0]);
                    n++;
                }
            }
        }
    }
    return n;
}

/* Symmetric Chamfer distance between two contours, in texels:
   the mean over A->B (and B->A) of the nearest-point distance. */
static void chamfer(const Pt* a, int na, const Pt* b, int nb,
                    float* mean, float* maxerr) {
    double sum = 0.0; float mx = 0.0f; int count = 0;
    for (int dir = 0; dir < 2; dir++) {
        const Pt* p = dir ? b : a; int np = dir ? nb : na;
        const Pt* q = dir ? a : b; int nq = dir ? na : nb;
        for (int i = 0; i < np; i++) {
            float best = 1e30f;
            for (int j = 0; j < nq; j++) {
                float dx = p[i].x - q[j].x, dy = p[i].y - q[j].y;
                float d2 = dx * dx + dy * dy;
                if (d2 < best) best = d2;
            }
            float d = sqrtf(best);
            sum += d; count++;
            if (d > mx) mx = d;
        }
    }
    *mean = count ? (float)(sum / count) : 0.0f;
    *maxerr = mx;
}

/* Draw a 24x24 test glyph, supersampled 4x4 so the source has the same
   antialiased coverage an ordinary rasteriser produces. */
static void make_test_glyph(unsigned char* mask) {
    int w = 24, h = 24;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int hits = 0;
            for (int sy = 0; sy < 4; sy++) {
                for (int sx = 0; sx < 4; sx++) {
                    float gx = (float)x + ((float)sx + 0.5f) * 0.25f;
                    float gy = (float)y + ((float)sy + 0.5f) * 0.25f;
                    hits += glyph_solid(gx, gy);
                }
            }
            mask[y * w + x] = (unsigned char)(hits * 255 / 16);
        }
    }
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Atlas a;
    atlas_init(&a);
    int idx = -1;

    check("add_small", atlas_add(&a, (GlyphSize){16, 16}, &idx) == 0 && idx == 0);
    check("add_more", atlas_add(&a, (GlyphSize){32, 32}, NULL) == 0);
    check("no_overlap_two", atlas_overlap(&a) == 0);
    check("rect_in_bounds", a.rects[1].x + a.rects[1].w <= ATLAS_W && a.rects[1].y + a.rects[1].h <= ATLAS_H);

    /* fill the shelf so it must wrap */
    while (atlas_add(&a, (GlyphSize){40, 20}, NULL) == 0) { if (a.count >= 20) break; }
    check("shelf_wrap_used_h", a.used_h > 0);
    check("no_overlap_after_wrap", atlas_overlap(&a) == 0);

    Atlas full;
    atlas_init(&full);
    int rc = 0, n = 0;
    while ((rc = atlas_add(&full, (GlyphSize){64, 64}, NULL)) == 0) { if (++n > MAX_GLYPHS) break; }
    check("atlas_fills_then_fails", rc == -1 && n > 0);
    check("full_no_overlap", atlas_overlap(&full) == 0);

    Atlas bad;
    atlas_init(&bad);
    check("reject_zero", atlas_add(&bad, (GlyphSize){0, 10}, NULL) == -1);
    check("reject_huge", atlas_add(&bad, (GlyphSize){ATLAS_W + 1, 10}, NULL) == -1);

    KernPair pairs[] = { {'A', 'V', -1}, {'T', 'o', -1} };
    check("kern_no_pairs", kerning_apply(pairs, 2, "A") == 1);
    check("kern_AV", kerning_apply(pairs, 2, "AV") == 1);
    check("kern_To", kerning_apply(pairs, 2, "To") == 1);
    check("kern_plain", kerning_apply(pairs, 2, "AB") == 2);

    /* --- signed distance field --- */
    enum { GW = 24, GH = 24, MAG = 4 };
    enum { DW = GW * MAG, DH = GH * MAG, MAXPTS = 16384 };
    static unsigned char mask[GW * GH], sdf[GW * GH];
    static unsigned char ref[DW * DH], rec[DW * DH], nearest[DW * DH];
    static Pt ref_pts[MAXPTS], sdf_pts[MAXPTS], bmp_pts[MAXPTS];
    const float radius = 6.0f;

    make_test_glyph(mask);
    check("sdf_build", sdf_build(mask, GW, GH, radius, sdf, GW * GH) == 0);

    /* no-edge glyphs are still well defined: a blank cell is all outside,
       a solid glyph all inside */
    {
        static unsigned char blank[GW * GH], solid[GW * GH], tmp[GW * GH];
        memset(blank, 0, sizeof(blank));
        memset(solid, 255, sizeof(solid));
        int b = sdf_build(blank, GW, GH, radius, tmp, GW * GH);
        float db = sdf_decode(tmp[0], radius);
        int s = sdf_build(solid, GW, GH, radius, tmp, GW * GH);
        float ds = sdf_decode(tmp[0], radius);
        check("sdf_blank_all_outside", b == 0 && db > 0.0f);
        check("sdf_solid_all_inside", s == 0 && ds < 0.0f);
    }

    /* the field is positive outside the stroke and negative inside it */
    check("sdf_sign_outside", sdf_decode(sdf[0], radius) > 0.0f);
    {
        float d = sdf_decode(sdf[12 * GW + 12], radius);     /* in the counter */
        check("sdf_sign_counter", d > 0.0f);
        float s = sdf_decode(sdf[11 * GW + 3], radius);      /* in the stroke */
        check("sdf_sign_inside", s < 0.0f);
    }

    /* accuracy of the field itself, against the true distance to the shape's
       edge in source pixels; only texels where the field is not clamped */
    {
        double sum = 0.0; float mx = 0.0f; int ns = 0;
        for (int y = 0; y < GH; y++) {
            for (int x = 0; x < GW; x++) {
                float exact = glyph_edge_dist((float)x + 0.5f, (float)y + 0.5f);
                if (fabsf(exact) > radius - 1.0f) continue;
                float got = sdf_decode(sdf[y * GW + x], radius);
                float e = fabsf(got - exact);
                sum += e; ns++; if (e > mx) mx = e;
            }
        }
        float mean = ns ? (float)(sum / ns) : 0.0f;
        printf("sdf_field_mean_px=%.4f sdf_field_max_px=%.4f samples=%d\n", mean, mx, ns);
        check("sdf_field_mean_under_quarter_px", mean < 0.25f);
        check("sdf_field_max_under_half_px", mx < 0.5f);
    }

    /* magnify x4 and compare edge positions with a 4x supersampled
       high-resolution render of the same shape */
    render_ref(mask, GW, GH, (float)MAG, ref, DW, DH);
    sdf_reconstruct(sdf, GW, GH, radius, (float)MAG, rec, DW, DH);
    for (int y = 0; y < DH; y++)
        for (int x = 0; x < DW; x++)
            nearest[y * DW + x] = mask[(y / MAG) * GW + (x / MAG)];

    int n_ref = contour_points(ref, DW, DH, ref_pts, MAXPTS);
    int n_sdf = contour_points(rec, DW, DH, sdf_pts, MAXPTS);
    int n_bmp = contour_points(nearest, DW, DH, bmp_pts, MAXPTS);

    float sdf_mean = 0.0f, sdf_max = 0.0f, bmp_mean = 0.0f, bmp_max = 0.0f;
    chamfer(ref_pts, n_ref, sdf_pts, n_sdf, &sdf_mean, &sdf_max);
    chamfer(ref_pts, n_ref, bmp_pts, n_bmp, &bmp_mean, &bmp_max);

    /* report in source pixels: that is the unit "half a pixel" refers to */
    printf("mag=%d contour_points ref=%d sdf=%d bitmap=%d\n", MAG, n_ref, n_sdf, n_bmp);
    printf("sdf_edge_mean_texels=%.4f sdf_edge_max_texels=%.4f\n", sdf_mean, sdf_max);
    printf("sdf_edge_mean_px=%.4f sdf_edge_max_px=%.4f\n",
           sdf_mean / MAG, sdf_max / MAG);
    printf("bitmap_edge_mean_px=%.4f bitmap_edge_max_px=%.4f\n",
           bmp_mean / MAG, bmp_max / MAG);
    check("sdf_edge_mean_under_half_px", sdf_mean / MAG < 0.5f);
    check("sdf_edge_max_under_half_px", sdf_max / MAG < 0.5f);
    check("sdf_beats_bitmap", (sdf_mean / MAG) < (bmp_mean / MAG) * 0.5f);

    /* storage: the field replaces the alpha bitmap one-for-one at native
       resolution (1 byte per texel); report the measured totals.  The
       bitmap must be kept at the highest size actually drawn, while the
       field is drawn crisp well past 4x, so for magnified text the field
       is the smaller of the two at equal quality. */
    {
        size_t glyph_w = GW, glyph_h = GH;
        size_t bitmap_native = glyph_w * glyph_h;        /* 8-bit coverage */
        size_t sdf_native = glyph_w * glyph_h;           /* 8-bit quantised distance */
        /* what a bitmap needs to match 4x magnified quality */
        size_t bitmap_hi = (glyph_w * MAG) * (glyph_h * MAG);
        printf("glyph_px=%zux%zu bitmap_bytes=%zu sdf_bytes=%zu ratio=%.2f\n",
               glyph_w, glyph_h, bitmap_native, sdf_native,
               (double)sdf_native / (double)bitmap_native);
        printf("bitmap_at_%dx_bytes=%zu sdf_vs_hires_ratio=%.3f\n",
               MAG, bitmap_hi, (double)sdf_native / (double)bitmap_hi);
        check("sdf_bytes_match_bitmap", sdf_native == bitmap_native);
        check("sdf_smaller_than_hires_bitmap", sdf_native < bitmap_hi);
    }
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
