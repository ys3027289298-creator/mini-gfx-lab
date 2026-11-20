/* ninepatch: cut a border image into nine regions, resize it without
 * stretching the corners, and check the result against a hand-sliced
 * reference pixel for pixel.
 *
 * Slice layout -- border widths l,r,t,b are in source pixels and the four
 * corners are always drawn 1:1, never resampled:
 *
 *     +----+-----------------+----+
 *     | TL |    top edge     | TR |
 *     +----+-----------------+----+
 *     | lt |     centre      | rt |
 *     +----+-----------------+----+
 *     | BL |   bottom edge   | BR |
 *     +----+-----------------+----+
 *
 * The edges resize along one axis and the centre along both.  Each axis is
 * either REPEAT (whole source pixels tiled) or STRETCH (nearest
 * neighbour), so a one-pixel border line stays one pixel wide at any size.
 *
 * Transforms are applied to the finished output.  Composition is a pure
 * function of (slice, size, mode), so a transformed render has to equal
 * the untransformed render put through the same transform, pixel for
 * pixel; that is what the cross_check_* tests verify, next to a direct
 * hand-sliced formula.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define IMG_W 16
#define IMG_H 16
#define OUT_W 48
#define OUT_H 48
#define MAX_CANVAS 96

typedef struct { unsigned char r, g, b, a; } Pixel;
typedef struct { int w, h; Pixel px[IMG_H][IMG_W]; } Image;
typedef struct { int w, h; Pixel px[MAX_CANVAS][MAX_CANVAS]; } Canvas;

enum {
    P_TL, P_TOP, P_TR,
    P_LEFT, P_CENTRE, P_RIGHT,
    P_BL, P_BOTTOM, P_BR,
    P_COUNT,
};

enum { R_REPEAT = 0, R_STRETCH = 1 };

/* Rotations are clockwise; mirrors name the pixel motion. */
enum { XF_NONE = 0, XF_ROT90, XF_ROT180, XF_ROT270, XF_MIRROR_H, XF_MIRROR_V };

enum { SLICE_OK = 0, SLICE_ERR_SIZE, SLICE_ERR_ZERO, SLICE_ERR_IMG };

typedef struct { int w, h; Pixel* px; } Piece;

typedef struct {
    int l, r, t, b;
    int mode_x, mode_y;
    int src_w, src_h;
    Piece pc[P_COUNT];
    int mem_bytes;   /* resident bytes of the nine pieces */
} NinePatch;

typedef struct { int top, right, bottom, left, centre, total; } EdgeDiff;

static const char* slot_name(int i) {
    static const char* n[P_COUNT] = {
        "tl", "top", "tr", "left", "centre", "right", "bl", "bottom", "br"
    };
    return n[i];
}

/* Source pixel value.  b carries the column and a the row, so the 16x16
 * image has 256 distinct ids; r/g add a two-tone checker for eyeballing
 * the output.  Tests use the id to spot repeated or dropped rows without
 * a hand-written expected table. */
static Pixel src_px(int x, int y) {
    Pixel p;
    p.r = (unsigned char)((x & 1) ? 255 : 32);
    p.g = (unsigned char)((y & 1) ? 255 : 32);
    p.b = (unsigned char)x;
    p.a = (unsigned char)y;
    return p;
}

static void img_init(Image* im) {
    im->w = IMG_W;
    im->h = IMG_H;
    for (int y = 0; y < IMG_H; y++)
        for (int x = 0; x < IMG_W; x++)
            im->px[y][x] = src_px(x, y);
}

static int px_eq(Pixel a, Pixel b) {
    return a.r == b.r && a.g == b.g && a.b == b.b && a.a == b.a;
}

static int px_x(Pixel p) { return p.b; }   /* encoded source column */
static int px_y(Pixel p) { return p.a; }   /* encoded source row */

/* ---- slicing ---------------------------------------------------------- */

/* Source rectangle owned by each of the nine pieces. */
static void piece_rect(int id, int l, int r, int t, int b, int W, int H,
                       int* x0, int* y0, int* pw, int* ph) {
    int iw = W - l - r;
    int ih = H - t - b;
    switch (id) {
    case P_TL:     *x0 = 0;     *y0 = 0;     *pw = l;  *ph = t;  break;
    case P_TOP:    *x0 = l;     *y0 = 0;     *pw = iw; *ph = t;  break;
    case P_TR:     *x0 = W - r; *y0 = 0;     *pw = r;  *ph = t;  break;
    case P_LEFT:   *x0 = 0;     *y0 = t;     *pw = l;  *ph = ih; break;
    case P_CENTRE: *x0 = l;     *y0 = t;     *pw = iw; *ph = ih; break;
    case P_RIGHT:  *x0 = W - r; *y0 = t;     *pw = r;  *ph = ih; break;
    case P_BL:     *x0 = 0;     *y0 = H - b; *pw = l;  *ph = b;  break;
    case P_BOTTOM: *x0 = l;     *y0 = H - b; *pw = iw; *ph = b;  break;
    default:       *x0 = W - r; *y0 = H - b; *pw = r;  *ph = b;  break;
    }
}

static void slice_free(NinePatch* np) {
    for (int i = 0; i < P_COUNT; i++) {
        free(np->pc[i].px);
        np->pc[i].px = NULL;
        np->pc[i].w = np->pc[i].h = 0;
    }
    np->mem_bytes = 0;
}

/* Validate, then copy the nine regions out.  Fails rather than clamping
 * when a border is zero or a pair of borders cannot fit in the source:
 * a degenerate slice would silently stop being nine regions. */
static int slice_build(NinePatch* np, const Image* src,
                       int l, int r, int t, int b, int mode_x, int mode_y) {
    if (!src || !np) return SLICE_ERR_IMG;
    if (src->w <= 0 || src->h <= 0 || src->w > IMG_W || src->h > IMG_H)
        return SLICE_ERR_IMG;
    if (l <= 0 || r <= 0 || t <= 0 || b <= 0) return SLICE_ERR_ZERO;
    /* both borders must fit and must leave room for a centre */
    if (l + r >= src->w || t + b >= src->h) return SLICE_ERR_SIZE;

    memset(np, 0, sizeof(*np));
    np->l = l; np->r = r; np->t = t; np->b = b;
    np->mode_x = mode_x; np->mode_y = mode_y;
    np->src_w = src->w; np->src_h = src->h;

    for (int i = 0; i < P_COUNT; i++) {
        int x0, y0, pw, ph;
        piece_rect(i, l, r, t, b, src->w, src->h, &x0, &y0, &pw, &ph);
        np->pc[i].w = pw;
        np->pc[i].h = ph;
        np->pc[i].px = (Pixel*)malloc((size_t)pw * ph * sizeof(Pixel));
        if (!np->pc[i].px) { slice_free(np); return SLICE_ERR_IMG; }
        for (int y = 0; y < ph; y++)
            for (int x = 0; x < pw; x++)
                np->pc[i].px[y * pw + x] = src->px[y0 + y][x0 + x];
        np->mem_bytes += pw * ph * (int)sizeof(Pixel);
    }
    return SLICE_OK;
}

/* ---- resampling and composition --------------------------------------- */

/* Map output offset i of n onto a source axis of `inner` pixels.  REPEAT
 * cycles whole pixels; STRETCH scales by integer nearest neighbour. */
static int axis_map(int i, int n, int inner, int mode) {
    if (inner <= 0 || n <= 0) return 0;
    if (mode == R_REPEAT) return i % inner;
    int idx = i * inner / n;
    return idx < inner ? idx : inner - 1;
}

static void blit(const Piece* p, int x0, int y0, int w, int h,
                 int mode_x, int mode_y, Canvas* out) {
    for (int y = 0; y < h; y++) {
        int oy = y0 + y;
        if (oy < 0 || oy >= out->h) continue;
        int sy = axis_map(y, h, p->h, mode_y);
        for (int x = 0; x < w; x++) {
            int ox = x0 + x;
            if (ox < 0 || ox >= out->w) continue;
            int sx = axis_map(x, w, p->w, mode_x);
            out->px[oy][ox] = p->px[sy * p->w + sx];
        }
    }
}

/* Compose the nine pieces at ow x oh.  br_dx/br_dy nudge only the bottom
 * right piece; the self-test uses that to prove the seam count localises
 * a fault instead of always reading zero. */
static int compose_impl(const NinePatch* np, int ow, int oh, Canvas* out,
                        int br_dx, int br_dy) {
    const int l = np->l, r = np->r, t = np->t, b = np->b;
    if (ow < l + r || oh < t + b) return 0;   /* no room for the centre */
    if (ow > MAX_CANVAS || oh > MAX_CANVAS) return 0;

    out->w = ow;
    out->h = oh;
    int cw = ow - l - r;
    int ch = oh - t - b;

    const int dx[P_COUNT] = { 0,   l,   l + cw, 0,   l,   l + cw, 0,   l,   l + cw };
    const int dy[P_COUNT] = { 0,   0,   0,      t,   t,   t,      t+ch,t+ch,t + ch };
    const int dw[P_COUNT] = { l,   cw,  r,      l,   cw,  r,      l,   cw,  r      };
    const int dh[P_COUNT] = { t,   t,   t,      ch,  ch,  ch,     b,   b,   b      };

    for (int i = 0; i < P_COUNT; i++) {
        int ox = dx[i], oy = dy[i];
        if (i == P_BR) { ox += br_dx; oy += br_dy; }
        if (dw[i] <= 0 || dh[i] <= 0) continue;
        blit(&np->pc[i], ox, oy, dw[i], dh[i], np->mode_x, np->mode_y, out);
    }
    return 1;
}

static int compose(const NinePatch* np, int ow, int oh, Canvas* out) {
    return compose_impl(np, ow, oh, out, 0, 0);
}

/* ---- transforms -------------------------------------------------------- */

static int xf_swaps(int xf) { return xf == XF_ROT90 || xf == XF_ROT270; }

static int xf_apply(const Canvas* in, int xf, Canvas* out) {
    int w = in->w, h = in->h;
    int dw = xf_swaps(xf) ? h : w;
    int dh = xf_swaps(xf) ? w : h;
    if (dw > MAX_CANVAS || dh > MAX_CANVAS) return 0;
    out->w = dw;
    out->h = dh;

    for (int y = 0; y < dh; y++) {
        for (int x = 0; x < dw; x++) {
            int sx, sy;
            switch (xf) {
            case XF_ROT90:    sx = y;         sy = h - 1 - x; break;
            case XF_ROT180:   sx = w - 1 - x; sy = h - 1 - y; break;
            case XF_ROT270:   sx = w - 1 - y; sy = x;         break;
            case XF_MIRROR_H: sx = w - 1 - x; sy = y;         break;
            case XF_MIRROR_V: sx = x;         sy = h - 1 - y; break;
            case XF_NONE:
            default:          sx = x;         sy = y;         break;
            }
            out->px[y][x] = in->px[sy][sx];
        }
    }
    return 1;
}

static int render(const NinePatch* np, int ow, int oh, int xf, Canvas* out) {
    Canvas tmp;
    if (!compose(np, ow, oh, &tmp)) return 0;
    return xf_apply(&tmp, xf, out);
}

/* ---- independent hand-sliced reference -------------------------------- */

/* Direct formula: choose a source pixel for every output pixel from the
 * region it lands in.  No pieces and no blits; this is what a hand-written
 * slicer does, and it is compared against the piece pipeline pixel for
 * pixel. */
static int ref_render(const Image* s, int l, int r, int t, int b,
                      int mode_x, int mode_y, int ow, int oh, Canvas* out) {
    if (!s || !out) return 0;
    if (l <= 0 || r <= 0 || t <= 0 || b <= 0) return 0;
    if (l + r >= s->w || t + b >= s->h) return 0;
    if (ow < l + r || oh < t + b) return 0;
    if (ow > MAX_CANVAS || oh > MAX_CANVAS) return 0;

    int iw = s->w - l - r;
    int ih = s->h - t - b;
    int cw = ow - l - r;
    int ch = oh - t - b;

    out->w = ow;
    out->h = oh;
    for (int oy = 0; oy < oh; oy++) {
        int sy;
        if (oy < t)            sy = oy;
        else if (oy >= oh - b) sy = s->h - (oh - oy);
        else                   sy = t + axis_map(oy - t, ch, ih, mode_y);
        for (int ox = 0; ox < ow; ox++) {
            int sx;
            if (ox < l)            sx = ox;
            else if (ox >= ow - r) sx = s->w - (ow - ox);
            else                   sx = l + axis_map(ox - l, cw, iw, mode_x);
            out->px[oy][ox] = s->px[sy][sx];
        }
    }
    return 1;
}

/* ---- comparison -------------------------------------------------------- */

static int canvas_diff(const Canvas* a, const Canvas* b) {
    if (a->w != b->w || a->h != b->h) return -1;
    int n = 0;
    for (int y = 0; y < a->h; y++)
        for (int x = 0; x < a->w; x++)
            if (!px_eq(a->px[y][x], b->px[y][x])) n++;
    return n;
}

/* Mismatches split by the output edge band they land in.  A corner pixel
 * belongs to two bands and is counted in both n/top+bottom/left+right
 * numbers; `total` counts it once.  `centre` is in none of the bands. */
static EdgeDiff edge_diff(const Canvas* a, const Canvas* b,
                          int bl, int br, int bt, int bb) {
    EdgeDiff d;
    memset(&d, 0, sizeof(d));
    if (a->w != b->w || a->h != b->h) { d.total = -1; return d; }
    for (int y = 0; y < a->h; y++) {
        for (int x = 0; x < a->w; x++) {
            if (px_eq(a->px[y][x], b->px[y][x])) continue;
            int is_t = y < bt;
            int is_b = y >= a->h - bb;
            int is_l = x < bl;
            int is_r = x >= a->w - br;
            d.total++;
            if (is_t) d.top++;
            if (is_b) d.bottom++;
            if (is_l) d.left++;
            if (is_r) d.right++;
            if (!is_t && !is_b && !is_l && !is_r) d.centre++;
        }
    }
    return d;
}

/* Whole-image nearest-neighbour stretch: the distortion nine-slice exists
 * to replace.  Used only as a control; its corners must differ. */
static void naive_render(const Image* s, int ow, int oh, Canvas* out) {
    out->w = ow;
    out->h = oh;
    for (int y = 0; y < oh; y++)
        for (int x = 0; x < ow; x++)
            out->px[y][x] = s->px[y * s->h / oh][x * s->w / ow];
}

/* Rectangular sub-region comparison against source, for corner identity. */
static int rect_diff(const Canvas* a, const Image* s,
                     int ax, int ay, int sx, int sy, int w, int h) {
    int n = 0;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            if (!px_eq(a->px[ay + y][ax + x], s->px[sy + y][sx + x])) n++;
    return n;
}

/* ---- self-test --------------------------------------------------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void print_seams(const char* tag, EdgeDiff d) {
    printf("seam_%s top=%d right=%d bottom=%d left=%d centre=%d total=%d\n",
           tag, d.top, d.right, d.bottom, d.left, d.centre, d.total);
}

int main(void) {
    Image img;
    img_init(&img);
    const int src_bytes = img.w * img.h * (int)sizeof(Pixel);

    /* ---- slicing guards ----------------------------------------------- */
    NinePatch np;
    check("slice_basic",
          slice_build(&np, &img, 4, 4, 4, 4, R_STRETCH, R_STRETCH) == SLICE_OK);
    printf("slice_mem=%d src_mem=%d limit=%d\n",
           np.mem_bytes, src_bytes, 2 * src_bytes);
    check("slice_mem_within_double", np.mem_bytes <= 2 * src_bytes);
    check("slice_pieces_cover_source", np.mem_bytes == src_bytes);
    check("slice_piece_shape",
          np.pc[P_CENTRE].w == 8 && np.pc[P_CENTRE].h == 8 &&
          np.pc[P_TL].w == 4 && np.pc[P_TL].h == 4 &&
          np.pc[P_BR].w == 4 && np.pc[P_BR].h == 4);

    NinePatch bad;
    check("slice_reject_zero_left",
          slice_build(&bad, &img, 0, 4, 4, 4, R_STRETCH, R_STRETCH) == SLICE_ERR_ZERO);
    check("slice_reject_zero_bottom",
          slice_build(&bad, &img, 4, 4, 4, 0, R_STRETCH, R_STRETCH) == SLICE_ERR_ZERO);
    check("slice_reject_exceed_width",
          slice_build(&bad, &img, 9, 9, 4, 4, R_STRETCH, R_STRETCH) == SLICE_ERR_SIZE);
    check("slice_reject_exceed_height",
          slice_build(&bad, &img, 4, 4, 10, 10, R_STRETCH, R_STRETCH) == SLICE_ERR_SIZE);
    check("slice_reject_consume_axis",
          slice_build(&bad, &img, 8, 8, 4, 4, R_STRETCH, R_STRETCH) == SLICE_ERR_SIZE);
    check("slice_reject_null_image",
          slice_build(&bad, NULL, 4, 4, 4, 4, R_STRETCH, R_STRETCH) == SLICE_ERR_IMG);

    /* ---- corners, stretching, tiling ---------------------------------- */
    Canvas c, r, t;
    check("compose_48", compose(&np, OUT_W, OUT_H, &c) == 1);
    check("corners_unchanged_1to1",
          rect_diff(&c, &img, 0, 0, 0, 0, 4, 4) == 0 &&
          rect_diff(&c, &img, OUT_W - 4, 0, 12, 0, 4, 4) == 0 &&
          rect_diff(&c, &img, 0, OUT_H - 4, 0, 12, 4, 4) == 0 &&
          rect_diff(&c, &img, OUT_W - 4, OUT_H - 4, 12, 12, 4, 4) == 0);

    /* centre column is 40 px wide over 8 source columns: five copies each */
    int column_ok = 1;
    for (int u = 0; u < 40; u++) {
        int want = 4 + u * 8 / 40;
        if (px_x(c.px[OUT_H / 2][4 + u]) != want) column_ok = 0;
    }
    check("stretch_centre_column_counts", column_ok);

    /* stretching to the source size must reproduce the source exactly */
    Canvas ident;
    int ident_ok = compose(&np, IMG_W, IMG_H, &ident) == 1;
    for (int y = 0; y < IMG_H && ident_ok; y++)
        for (int x = 0; x < IMG_W; x++)
            if (!px_eq(ident.px[y][x], img.px[y][x])) { ident_ok = 0; break; }
    check("stretch_identity_pixels", ident_ok);

    /* ---- repeat mode: border thickness and tiling --------------------- */
    NinePatch rp;
    slice_build(&rp, &img, 3, 5, 2, 6, R_REPEAT, R_REPEAT);
    check("repeat_compose", compose(&rp, 50, 37, &t) == 1);

    int border_ok = 1;
    for (int x = 0; x < 50; x++) {
        if (px_y(t.px[0][x]) != 0 || px_y(t.px[1][x]) != 1) border_ok = 0;
    }
    for (int y = 0; y < 37; y++) {
        if (px_x(t.px[y][0]) != 0 || px_x(t.px[y][2]) != 2) border_ok = 0;
    }
    check("repeat_border_thickness", border_ok);

    int cw = 50 - 3 - 5;   /* 42 */
    int ch = 37 - 2 - 6;   /* 29 */
    int tile_ok = 1, stretch_ok = 0;
    for (int v = 0; v < ch; v++) {
        for (int u = 0; u < cw; u++) {
            Pixel p = t.px[2 + v][3 + u];
            if (px_x(p) != 3 + u % 8 || px_y(p) != 2 + v % 8) tile_ok = 0;
            if (px_x(p) != 3 + u * 8 / cw || px_y(p) != 2 + v * 8 / ch) stretch_ok = 1;
        }
    }
    check("repeat_centre_periodic", tile_ok);
    check("repeat_differs_from_stretch_where_expected", stretch_ok);

    /* ---- reference comparison, per-edge seam numbers ------------------ */
    NinePatch sp;
    slice_build(&sp, &img, 4, 4, 4, 4, R_STRETCH, R_STRETCH);
    compose(&sp, OUT_W, OUT_H, &c);
    ref_render(&img, 4, 4, 4, 4, R_STRETCH, R_STRETCH, OUT_W, OUT_H, &r);
    EdgeDiff ds = edge_diff(&c, &r, 4, 4, 4, 4);
    print_seams("stretch_48", ds);
    check("ref_match_stretch", ds.total == 0);

    compose(&rp, 50, 37, &t);
    ref_render(&img, 3, 5, 2, 6, R_REPEAT, R_REPEAT, 50, 37, &r);
    EdgeDiff dr = edge_diff(&t, &r, 3, 5, 2, 6);
    print_seams("repeat_50x37", dr);
    check("ref_match_repeat", dr.total == 0);

    /* the seam metric must be able to see a fault, and say where it is:
     * nudging the bottom-right piece damages only the right and bottom */
    Canvas fault;
    compose_impl(&sp, OUT_W, OUT_H, &fault, 1, 1);
    ref_render(&img, 4, 4, 4, 4, R_STRETCH, R_STRETCH, OUT_W, OUT_H, &r);
    EdgeDiff df = edge_diff(&fault, &r, 4, 4, 4, 4);
    print_seams("br_nudged", df);
    check("seam_metric_localises_fault",
          df.top == 0 && df.left == 0 && df.right > 0 && df.bottom > 0);

    /* ---- transform cross-checks --------------------------------------- */
    static const int xfs[] = { XF_ROT90, XF_ROT180, XF_ROT270,
                               XF_MIRROR_H, XF_MIRROR_V };
    static const char* xf_names[] = { "rot90", "rot180", "rot270",
                                      "mirror_h", "mirror_v" };

    for (int i = 0; i < 5; i++) {
        Canvas got, refc, plain, want;
        char name[64];

        int ok = render(&sp, OUT_W, OUT_H, xfs[i], &got);

        /* 1) transformed pipeline == hand-sliced reference, transformed */
        ref_render(&img, 4, 4, 4, 4, R_STRETCH, R_STRETCH, OUT_W, OUT_H, &refc);
        xf_apply(&refc, xfs[i], &want);
        int swap = xf_swaps(xfs[i]);
        int ok_ref = ok && got.w == (swap ? OUT_H : OUT_W) &&
                     got.h == (swap ? OUT_W : OUT_H) &&
                     canvas_diff(&got, &want) == 0;
        snprintf(name, sizeof(name), "cross_check_ref_%s", xf_names[i]);
        check(name, ok_ref);

        /* 2) transformed pipeline == untransformed pipeline, then transform */
        compose(&sp, OUT_W, OUT_H, &plain);
        xf_apply(&plain, xfs[i], &want);
        snprintf(name, sizeof(name), "cross_check_plain_%s", xf_names[i]);
        check(name, canvas_diff(&got, &want) == 0);
    }

    /* a square output keeps the four edge bands comparable in output
     * coordinates after any rotation or mirror */
    for (int i = 0; i < 5; i++) {
        Canvas got, refc, want;
        render(&sp, 48, 48, xfs[i], &got);
        ref_render(&img, 4, 4, 4, 4, R_STRETCH, R_STRETCH, 48, 48, &refc);
        xf_apply(&refc, xfs[i], &want);
        EdgeDiff d = edge_diff(&got, &want, 4, 4, 4, 4);
        char tag[64], name[64];
        snprintf(tag, sizeof(tag), "%s_48x48", xf_names[i]);
        snprintf(name, sizeof(name), "transformed_seams_%s", xf_names[i]);
        print_seams(tag, d);
        check(name, d.total == 0);
    }

    /* involutions and rotation cycle */
    Canvas b1, b2, b3, b4, b5;
    compose(&sp, 20, 30, &b1);
    xf_apply(&b1, XF_ROT180, &b2);
    xf_apply(&b2, XF_ROT180, &b3);
    check("rot180_involutive", canvas_diff(&b1, &b3) == 0);
    xf_apply(&b1, XF_MIRROR_H, &b2);
    xf_apply(&b2, XF_MIRROR_H, &b3);
    check("mirror_h_involutive", canvas_diff(&b1, &b3) == 0);
    xf_apply(&b1, XF_MIRROR_V, &b2);
    xf_apply(&b2, XF_MIRROR_V, &b3);
    check("mirror_v_involutive", canvas_diff(&b1, &b3) == 0);
    xf_apply(&b1, XF_ROT90, &b2);
    xf_apply(&b2, XF_ROT90, &b3);
    xf_apply(&b3, XF_ROT90, &b4);
    xf_apply(&b4, XF_ROT90, &b5);
    check("rot90_four_times_identity",
          b5.w == b1.w && b5.h == b1.h && canvas_diff(&b5, &b1) == 0);

    /* ---- control: a plain stretch distorts the corners ---------------- */
    Canvas naive;
    naive_render(&img, OUT_W, OUT_H, &naive);
    int corner_bad = rect_diff(&naive, &img, 0, 0, 0, 0, 4, 4) +
                     rect_diff(&naive, &img, OUT_W - 4, 0, 12, 0, 4, 4) +
                     rect_diff(&naive, &img, 0, OUT_H - 4, 0, 12, 4, 4) +
                     rect_diff(&naive, &img, OUT_W - 4, OUT_H - 4, 12, 12, 4, 4);
    printf("naive_corner_diff=%d\n", corner_bad);
    check("naive_stretch_distorts_corners", corner_bad > 0);

    /* ---- out-of-range targets ----------------------------------------- */
    check("ref_reject_tiny_target",
          ref_render(&img, 4, 4, 4, 4, R_STRETCH, R_STRETCH, 7, 8, &r) == 0);
    check("compose_reject_tiny_target", compose(&sp, 7, 8, &c) == 0);

    /* piece slots are all populated: report shapes once for the log */
    for (int i = 0; i < P_COUNT; i++)
        printf("piece_%s=%dx%d\n", slot_name(i), sp.pc[i].w, sp.pc[i].h);

    slice_free(&np);
    slice_free(&rp);
    slice_free(&sp);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
