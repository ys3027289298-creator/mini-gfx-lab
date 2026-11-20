
/* palette: theme recolor of an icon atlas.
 *
 * source palette maps slot-by-slot to the destination palette, with optional
 * per-color overrides. Pixels that do not belong to any palette color stay
 * untouched. Anti-aliased edges are rebuilt by coverage: the edge pixel's
 * coverage over its background is recovered, then the replacement color is
 * blended with the same integer rounding the hand-exported reference uses,
 * so a full-atlas recolor compares pixel-identical (zero stray pixels). */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define PAL_MAX 32

typedef struct { unsigned char r, g, b, a; } RGBA;

typedef enum {
    PAL_OK = 0,
    PAL_ERR_EMPTY = -1,        /* a palette is empty or longer than PAL_MAX */
    PAL_ERR_LENGTH = -2,       /* source and destination lengths differ */
    PAL_ERR_DUP_SRC = -3,      /* duplicate color inside the source palette */
    PAL_ERR_DUP_DST = -4,      /* duplicate color inside the destination palette */
    PAL_ERR_OVERRIDE_KEY = -5, /* override key not found in the source palette */
    PAL_ERR_DUP_OVERRIDE = -6, /* same source key overridden twice */
    PAL_ERR_OVERRIDE_DUP = -7  /* override target collides with another destination color */
} PalError;

typedef enum {
    PAL_BG_SOLID = 0,  /* opaque atlas: edges blend against a solid background */
    PAL_BG_STRAIGHT = 1, /* transparent atlas, straight (non-premultiplied) alpha */
    PAL_BG_PREMUL = 2   /* transparent atlas, premultiplied alpha */
} PalBgMode;

typedef struct { RGBA key; RGBA target; } PalOverride;

typedef struct {
    int enabled;
    int count;
    RGBA src[PAL_MAX];
    RGBA dst[PAL_MAX];
    PalBgMode bg;
    RGBA background;
} PalMap;

static int rgba_eq(RGBA x, RGBA y) {
    return x.r == y.r && x.g == y.g && x.b == y.b && x.a == y.a;
}

/* integer lerp with round-half-up: the shared blend convention */
static unsigned char blend_byte(int a, int b, int t) {
    return (unsigned char)((a * (255 - t) + b * t + 127) / 255);
}

static RGBA blend_solid(RGBA bg, RGBA fg, int t) {
    RGBA out;
    out.r = blend_byte(bg.r, fg.r, t);
    out.g = blend_byte(bg.g, fg.g, t);
    out.b = blend_byte(bg.b, fg.b, t);
    out.a = 255;
    return out;
}

/* build a recolor map. overrides may be NULL. destination entries matched by
 * an override still require an entry in src; overriding is per-color, not
 * per-slot, and wins over the slot mapping. */
static int pal_build(PalMap* m,
                     const RGBA* src, int src_len,
                     const RGBA* dst, int dst_len,
                     const PalOverride* ov, int ov_count,
                     PalBgMode bg, RGBA background) {
    memset(m, 0, sizeof(*m));
    if (src_len <= 0 || dst_len <= 0 || src_len > PAL_MAX || dst_len > PAL_MAX)
        return PAL_ERR_EMPTY;
    if (src_len != dst_len) return PAL_ERR_LENGTH;

    for (int i = 0; i < src_len; i++) {
        for (int j = i + 1; j < src_len; j++)
            if (rgba_eq(src[i], src[j])) return PAL_ERR_DUP_SRC;
    }
    for (int i = 0; i < dst_len; i++) {
        for (int j = i + 1; j < dst_len; j++)
            if (rgba_eq(dst[i], dst[j])) return PAL_ERR_DUP_DST;
    }

    m->count = src_len;
    memcpy(m->src, src, src_len * sizeof(RGBA));
    memcpy(m->dst, dst, dst_len * sizeof(RGBA));
    m->bg = bg;
    m->background = background;
    m->enabled = 1;

    for (int k = 0; k < ov_count; k++) {
        int slot = -1;
        for (int i = 0; i < m->count; i++)
            if (rgba_eq(ov[k].key, m->src[i])) { slot = i; break; }
        if (slot < 0) return PAL_ERR_OVERRIDE_KEY;
        if (!rgba_eq(m->dst[slot], dst[slot])) return PAL_ERR_DUP_OVERRIDE;
        for (int i = 0; i < m->count; i++)
            if (i != slot && rgba_eq(ov[k].target, m->dst[i]))
                return PAL_ERR_OVERRIDE_DUP;
        m->dst[slot] = ov[k].target;
    }
    return PAL_OK;
}

/* recover the palette slot and coverage of a solid-background pixel.
 * for each slot, coverage t is read off the widest-separation channel and
 * the other two channels are verified against blend_byte, so only blends
 * exactly explainable by the convention match. returns 1 with exactly one
 * explanation, 0 otherwise (ambiguous or foreign -> pixel stays as-is). */
static int solve_solid(const PalMap* m, RGBA p, int* out_slot, int* out_t) {
    int hits = 0, hit_slot = -1, hit_t = -1;
    for (int s = 0; s < m->count; s++) {
        RGBA c = m->src[s];
        RGBA bg = m->background;
        int d[3] = { c.r - bg.r, c.g - bg.g, c.b - bg.b };
        int v[3] = { p.r - bg.r, p.g - bg.g, p.b - bg.b };
        int k = 0;
        if (abs(d[1]) > abs(d[k])) k = 1;
        if (abs(d[2]) > abs(d[k])) k = 2;
        if (d[k] == 0) continue;
        int t = (255 * v[k] + 127 * (d[k] < 0 ? -1 : 1)) / d[k];
        if (t < 1 || t > 254) continue;
        RGBA q = blend_solid(bg, c, t);
        if (rgba_eq(q, p)) {
            hits++;
            hit_slot = s;
            hit_t = t;
        }
    }
    if (hits == 1) {
        *out_slot = hit_slot;
        *out_t = hit_t;
        return 1;
    }
    return 0;
}

/* straight-alpha edge: RGB is the palette color exactly, alpha is coverage. */
static int solve_straight(const PalMap* m, RGBA p, int* out_slot) {
    int hits = 0, slot = -1;
    for (int s = 0; s < m->count; s++) {
        RGBA c = m->src[s];
        if (p.r == c.r && p.g == c.g && p.b == c.b && p.a > 0 && p.a < 255) {
            hits++;
            slot = s;
        }
    }
    if (hits == 1) {
        *out_slot = slot;
        return 1;
    }
    return 0;
}

/* premultiplied edge: stored RGB == palette RGB * coverage / 255, alpha is
 * the same coverage; every channel must round-trip exactly. */
static int solve_premul(const PalMap* m, RGBA p, int* out_slot, int* out_t) {
    int hits = 0, slot = -1, t = -1;
    for (int s = 0; s < m->count; s++) {
        RGBA c = m->src[s];
        if (p.a == 0 || p.a == 255) continue;
        if (p.r == (c.r * p.a + 127) / 255 &&
            p.g == (c.g * p.a + 127) / 255 &&
            p.b == (c.b * p.a + 127) / 255) {
            hits++;
            slot = s;
            t = p.a;
        }
    }
    if (hits == 1) {
        *out_slot = slot;
        *out_t = t;
        return 1;
    }
    return 0;
}

static RGBA remap_pixel(const PalMap* m, RGBA p) {
    int slot, t;

    /* exact palette hit: hard replace, including the alpha channel */
    for (int s = 0; s < m->count; s++) {
        if (rgba_eq(p, m->src[s])) return m->dst[s];
    }

    if (m->bg == PAL_BG_SOLID) {
        /* fully background and anything unexplainable stays untouched */
        if (solve_solid(m, p, &slot, &t))
            return blend_solid(m->background, m->dst[slot], t);
        return p;
    }

    if (m->bg == PAL_BG_STRAIGHT) {
        if (rgba_eq(p, (RGBA){0, 0, 0, 0})) return p;
        if (solve_straight(m, p, &slot)) {
            RGBA out = m->dst[slot];
            out.a = p.a;
            return out;
        }
        return p;
    }

    /* PAL_BG_PREMUL */
    if (p.a == 0) return (RGBA){0, 0, 0, 0};
    if (solve_premul(m, p, &slot, &t)) {
        RGBA c = m->dst[slot];
        RGBA out;
        out.r = (unsigned char)((c.r * t + 127) / 255);
        out.g = (unsigned char)((c.g * t + 127) / 255);
        out.b = (unsigned char)((c.b * t + 127) / 255);
        out.a = (unsigned char)t;
        return out;
    }
    return p;
}

static void pal_apply(const PalMap* m, RGBA* pixels, int n) {
    if (!m->enabled) return;
    for (int i = 0; i < n; i++)
        pixels[i] = remap_pixel(m, pixels[i]);
}

/* count bytes that differ between two buffers: zero means pixel-identical */
static int count_diff_bytes(const RGBA* a, const RGBA* b, int n) {
    int bad = 0;
    const unsigned char* x = (const unsigned char*)a;
    const unsigned char* y = (const unsigned char*)b;
    for (int i = 0; i < n * (int)sizeof(RGBA); i++)
        if (x[i] != y[i]) bad++;
    return bad;
}

/* ---------------- self-test ---------------- */

#define ATW 16
#define ATH 16
#define N (ATW * ATH)

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void fill(RGBA* buf, RGBA c) {
    for (int i = 0; i < N; i++) buf[i] = c;
}

/* paint one palette block: solid row plus two anti-aliased edge rows */
static void paint_block(RGBA* buf, int slot, const RGBA* pal, PalBgMode bg, RGBA bgc) {
    int y = slot * 4;
    for (int x = 0; x < 8; x++) buf[y * ATW + x] = pal[slot];
    int cover[2] = {64, 192};
    for (int e = 0; e < 2; e++) {
        for (int x = 0; x < 8; x++) {
            RGBA p;
            if (bg == PAL_BG_SOLID) {
                p = blend_solid(bgc, pal[slot], cover[e]);
            } else if (bg == PAL_BG_STRAIGHT) {
                p = pal[slot];
                p.a = (unsigned char)cover[e];
            } else {
                RGBA c = pal[slot];
                p.r = (unsigned char)((c.r * cover[e] + 127) / 255);
                p.g = (unsigned char)((c.g * cover[e] + 127) / 255);
                p.b = (unsigned char)((c.b * cover[e] + 127) / 255);
                p.a = (unsigned char)cover[e];
            }
            buf[(y + 1 + e) * ATW + x] = p;
        }
    }
}

/* hand-export reference: same blend convention, evaluated with dst colors */
static void hand_export(const RGBA* in, RGBA* out, const RGBA* src, int nsrc,
                        const RGBA* dst, PalBgMode bg, RGBA bgc) {
    for (int i = 0; i < N; i++) {
        RGBA p = in[i];
        int matched = 0;
        for (int s = 0; s < nsrc; s++) {
            if (rgba_eq(p, src[s])) { out[i] = dst[s]; matched = 1; break; }
        }
        if (matched) continue;
        if (bg == PAL_BG_SOLID) {
            int slot = -1, t = -1;
            for (int s = 0; s < nsrc; s++) {
                RGBA c = src[s];
                int d[3] = { c.r - bgc.r, c.g - bgc.g, c.b - bgc.b };
                int v[3] = { p.r - bgc.r, p.g - bgc.g, p.b - bgc.b };
                int k = 0;
                if (abs(d[1]) > abs(d[k])) k = 1;
                if (abs(d[2]) > abs(d[k])) k = 2;
                if (d[k] == 0) continue;
                int tt = (255 * v[k] + 127 * (d[k] < 0 ? -1 : 1)) / d[k];
                if (tt < 1 || tt > 254) continue;
                if (rgba_eq(blend_solid(bgc, c, tt), p)) { slot = s; t = tt; break; }
            }
            out[i] = slot >= 0 ? blend_solid(bgc, dst[slot], t) : p;
        } else if (bg == PAL_BG_STRAIGHT) {
            int slot = -1;
            for (int s = 0; s < nsrc; s++) {
                RGBA c = src[s];
                if (p.r == c.r && p.g == c.g && p.b == c.b && p.a > 0 && p.a < 255)
                    { slot = s; break; }
            }
            if (slot >= 0) {
                RGBA q = dst[slot];
                q.a = p.a;
                out[i] = q;
            } else out[i] = p;
        } else {
            int slot = -1;
            for (int s = 0; s < nsrc; s++) {
                RGBA c = src[s];
                if (p.a > 0 && p.a < 255 &&
                    p.r == (c.r * p.a + 127) / 255 &&
                    p.g == (c.g * p.a + 127) / 255 &&
                    p.b == (c.b * p.a + 127) / 255)
                    { slot = s; break; }
            }
            if (slot >= 0) {
                RGBA c = dst[slot];
                RGBA q;
                q.r = (unsigned char)((c.r * p.a + 127) / 255);
                q.g = (unsigned char)((c.g * p.a + 127) / 255);
                q.b = (unsigned char)((c.b * p.a + 127) / 255);
                q.a = p.a;
                out[i] = q;
            } else out[i] = p;
        }
    }
}

int main(void) {
    RGBA src[4] = {
        {200, 40, 40, 255}, {40, 180, 60, 255},
        {50, 90, 220, 255}, {220, 180, 40, 255}
    };
    RGBA dst[4] = {
        {16, 16, 16, 255}, {120, 220, 140, 255},
        {90, 130, 240, 255}, {240, 210, 120, 255}
    };
    RGBA white = {255, 255, 255, 255};
    RGBA clear = {0, 0, 0, 0};

    PalMap m;
    RGBA in[N], out[N], ref[N], before[N];

    /* solid background atlas */
    fill(in, white);
    for (int s = 0; s < 4; s++) paint_block(in, s, src, PAL_BG_SOLID, white);
    RGBA foreign_s = {11, 222, 7, 255};  /* not explainable by any blend */
    in[15 * ATW + 15] = foreign_s;
    hand_export(in, ref, src, 4, dst, PAL_BG_SOLID, white);

    check("solid_build", pal_build(&m, src, 4, dst, 4, NULL, 0,
                                   PAL_BG_SOLID, white) == PAL_OK);
    memcpy(out, in, sizeof(in));
    memcpy(before, in, sizeof(in));
    pal_apply(&m, out, N);
    check("solid_zero_stray", count_diff_bytes(out, ref, N) == 0);
    check("solid_foreign_kept", rgba_eq(out[15 * ATW + 15], foreign_s));

    /* straight-alpha transparent atlas */
    fill(in, clear);
    for (int s = 0; s < 4; s++) paint_block(in, s, src, PAL_BG_STRAIGHT, clear);
    RGBA foreign_t = {11, 222, 7, 128};
    in[15 * ATW + 15] = foreign_t;
    hand_export(in, ref, src, 4, dst, PAL_BG_STRAIGHT, clear);

    check("straight_build", pal_build(&m, src, 4, dst, 4, NULL, 0,
                                      PAL_BG_STRAIGHT, clear) == PAL_OK);
    memcpy(out, in, sizeof(in));
    pal_apply(&m, out, N);
    check("straight_zero_stray", count_diff_bytes(out, ref, N) == 0);
    check("straight_foreign_kept", rgba_eq(out[15 * ATW + 15], foreign_t));
    check("straight_clear_kept", rgba_eq(out[ATW - 1], clear));

    /* premultiplied-alpha transparent atlas */
    fill(in, clear);
    for (int s = 0; s < 4; s++) paint_block(in, s, src, PAL_BG_PREMUL, clear);
    RGBA foreign_p = {3, 5, 2, 128};
    in[15 * ATW + 15] = foreign_p;
    hand_export(in, ref, src, 4, dst, PAL_BG_PREMUL, clear);

    check("premul_build", pal_build(&m, src, 4, dst, 4, NULL, 0,
                                    PAL_BG_PREMUL, clear) == PAL_OK);
    memcpy(out, in, sizeof(in));
    pal_apply(&m, out, N);
    check("premul_zero_stray", count_diff_bytes(out, ref, N) == 0);
    check("premul_foreign_kept", rgba_eq(out[15 * ATW + 15], foreign_p));

    /* replacement disabled: output is identical to input */
    PalMap off;
    memset(&off, 0, sizeof(off));
    check("disabled_default", off.enabled == 0);
    memcpy(out, before, sizeof(before));
    pal_apply(&off, out, N);
    check("disabled_unchanged", memcmp(out, before, sizeof(before)) == 0);

    /* per-color override wins over the slot mapping */
    RGBA custom = {250, 10, 200, 255};
    PalOverride ov = { src[1], custom };
    RGBA dst_ov[4];
    memcpy(dst_ov, dst, sizeof(dst));
    dst_ov[1] = custom;
    hand_export(before, ref, src, 4, dst_ov, PAL_BG_SOLID, white);
    check("override_build", pal_build(&m, src, 4, dst, 4, &ov, 1,
                                      PAL_BG_SOLID, white) == PAL_OK);
    memcpy(out, before, sizeof(before));
    pal_apply(&m, out, N);
    check("override_zero_stray", count_diff_bytes(out, ref, N) == 0);

    /* validation: length mismatch and duplicate colors are rejected */
    check("reject_length", pal_build(&m, src, 3, dst, 4, NULL, 0,
                                     PAL_BG_SOLID, white) == PAL_ERR_LENGTH);
    RGBA bad_src[4];
    memcpy(bad_src, src, sizeof(src));
    bad_src[2] = src[0];
    check("reject_dup_src", pal_build(&m, bad_src, 4, dst, 4, NULL, 0,
                                      PAL_BG_SOLID, white) == PAL_ERR_DUP_SRC);
    RGBA bad_dst[4];
    memcpy(bad_dst, dst, sizeof(dst));
    bad_dst[3] = dst[0];
    check("reject_dup_dst", pal_build(&m, src, 4, bad_dst, 4, NULL, 0,
                                      PAL_BG_SOLID, white) == PAL_ERR_DUP_DST);
    check("reject_empty", pal_build(&m, NULL, 0, dst, 4, NULL, 0,
                                    PAL_BG_SOLID, white) == PAL_ERR_EMPTY);

    /* override validation: unknown key, duplicate override, target collision */
    RGBA unknown = {1, 2, 3, 255};
    PalOverride bad_ov = { unknown, custom };
    check("reject_override_key",
          pal_build(&m, src, 4, dst, 4, &bad_ov, 1,
                    PAL_BG_SOLID, white) == PAL_ERR_OVERRIDE_KEY);
    PalOverride ov2[2] = { { src[1], custom }, { src[1], dst[2] } };
    check("reject_dup_override",
          pal_build(&m, src, 4, dst, 4, ov2, 2,
                    PAL_BG_SOLID, white) == PAL_ERR_DUP_OVERRIDE);
    PalOverride collide = { src[1], dst[2] };
    check("reject_override_collision",
          pal_build(&m, src, 4, dst, 4, &collide, 1,
                    PAL_BG_SOLID, white) == PAL_ERR_OVERRIDE_DUP);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
