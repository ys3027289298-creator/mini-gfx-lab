
/* fontatlas: shelf packing of glyph bitmaps into a fixed atlas, plus kerning. */
#include <stdio.h>
#include <string.h>
#include "diag.h"

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

static int failures = 0;
static Diag g_diag;

#if DIAG
/* evidence material: names of the failed checks, for the repro context */
#define FAIL_CAP 16
static const char* fail_names[FAIL_CAP];
static int fail_n;

static void emit_repro_command(void) {
    int i;
    fprintf(stderr, "[fontatlas] failed checks:");
    for (i = 0; i < fail_n; i++) fprintf(stderr, " %s", fail_names[i]);
    fprintf(stderr, "\n");
    fprintf(stderr, "[fontatlas] repro (copy & paste from the repo root):\n");
    if (g_diag.forced)
        fprintf(stderr, "  make -C fontatlas diag-test MINI_GFX_FAULT=fontatlas\n");
    else
        fprintf(stderr, "  make -C fontatlas diag-test\n");
}
#endif

static void check(const char* name, int ok) {
#if DIAG
    diag_check(&g_diag, ok);
    if (!ok && fail_n < FAIL_CAP) fail_names[fail_n++] = name;
    DIAG_DRAW("%s=%d\n", name, ok ? 1 : 0);
#else
    printf("%s=%d\n", name, ok ? 1 : 0);
#endif
    if (!ok) failures++;
}

int main(void) {
    diag_begin(&g_diag, "fontatlas");
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

#if DIAG
    diag_check(&g_diag, !diag_forced(&g_diag));
    if (g_diag.forced && fail_n < FAIL_CAP) fail_names[fail_n++] = "no_injected_fault";
    DIAG_DRAW("all_ok=%d\n", failures == 0);
    if (diag_ok(&g_diag)) { diag_draw_flush(); return 0; }
    diag_drop_notice(&g_diag);
    emit_repro_command();
    return DIAG_EXIT_DROP;
#else
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
#endif
}
