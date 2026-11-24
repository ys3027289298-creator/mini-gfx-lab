/* transsort: depth-sorted blending of translucent surfaces (water over a
   ship hull).  A software framebuffer reproduces the bright rim that appears
   when semi-transparent water is composited in the wrong order against the
   ship's anti-aliased silhouette.

   Two defects must be fixed together:
     1. sort key degeneracy  - a depth-only key cannot order equal-depth
        items, so an unstable sort silently reorders them;
     2. state-switch insertion - a batcher that regroups draws by render
        state (blend/texture/program) destroys the depth order.

   The fixed pipeline uses a composite key (depth, opaque-first, stable
   sequence id) with a stable sort, then walks the sorted list and only
   *applies* state changes - it never reorders for them. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define W 48
#define H 32
#define MAX_ITEMS 8

/* ---------- render states: blend mode + color + opacity ---------- */
enum { BLEND_OVER = 0, BLEND_ADD = 1 };

typedef struct {
    const char* name;
    int blend;
    float r, g, b, alpha;
} State;

static const State STATES[] = {
    /* 0 */ { "hull_opaque", BLEND_OVER, 0.35f, 0.22f, 0.12f, 1.00f },
    /* 1 */ { "water_semi",  BLEND_OVER, 0.20f, 0.55f, 0.75f, 0.45f },
    /* 2 */ { "foam_add",    BLEND_ADD,  0.90f, 0.95f, 1.00f, 0.50f },
};
#define NSTATES ((int)(sizeof(STATES) / sizeof(STATES[0])))

typedef struct {
    int id;
    const char* name;
    int state;
    float depth;                 /* smaller = nearer the camera */
    float x0, y0, x1, y1;        /* rectangle in pixel space */
    int seq;                     /* stable submission index */
} Item;

typedef struct { float r, g, b; } Fb;

static Fb FB_REF[H][W];

/* anti-aliased rectangle coverage at a pixel center (1 px AA border) */
static float cov_rect(float px, float py, const Item* it) {
    float dx = px - it->x0; if (it->x1 - px < dx) dx = it->x1 - px;
    float dy = py - it->y0; if (it->y1 - py < dy) dy = it->y1 - py;
    if (dx <= 0.0f || dy <= 0.0f) return 0.0f;
    if (dx > 1.0f) dx = 1.0f;
    if (dy > 1.0f) dy = 1.0f;
    return dx * dy;
}

static void blend_pixel(Fb* px, const State* st, float cov) {
    float a = st->alpha * cov;
    if (a <= 0.0f) return;
    if (st->blend == BLEND_ADD) {
        px->r += st->r * a; px->g += st->g * a; px->b += st->b * a;
    } else {
        px->r = st->r * a + px->r * (1.0f - a);
        px->g = st->g * a + px->g * (1.0f - a);
        px->b = st->b * a + px->b * (1.0f - a);
    }
    if (px->r > 1) px->r = 1;
    if (px->g > 1) px->g = 1;
    if (px->b > 1) px->b = 1;
}

static void fb_clear(Fb fb[H][W]) {
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            fb[y][x] = (Fb){ 0.05f, 0.10f, 0.18f };
}

static void draw_item(Fb fb[H][W], const Item* it) {
    const State* st = &STATES[it->state];
    int xa = (int)floorf(it->x0) - 1, xb = (int)ceilf(it->x1) + 1;
    int ya = (int)floorf(it->y0) - 1, yb = (int)ceilf(it->y1) + 1;
    if (xa < 0) xa = 0;
    if (xb >= W) xb = W - 1;
    if (ya < 0) ya = 0;
    if (yb >= H) yb = H - 1;
    for (int y = ya; y <= yb; y++)
        for (int x = xa; x <= xb; x++) {
            float c = cov_rect(x + 0.5f, y + 0.5f, it);
            if (c > 0.0f) blend_pixel(&fb[y][x], st, c);
        }
}

/* ---------- ordering ---------- */
static int is_opaque(const Item* it) { return STATES[it->state].alpha >= 1.0f; }

/* composite sort key in standard convention: cmp(a,b) < 0 means a is
   placed earlier, and earlier means drawn first (painter's order).
   Draw order is far-to-near, so LARGER depth sorts earlier; at equal
   depth the opaque surface goes first; seq breaks any remaining tie. */
static int key_full(const Item* a, const Item* b) {
    if (a->depth > b->depth) return -1;
    if (a->depth < b->depth) return 1;
    int oa = is_opaque(a), ob = is_opaque(b);
    if (oa != ob) return oa ? -1 : 1;
    return (a->seq < b->seq) ? -1 : (a->seq > b->seq) ? 1 : 0;
}

/* degenerate key: depth alone.  Equal-depth items get an arbitrary tiebreak
   (reverse submission order) - this is exactly what an unstable sort does
   when the comparator returns 0, and nothing in the key forbids it. */
static int key_depth_only(const Item* a, const Item* b) {
    if (a->depth > b->depth) return -1;
    if (a->depth < b->depth) return 1;
    return (a->seq > b->seq) ? -1 : (a->seq < b->seq) ? 1 : 0;
}

/* insertion sort producing comparator order: cmp(a,b) < 0 means a belongs
   before b, so shift while the earlier element belongs after v */
static void insertsort_items(Item* a, int n, int (*cmp)(const Item*, const Item*)) {
    for (int i = 1; i < n; i++) {
        Item v = a[i];
        int j = i - 1;
        while (j >= 0 && cmp(&a[j], &v) > 0) { a[j + 1] = a[j]; j--; }
        a[j + 1] = v;
    }
}

enum { P_NAIVE = 0, P_DEGEN_KEY, P_STATE_BUCKET, P_FIXED, NPIPE };
static const char* PIPE_NAME[NPIPE] = {
    "naive-submission ", "depth-only-key   ", "state-bucket     ", "fixed-composite  "
};

/* fills order[] with item indices in the order the pipeline draws them */
static void build_order(const Item* scene, int n, int pipe, int order[MAX_ITEMS]) {
    Item tmp[MAX_ITEMS];
    memcpy(tmp, scene, (size_t)n * sizeof(Item));
    if (pipe == P_NAIVE) {
        for (int i = 0; i < n; i++) order[i] = i;
    } else if (pipe == P_DEGEN_KEY) {
        insertsort_items(tmp, n, key_depth_only);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) if (scene[j].id == tmp[i].id) order[i] = j;
    } else if (pipe == P_STATE_BUCKET) {
        /* all hull draws go out first, then water, then additive foam;
           each bucket is depth-sorted, so the only wrong inversions left
           are the cross-state ones inserted to skip a state switch */
        int k = 0;
        for (int s = 0; s < NSTATES; s++) {
            Item bucket[MAX_ITEMS]; int nb = 0;
            for (int i = 0; i < n; i++)
                if (scene[i].state == s) bucket[nb++] = scene[i];
            insertsort_items(bucket, nb, key_full);
            for (int i = 0; i < nb; i++)
                for (int j = 0; j < n; j++)
                    if (scene[j].id == bucket[i].id) order[k++] = j;
        }
    } else { /* P_FIXED: stable sort on the composite key */
        insertsort_items(tmp, n, key_full);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) if (scene[j].id == tmp[i].id) order[i] = j;
    }
}

/* per-pixel ground truth: sort every covering item by the composite key */
static void render_reference(const Item* scene, int n) {
    fb_clear(FB_REF);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            const Item* hit[MAX_ITEMS]; int nh = 0;
            for (int i = 0; i < n; i++) {
                if (cov_rect(x + 0.5f, y + 0.5f, &scene[i]) > 0.0f)
                    hit[nh++] = &scene[i];
            }
            for (int i = 0; i < nh; i++)
                for (int j = i + 1; j < nh; j++)
                    if (key_full(hit[j], hit[i]) < 0) {
                        const Item* t = hit[i]; hit[i] = hit[j]; hit[j] = t;
                    }
            for (int i = 0; i < nh; i++) {
                float c = cov_rect(x + 0.5f, y + 0.5f, hit[i]);
                blend_pixel(&FB_REF[y][x], &STATES[hit[i]->state], c);
            }
        }
    }
}

static float lum(Fb p) { return 0.2126f * p.r + 0.7152f * p.g + 0.0722f * p.b; }

/* fringe: measured on the ship silhouette's anti-aliased edge pixels */
static int edge_pixel(const Item* scene, int n, int x, int y) {
    for (int i = 0; i < n; i++) {
        if (!is_opaque(&scene[i])) continue;
        float c = cov_rect(x + 0.5f, y + 0.5f, &scene[i]);
        if (c > 0.02f && c < 0.98f) return 1;
    }
    return 0;
}

typedef struct {
    float max_abs, max_bright;
    int rim_px, edge_px;
    float edge_lum, inner_lum;   /* mean luminance of hull edge band / hull interior */
} Fringe;

static Fringe measure(const Item* scene, int n, Fb fb[H][W]) {
    Fringe f = { 0.0f, 0.0f, 0, 0, 0.0f, 0.0f };
    double esum = 0.0, isum = 0.0; int inner_px = 0;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            int on_edge = edge_pixel(scene, n, x, y);
            int inside = 0;
            for (int i = 0; i < n && !inside; i++)
                if (is_opaque(&scene[i]) &&
                    cov_rect(x + 0.5f, y + 0.5f, &scene[i]) >= 0.98f) inside = 1;
            if (inside) { isum += lum(fb[y][x]); inner_px++; }
            if (!on_edge) continue;
            f.edge_px++;
            esum += lum(fb[y][x]);
            float d = lum(fb[y][x]) - lum(FB_REF[y][x]);
            float ad = fabsf(d);
            if (ad > f.max_abs) f.max_abs = ad;
            if (d > f.max_bright) f.max_bright = d;
            if (ad > 0.02f) f.rim_px++;
        }
    if (f.edge_px) f.edge_lum = (float)(esum / f.edge_px);
    if (inner_px) f.inner_lum = (float)(isum / inner_px);
    return f;
}

/* two rectangles share at least one anti-aliased edge pixel */
static int items_overlap(const Item* a, const Item* b) {
    return a->x0 < b->x1 && b->x0 < a->x1 && a->y0 < b->y1 && b->y0 < a->y1;
}

/* score how strongly an inversion explains the rim.  An OVER translucent
   draw crossing the opaque hull is non-commutative and produces the
   silhouette edge; translucent/translucent swaps score lower and additive
   vs opaque swaps cancel out */
static int inversion_score(const Item* scene, int a, int b) {
    int opaque_a = is_opaque(&scene[a]), opaque_b = is_opaque(&scene[b]);
    int over_a = STATES[scene[a].state].blend == BLEND_OVER;
    int over_b = STATES[scene[b].state].blend == BLEND_OVER;
    if ((opaque_a || opaque_b) && over_a && over_b) return 3;
    if (opaque_a || opaque_b) return 2;
    if (over_a && over_b) return 1;
    return 0;
}

/* painter's rule: the farther surface (larger depth) is drawn first.
   An inversion is any overlapping pair drawn in the wrong depth order;
   equal-depth pairs are inversions too, but only the degenerate-key
   pipeline is allowed to misorder them. */
static int find_inversion(const Item* scene, int n, const int* order,
                          int* out_i, int* out_j) {
    int best = -1;
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            int a = order[i], b = order[j];
            if (!items_overlap(&scene[a], &scene[b])) continue;
            /* painter's legality: the earlier draw must be the farther
               surface (a.depth >= b.depth).  Anything else is an
               inversion; at equal depth opaque must precede translucent. */
            int wrong;
            if (scene[a].depth != scene[b].depth)
                wrong = scene[a].depth < scene[b].depth;
            else
                wrong = !(is_opaque(&scene[a]) && !is_opaque(&scene[b]));
            if (wrong) {
                int s = inversion_score(scene, a, b);
                if (s > best) { best = s; *out_i = i; *out_j = j; }
            }
        }
    }
    return best >= 0;
}

static void diagnose(const Item* scene, int n, const int* order, int pipe, Fringe f) {
    if (f.rim_px == 0) { printf("  verdict: PASS, no rim\n"); return; }
    int bi = -1, bj = -1;
    find_inversion(scene, n, order, &bi, &bj);
    if (bi >= 0) {
        const Item* hoisted = &scene[order[bi]];   /* drawn too early */
        const Item* delayed = &scene[order[bj]];   /* should have gone first */
        printf("  verdict: RIM, root cause = ");
        if (pipe == P_DEGEN_KEY) {
            printf("SORT-KEY DEGENERACY\n");
            printf("    equal-depth pair '%s' (state %s) and '%s' (state %s):\n",
                   hoisted->name, STATES[hoisted->state].name,
                   delayed->name, STATES[delayed->state].name);
            printf("    depth-only key cannot break the tie, unstable sort drew '%s' first\n",
                   hoisted->name);
        } else if (pipe == P_STATE_BUCKET) {
            printf("STATE-SWITCH INSERTION\n");
            printf("    '%s' (state %s, depth %.1f) hoisted in front of '%s' (state %s, depth %.1f)\n",
                   hoisted->name, STATES[hoisted->state].name, hoisted->depth,
                   delayed->name, STATES[delayed->state].name, delayed->depth);
            printf("    depth order broken to keep one render state; the opaque/translucent crossing is non-commutative\n");
        } else {
            printf("NO DEPTH SORT\n");
            printf("    submission order draws '%s' (depth %.1f) before '%s' (depth %.1f), violating far-to-near\n",
                   hoisted->name, hoisted->depth, delayed->name, delayed->depth);
        }
        return;
    }
    printf("  verdict: RIM (no order inversion found; check blending math)\n");
}

static void print_order(const Item* scene, int n, const int* order) {
    int cur = -1;
    for (int i = 0; i < n; i++) {
        const Item* it = &scene[order[i]];
        if (it->state != cur) {
            printf("%s[switch -> %s]", i ? " " : "", STATES[it->state].name);
            cur = it->state;
        }
        printf(" %s(d%.1f)", it->name, it->depth);
    }
    printf("\n");
}

static int run_scene(const char* title, const Item* scene, int n) {
    printf("== %s ==\n", title);
    render_reference(scene, n);
    int fixed_bad = 0, any_buggy_rim = 0;
    for (int pipe = 0; pipe < NPIPE; pipe++) {
        int order[MAX_ITEMS];
        build_order(scene, n, pipe, order);
        Fb fb[H][W]; fb_clear(fb);
        printf("[%s] draw:", PIPE_NAME[pipe]);
        print_order(scene, n, order);
        for (int i = 0; i < n; i++) draw_item(fb, &scene[order[i]]);
        Fringe f = measure(scene, n, fb);
        printf("  edge_px=%d rim_px=%d max|dL|=%.3f max_bright_dL=%.3f edgeL=%.3f innerL=%.3f rim_gap=%+.3f\n",
               f.edge_px, f.rim_px, f.max_abs, f.max_bright,
               f.edge_lum, f.inner_lum, f.edge_lum - f.inner_lum);
        diagnose(scene, n, order, pipe, f);
        if (pipe == P_FIXED) fixed_bad = (f.rim_px != 0);
        else if (f.rim_px > 0) any_buggy_rim = 1;
    }
    printf("\n");
    return fixed_bad + (any_buggy_rim ? 0 : 1);
}

int main(void) {
    int bad = 0;

    /* A: water fully in front of the ship (near water, far hull) */
    Item scene_a[] = {
        { 3, "foam ", 2, 4.0f, 14, 13, 34, 15, 0 },
        { 2, "water", 1, 5.0f,  4, 14, 44, 20, 1 },
        { 1, "ship ", 0, 8.0f, 10, 12, 38, 22, 2 },
    };
    bad += run_scene("scene A: water fully in FRONT of ship", scene_a, 3);

    /* B: bow cuts into the water - bow and water share one depth plane.
       Submission order already has water after the bow, so the degenerate
       key's arbitrary reverse-seq tiebreak flips them. */
    Item scene_b[] = {
        { 11, "stern", 0, 9.0f,  6, 12, 22, 22, 0 },
        { 12, "bow  ", 0, 6.0f, 22, 12, 40, 22, 1 },
        { 13, "water", 1, 6.0f,  4, 14, 44, 20, 2 },
    };
    bad += run_scene("scene B: bow PARTLY in water (bow/water equal depth)", scene_b, 3);

    /* C: water fully behind the ship.  The water surface and the foam band
       both cross the hull's top edge; moving the OVER water in front of the
       opaque hull (as state batching does) is non-commutative and lights
       the silhouette. */
    Item scene_c[] = {
        { 21, "ship ", 0, 3.0f, 10, 10, 38, 20, 0 },
        { 22, "foam ", 2, 2.8f, 24,  8, 44, 13, 1 },
        { 23, "water", 1, 2.0f,  4,  8, 44, 19, 2 },
    };
    bad += run_scene("scene C: water fully BEHIND ship", scene_c, 3);

    printf(bad == 0
           ? "summary: fixed pipeline rim-free in all 3 scenes; each scene reproduces a rim on at least one defect pipeline\n"
           : "summary: UNEXPECTED %d invariant failures\n", bad);
    return bad != 0;
}
