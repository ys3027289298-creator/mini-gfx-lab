/* particle-pool: fixed-capacity particle pool with per-frame spawn budget,
   emitter priority scheduling, and stable layer/depth/blend draw-order keys. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MAX_PARTICLES 4096
#define MAX_EMITTERS 8
#define MAX_SCENE_OBJECTS 64
#define MAX_DRAW_ITEMS (MAX_PARTICLES + MAX_SCENE_OBJECTS)

typedef enum { BLEND_OPAQUE = 0, BLEND_ALPHA = 1, BLEND_ADDITIVE = 2 } BlendMode;

typedef struct {
    float x, y, vx, vy, life, age;
    float depth;                 /* camera distance; larger = farther away */
    float size;                  /* half-extent used by the demo rasterizer */
    unsigned rgba;
    unsigned char layer, blend;
    int alive;
    int tag;
} Particle;

typedef struct {
    int id;
    int quota;
    int spawned;
    int priority;
    float depth;
    unsigned char layer, blend;
    unsigned rgba;
} Emitter;

typedef struct {
    int x0, y0, x1, y1;          /* inclusive pixel rect */
    float depth;
    unsigned char layer, blend;
    unsigned rgba;
    int visible;
} SceneObject;

typedef struct {
    Particle pool[MAX_PARTICLES];
    int alive_count, high_water;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;
    SceneObject scene[MAX_SCENE_OBJECTS];
    int scene_count;
} World;

/* One draw-list entry. Particles and scene objects share this payload so the
   key-sorted list and the hand-sorted reference list can be rendered by the
   same rasterizer and compared byte for byte. */
typedef struct {
    uint64_t key;
    float depth;
    unsigned char layer, blend;
    int x0, y0, x1, y1;
    unsigned rgba;
    int seq;                     /* submission position, for the stable tie-break */
} DrawItem;

static void world_init(World* w) {
    memset(w, 0, sizeof(*w));
    w->emitter_count = 0;
}

static unsigned char clamp_u8(int v) {
    return (unsigned char)(v < 0 ? 0 : (v > 255 ? 255 : v));
}

static unsigned char clamp_blend(int v) {
    if (v < (int)BLEND_OPAQUE) return BLEND_OPAQUE;
    if (v > (int)BLEND_ADDITIVE) return BLEND_ADDITIVE;
    return (unsigned char)v;
}

/* 0xRRGGBBAA */
static unsigned palette_rgba(int tag) {
    static const unsigned pal[] = {
        0xff7043ffu, 0x42a5f5ffu, 0x66bb6affu,
        0xffca28ffu, 0xab47bcffu, 0x26c6daffu
    };
    unsigned idx = (unsigned)(tag < 0 ? -tag : tag) % (unsigned)(sizeof pal / sizeof pal[0]);
    return pal[idx];
}

static int emitter_add(World* w, int id, int quota, int priority) {
    if (w->emitter_count >= MAX_EMITTERS) return -1;
    Emitter* e = &w->emitters[w->emitter_count++];
    e->id = id;
    e->quota = quota;
    e->spawned = 0;
    e->priority = priority;
    e->depth = 1.0f;
    e->layer = 0;
    e->blend = BLEND_ALPHA;
    e->rgba = palette_rgba(id);
    return e->id;
}

static Emitter* emitter_find(World* w, int id) {
    for (int i = 0; i < w->emitter_count; i++)
        if (w->emitters[i].id == id) return &w->emitters[i];
    return NULL;
}

static void emitter_visual(World* w, int id, float depth, int layer, int blend, unsigned rgba) {
    Emitter* e = emitter_find(w, id);
    if (!e) return;
    e->depth = depth;
    e->layer = clamp_u8(layer);
    e->blend = clamp_blend(blend);
    e->rgba = rgba;
}

static int scene_add(World* w, int x0, int y0, int x1, int y1,
                     float depth, int layer, int blend, unsigned rgba) {
    if (w->scene_count >= MAX_SCENE_OBJECTS) return -1;
    SceneObject* s = &w->scene[w->scene_count++];
    s->x0 = x0; s->y0 = y0; s->x1 = x1; s->y1 = y1;
    s->depth = depth;
    s->layer = clamp_u8(layer);
    s->blend = clamp_blend(blend);
    s->rgba = rgba;
    s->visible = 1;
    return 0;
}

/* ------------------------------------------------------------------------- */
/* Draw-order keys                                                           */
/*                                                                           */
/* Ascending key == paint order (first drawn to last drawn):                 */
/*   bits 63..34  layer   0 = background                                      */
/*   bits 33..2   depth   inverted camera distance, so far draws before near */
/*   bits  1..0   blend   opaque -> alpha -> additive (tie-break only)        */
/*                                                                           */
/* The particle index is deliberately NOT part of the key: items whose       */
/* layer/depth/blend collide share a key, and the stable sort preserves      */
/* their submission order.                                                    */
/* ------------------------------------------------------------------------- */

#define SORT_KEY_BLEND_SHIFT 0
#define SORT_KEY_DEPTH_SHIFT 2
#define SORT_KEY_LAYER_SHIFT 34

static long keygen_calls;        /* instrumentation: proves O(1) work per item */

/* NaN, negative distances and -0.0 collapse to 0 so the numeric comparator
   used by the reference sort and the quantized key agree on every value. */
static float depth_clamp(float d) {
    if (!(d > 0.0f)) return 0.0f;
    return d;
}

/* Non-negative IEEE-754 bit patterns are monotonic in numeric value. The
   result is inverted at the call site so ascending keys paint far first. */
static uint32_t depth_order_bits(float d) {
    float c = depth_clamp(d);
    uint32_t bits;
    memcpy(&bits, &c, sizeof bits);
    return ~bits;
}

static uint64_t make_draw_key(int layer, float depth, int blend) {
    keygen_calls++;
    return ((uint64_t)((unsigned)layer & 0xFFu) << SORT_KEY_LAYER_SHIFT)
         | ((uint64_t)depth_order_bits(depth) << SORT_KEY_DEPTH_SHIFT)
         | (uint64_t)((unsigned)blend & 0x3u);
}

static uint64_t particle_sort_key(const Particle* p) {
    return make_draw_key(p->layer, p->depth, p->blend);
}

/* ------------------------------------------------------------------------- */
/* Pool and emitters                                                         */
/* ------------------------------------------------------------------------- */

static int pool_find_free(World* w) {
    for (int i = 0; i < MAX_PARTICLES; i++) if (!w->pool[i].alive) return i;
    return -1;
}

static int spawn_at(World* w, int tag, float x, float y, float size,
                    float depth, int layer, int blend, unsigned rgba) {
    int idx = pool_find_free(w);
    if (idx < 0) return -1;
    Particle* p = &w->pool[idx];
    p->alive = 1;
    p->age = 0.0f;
    p->x = x; p->y = y;
    p->vx = 1.0f; p->vy = 0.5f;
    p->life = 2.0f;
    p->depth = depth_clamp(depth);
    p->size = size > 0.0f ? size : 1.0f;
    p->layer = clamp_u8(layer);
    p->blend = clamp_blend(blend);
    p->rgba = rgba;
    p->tag = tag;
    w->alive_count++;
    if (w->alive_count > w->high_water) w->high_water = w->alive_count;
    return idx;
}

static int spawn_one(World* w, const Emitter* e) {
    return spawn_at(w, e->id, 0.0f, 0.0f, 2.0f, e->depth, e->layer, e->blend, e->rgba);
}

/* highest priority emitters spend their quota first */
static void spawn_frame(World* w, int budget) {
    int left = budget;
    int order[MAX_EMITTERS]; int n = w->emitter_count;
    for (int i = 0; i < n; i++) order[i] = i;
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++)
            if (w->emitters[order[j]].priority > w->emitters[order[i]].priority) {
                int t = order[i]; order[i] = order[j]; order[j] = t;
            }
    for (int k = 0; k < n && left > 0; k++) {
        Emitter* e = &w->emitters[order[k]];
        e->spawned = 0;
        int want = e->quota < left ? e->quota : left;
        for (int i = 0; i < want; i++) if (spawn_one(w, e) >= 0) { e->spawned++; left--; }
    }
}

static void update(World* w, float dt) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive) continue;
        p->age += dt;
        p->x += p->vx * dt; p->y += p->vy * dt;
        if (p->age >= p->life) { p->alive = 0; w->alive_count--; }
    }
}

/* ------------------------------------------------------------------------- */
/* Draw list build and sorts                                                 */
/* ------------------------------------------------------------------------- */

static DrawItem particle_item(const Particle* p, int seq) {
    DrawItem it;
    memset(&it, 0, sizeof it);
    int cx = (int)lroundf(p->x);
    int cy = (int)lroundf(p->y);
    int half = (int)lroundf(p->size);
    if (half < 1) half = 1;
    it.x0 = cx - half; it.x1 = cx + half - 1;
    it.y0 = cy - half; it.y1 = cy + half - 1;
    it.depth = depth_clamp(p->depth);
    it.layer = p->layer;
    it.blend = p->blend;
    it.rgba = p->rgba;
    it.seq = seq;
    return it;
}

/* One pass over the live particles and visible scene objects. Keys are
   generated exactly once per emitted item; the sort below only compares them. */
static int drawlist_build(const World* w, DrawItem* out, int generate_keys) {
    int n = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle* p = &w->pool[i];
        if (!p->alive) continue;
        out[n] = particle_item(p, n);
        if (generate_keys) out[n].key = particle_sort_key(p);
        n++;
    }
    for (int i = 0; i < w->scene_count; i++) {
        const SceneObject* s = &w->scene[i];
        if (!s->visible) continue;
        DrawItem* it = &out[n];
        memset(it, 0, sizeof *it);
        it->x0 = s->x0; it->y0 = s->y0; it->x1 = s->x1; it->y1 = s->y1;
        it->depth = depth_clamp(s->depth);
        it->layer = s->layer;
        it->blend = s->blend;
        it->rgba = s->rgba;
        it->seq = n;
        if (generate_keys) it->key = make_draw_key(s->layer, s->depth, s->blend);
        n++;
    }
    return n;
}

static DrawItem sort_scratch[MAX_DRAW_ITEMS];

/* Bottom-up stable merge sort on the precomputed keys. Equal keys take the
   left run first, so colliding items keep their submission order. */
static void drawlist_sort(DrawItem* a, int n) {
    for (int width = 1; width < n; width *= 2) {
        for (int lo = 0; lo < n; lo += 2 * width) {
            int mid = lo + width < n ? lo + width : n;
            int hi = lo + 2 * width < n ? lo + 2 * width : n;
            int i = lo, j = mid, k = lo;
            while (i < mid && j < hi) {
                if (a[j].key < a[i].key) sort_scratch[k++] = a[j++];
                else sort_scratch[k++] = a[i++];     /* ties: earlier item first */
            }
            while (i < mid) sort_scratch[k++] = a[i++];
            while (j < hi) sort_scratch[k++] = a[j++];
        }
        memcpy(a, sort_scratch, (size_t)n * sizeof *a);
    }
}

/* Reference sort: compare the fields one by one, no keys involved. Depth is
   compared descending (far paints first) to match the inverted key field.
   The final seq comparison makes it a strict total order, matching stable
   key order. */
static int manual_less(const DrawItem* a, const DrawItem* b) {
    if (a->layer != b->layer) return a->layer < b->layer;
    if (a->depth != b->depth) return a->depth > b->depth;
    if (a->blend != b->blend) return a->blend < b->blend;
    return a->seq < b->seq;
}

static void drawlist_sort_manual(DrawItem* a, int n) {
    for (int i = 1; i < n; i++) {
        DrawItem t = a[i];
        int j = i - 1;
        while (j >= 0 && manual_less(&t, &a[j])) { a[j + 1] = a[j]; j--; }
        a[j + 1] = t;
    }
}

/* Public-ish entry point: build the full back-to-front draw list. */
static int world_drawlist(const World* w, DrawItem* out) {
    int n = drawlist_build(w, out, 1);
    drawlist_sort(out, n);
    return n;
}

/* ------------------------------------------------------------------------- */
/* Small deterministic rasterizer used to prove pixel equality               */
/* ------------------------------------------------------------------------- */

#define FB_W 64
#define FB_H 64

typedef struct { unsigned char rgb[FB_H][FB_W][3]; } Framebuffer;

static void fb_render(Framebuffer* fb, const DrawItem* list, int n) {
    memset(fb, 0, sizeof *fb);
    for (int k = 0; k < n; k++) {
        const DrawItem* it = &list[k];
        int x0 = it->x0 < 0 ? 0 : it->x0;
        int y0 = it->y0 < 0 ? 0 : it->y0;
        int x1 = it->x1 >= FB_W ? FB_W - 1 : it->x1;
        int y1 = it->y1 >= FB_H ? FB_H - 1 : it->y1;
        unsigned src[3] = {
            (it->rgba >> 24) & 0xFFu,
            (it->rgba >> 16) & 0xFFu,
            (it->rgba >> 8) & 0xFFu
        };
        unsigned sa = it->rgba & 0xFFu;
        if (it->blend == BLEND_OPAQUE) sa = 255;
        for (int y = y0; y <= y1; y++) {
            for (int x = x0; x <= x1; x++) {
                unsigned char* px = fb->rgb[y][x];
                for (int c = 0; c < 3; c++) {
                    unsigned v;
                    if (it->blend == BLEND_OPAQUE) {
                        v = src[c];
                    } else if (it->blend == BLEND_ADDITIVE) {
                        v = (unsigned)px[c] + (src[c] * sa + 127u) / 255u;
                    } else {
                        v = (src[c] * sa + (unsigned)px[c] * (255u - sa) + 127u) / 255u;
                    }
                    px[c] = (unsigned char)(v > 255u ? 255u : v);
                }
            }
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Tests                                                                     */
/* ------------------------------------------------------------------------- */

static DrawItem g_keyed[MAX_DRAW_ITEMS];
static DrawItem g_manual[MAX_DRAW_ITEMS];

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static unsigned rng_state;
static unsigned rng_next(void) {
    rng_state = rng_state * 1664525u + 1013904223u;
    return rng_state;
}
static int rng_int(int lo, int hi) {
    return lo + (int)(rng_next() % (unsigned)(hi - lo + 1));
}

static const float depth_choices[] = { 0.0f, 0.25f, 1.0f, 1.0f, 2.5f, 7.0f };

static void build_random_world(World* w, unsigned seed) {
    rng_state = seed;
    world_init(w);
    int np = rng_int(120, 260);
    for (int i = 0; i < np; i++) {
        unsigned rgba = ((unsigned)rng_int(0, 255) << 24)
                      | ((unsigned)rng_int(0, 255) << 16)
                      | ((unsigned)rng_int(0, 255) << 8)
                      | (unsigned)rng_int(96, 255);
        spawn_at(w, rng_int(1, 5),
                 (float)rng_int(4, 59), (float)rng_int(4, 59),
                 (float)rng_int(3, 18),
                 depth_choices[rng_int(0, (int)(sizeof depth_choices / sizeof depth_choices[0]) - 1)],
                 rng_int(0, 3), rng_int(0, 2), rgba);
    }
    int ns = rng_int(0, 8);
    for (int i = 0; i < ns; i++) {
        unsigned rgba = ((unsigned)rng_int(0, 255) << 24)
                      | ((unsigned)rng_int(0, 255) << 16)
                      | ((unsigned)rng_int(0, 255) << 8)
                      | (unsigned)rng_int(96, 255);
        int x0 = rng_int(-8, 32), y0 = rng_int(-8, 32);
        scene_add(w, x0, y0, x0 + rng_int(8, 48), y0 + rng_int(8, 48),
                  depth_choices[rng_int(0, (int)(sizeof depth_choices / sizeof depth_choices[0]) - 1)],
                  rng_int(0, 3), rng_int(0, 2), rgba);
    }
}

/* Key order must match a hand-written field comparator, and both orders must
   rasterize to the exact same pixels. */
static void randomized_consistency_case(void) {
    int order_ok = 1, pixel_ok = 1, stable_ok = 1, sorted_ok = 1;
    long collisions = 0;
    for (unsigned seed = 1; seed <= 48; seed++) {
        World w;
        build_random_world(&w, seed);
        int nk = drawlist_build(&w, g_keyed, 1);
        int nm = drawlist_build(&w, g_manual, 0);
        if (nk != nm) { order_ok = 0; pixel_ok = 0; continue; }

        drawlist_sort(g_keyed, nk);
        drawlist_sort_manual(g_manual, nm);

        for (int i = 0; i < nk; i++) {
            if (g_keyed[i].seq != g_manual[i].seq) order_ok = 0;
            if (i > 0) {
                if (g_keyed[i - 1].key > g_keyed[i].key) sorted_ok = 0;
                if (g_keyed[i - 1].key == g_keyed[i].key) {
                    collisions++;
                    if (g_keyed[i - 1].seq > g_keyed[i].seq) stable_ok = 0;
                }
            }
        }

        Framebuffer fa, fb;
        fb_render(&fa, g_keyed, nk);
        fb_render(&fb, g_manual, nm);
        if (memcmp(&fa, &fb, sizeof fa) != 0) pixel_ok = 0;
    }
    check("key_order_matches_manual", order_ok);
    check("keys_sorted_ascending", sorted_ok);
    check("collision_keeps_submission_order", stable_ok);
    check("collisions_exercised", collisions > 0);
    check("pixel_identical_to_manual", pixel_ok);
}

static void stability_case(void) {
    World w;
    world_init(&w);
    for (int i = 0; i < 16; i++)
        spawn_at(&w, i, (float)(4 * i), 8.0f, 2.0f, 1.5f, 2, BLEND_ALPHA,
                 ((unsigned)(i * 17) << 24) | 0xFFu);
    int n = drawlist_build(&w, g_keyed, 1);
    drawlist_sort(g_keyed, n);
    int ok = n == 16;
    for (int i = 0; i < n; i++) if (g_keyed[i].seq != i) ok = 0;
    check("equal_keys_keep_pool_order", ok);
}

static void linear_keygen_case(void) {
    static const int sizes[] = { 1, 16, 128, 1024, MAX_PARTICLES };
    int ok = 1;
    for (unsigned s = 0; s < sizeof sizes / sizeof sizes[0]; s++) {
        World w;
        world_init(&w);
        for (int i = 0; i < sizes[s]; i++)
            spawn_at(&w, 1, 8.0f, 8.0f, 2.0f, 1.0f, 0, BLEND_ALPHA, 0xffffff80u);
        keygen_calls = 0;
        int n = drawlist_build(&w, g_keyed, 1);
        if (n != sizes[s] || keygen_calls != (long)n) ok = 0;
    }
    check("keygen_linear_in_particles", ok);
}

static void key_field_case(void) {
    check("key_layer_major",
          make_draw_key(0, 1e9f, BLEND_ADDITIVE) < make_draw_key(1, 0.0f, BLEND_OPAQUE));
    check("key_depth_far_first",
          make_draw_key(2, 2.0f, BLEND_OPAQUE) < make_draw_key(2, 1.0f, BLEND_ADDITIVE));
    check("key_depth_beats_blend",
          make_draw_key(2, 2.0f, BLEND_ADDITIVE) < make_draw_key(2, 1.0f, BLEND_OPAQUE));
    check("key_blend_minor",
          make_draw_key(2, 2.0f, BLEND_OPAQUE) < make_draw_key(2, 2.0f, BLEND_ALPHA));
    check("key_blend_additive_last",
          make_draw_key(2, 2.0f, BLEND_ALPHA) < make_draw_key(2, 2.0f, BLEND_ADDITIVE));
    check("key_equal_for_equal_fields",
          make_draw_key(1, 2.0f, BLEND_ALPHA) == make_draw_key(1, 2.0f, BLEND_ALPHA));
    check("depth_nan_nearest",
          make_draw_key(0, NAN, 0) == make_draw_key(0, 0.0f, 0));
    check("depth_negative_nearest",
          make_draw_key(0, -4.0f, 0) == make_draw_key(0, 0.0f, 0));
    check("depth_inf_farthest",
          make_draw_key(0, INFINITY, 0) < make_draw_key(0, 1e30f, 0));
}

/* Sanity check that the rasterizer actually depends on order, so the pixel
   comparison above is not vacuous. */
static void renderer_sensitivity_case(void) {
    DrawItem red, blue;
    memset(&red, 0, sizeof red);
    memset(&blue, 0, sizeof blue);
    red.x0 = 16; red.y0 = 16; red.x1 = 47; red.y1 = 47;
    blue = red;
    red.rgba = 0xff000080u;  red.blend = BLEND_ALPHA;
    blue.rgba = 0x0000ff80u; blue.blend = BLEND_ALPHA;
    DrawItem ab[2] = { red, blue };
    DrawItem ba[2] = { blue, red };
    Framebuffer fa, fb;
    fb_render(&fa, ab, 2);
    fb_render(&fb, ba, 2);
    check("renderer_order_sensitive", memcmp(&fa, &fb, sizeof fa) != 0);
}

int main(void) {
    World w; world_init(&w);
    emitter_add(&w, 1, 40, 2);
    emitter_add(&w, 2, 30, 9);
    emitter_add(&w, 3, 20, 5);
    emitter_visual(&w, 2, 3.0f, 0, BLEND_ALPHA, 0x42a5f5ffu);   /* far  */
    emitter_visual(&w, 3, 1.0f, 0, BLEND_ALPHA, 0x66bb6affu);   /* near */
    for (int frame = 0; frame < 10; frame++) {
        spawn_frame(&w, 50);
        update(&w, 0.1f);
    }
    printf("alive=%d high_water=%d\n", w.alive_count, w.high_water);
    int tag1 = 0, tag2 = 0, tag3 = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (w.pool[i].alive) {
        if (w.pool[i].tag == 1) tag1++; else if (w.pool[i].tag == 2) tag2++; else tag3++;
    }
    printf("by_emitter=%d,%d,%d\n", tag1, tag2, tag3);
    check("priority_respected", tag2 >= tag3 && tag3 >= 0);

    /* the demo pool's mixed list must come out back-to-front and sorted */
    int dn = world_drawlist(&w, g_keyed);
    int sorted = dn == w.alive_count;
    for (int i = 1; i < dn; i++) if (g_keyed[i - 1].key > g_keyed[i].key) sorted = 0;
    int far_first = dn > 0 && g_keyed[0].depth == 3.0f && g_keyed[dn - 1].depth == 1.0f;
    check("demo_drawlist_sorted", sorted);
    check("demo_far_to_near", far_first);

    key_field_case();
    stability_case();
    linear_keygen_case();
    renderer_sensitivity_case();
    randomized_consistency_case();

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
