/* matbatch: material-state batching for draw lists.
   - groups draw items by material key (hash) with full-param compare, so
     key collisions split into separate groups instead of corrupting state
   - identical parameters are written once (state cache, field-level dedup)
   - software z-buffer renderer proves the batched output is pixel-identical
     to naive per-object state setting. Two passes like a real engine:
     opaque (z-test + z-write, freely reorderable) then additive (z-test,
     no z-write, commutative) — reordering inside a pass cannot change pixels
   - grouping is O(N + M): N items, M distinct materials */

#define _POSIX_C_SOURCE 199309L /* clock_gettime */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define FB_W 64
#define FB_H 64
#define NFIELDS 7 /* r,g,b,a,texture,shader,blend */

typedef struct {
    float r, g, b, a;
    int texture_id;
    int shader_id;
    int blend; /* 0 = opaque, 1 = additive (both order-independent) */
} MatParams;

typedef struct {
    MatParams mat;
    unsigned key; /* caller-supplied material key; may collide */
    int x, y, w, h;
    double z;     /* unique per item -> draw order irrelevant under z-test */
} DrawItem;

static int mat_equal(const MatParams* x, const MatParams* y) {
    return x->r == y->r && x->g == y->g && x->b == y->b && x->a == y->a &&
           x->texture_id == y->texture_id && x->shader_id == y->shader_id &&
           x->blend == y->blend;
}

static unsigned mat_hash(const MatParams* m) {
    const unsigned char* p = (const unsigned char*)m;
    unsigned h = 2166136261u;
    for (size_t i = 0; i < sizeof(*m); i++) { h ^= p[i]; h *= 16777619u; }
    return h;
}

/* ---------------- grouper: open-addressing map (key,params) -> group ------- */

typedef struct {
    MatParams mat;
    unsigned key;
    int head, tail; /* item-index linked list, submission order */
    int count;
} Group;

typedef struct {
    int* slots;    /* group index + 1, 0 = empty */
    int cap;       /* power of two */
    Group* groups;
    int ngroups, gcap;
    int* next;     /* per item: next item index in its group, -1 = tail */
    long probes;   /* slot examinations, grouping-cost metric */
} Grouper;

static void grouper_init(Grouper* g, int max_items) {
    g->cap = 16;
    while (g->cap < max_items * 2) g->cap *= 2;
    g->slots = calloc((size_t)g->cap, sizeof(int));
    g->gcap = 16;
    g->groups = malloc((size_t)g->gcap * sizeof(Group));
    g->ngroups = 0;
    g->next = malloc((size_t)max_items * sizeof(int));
    g->probes = 0;
}

static void grouper_free(Grouper* g) {
    free(g->slots);
    free(g->groups);
    free(g->next);
}

static void grouper_add(Grouper* g, const DrawItem* it, int idx) {
    unsigned mask = (unsigned)g->cap - 1;
    unsigned s = it->key & mask;
    for (;;) {
        g->probes++;
        int slot = g->slots[s];
        if (slot == 0) { /* empty: start a new group */
            if (g->ngroups == g->gcap) {
                g->gcap *= 2;
                g->groups = realloc(g->groups, (size_t)g->gcap * sizeof(Group));
            }
            int gi = g->ngroups++;
            g->groups[gi].mat = it->mat;
            g->groups[gi].key = it->key;
            g->groups[gi].head = g->groups[gi].tail = idx;
            g->groups[gi].count = 1;
            g->slots[s] = gi + 1;
            g->next[idx] = -1;
            return;
        }
        Group* gr = &g->groups[slot - 1];
        if (gr->key == it->key && mat_equal(&gr->mat, &it->mat)) {
            /* same key AND same params: merge; a key match with different
               params falls through and probes on -> collision split */
            g->next[gr->tail] = idx;
            gr->tail = idx;
            g->next[idx] = -1;
            gr->count++;
            return;
        }
        s = (s + 1) & mask;
    }
}

/* ---------------- simulated render device with state cache ---------------- */

static unsigned char fb[FB_H][FB_W][3];
static double zb[FB_H][FB_W];

static MatParams cur_state;
static int cur_valid;
static long param_sets;  /* individual parameter writes */
static long mat_applies; /* material bind attempts */

static void device_reset_stats(void) {
    param_sets = 0;
    mat_applies = 0;
    cur_valid = 0;
}

static void apply_material(const MatParams* m, int use_cache) {
    mat_applies++;
    if (use_cache && cur_valid) {
        if (cur_state.r != m->r) { cur_state.r = m->r; param_sets++; }
        if (cur_state.g != m->g) { cur_state.g = m->g; param_sets++; }
        if (cur_state.b != m->b) { cur_state.b = m->b; param_sets++; }
        if (cur_state.a != m->a) { cur_state.a = m->a; param_sets++; }
        if (cur_state.texture_id != m->texture_id) { cur_state.texture_id = m->texture_id; param_sets++; }
        if (cur_state.shader_id != m->shader_id) { cur_state.shader_id = m->shader_id; param_sets++; }
        if (cur_state.blend != m->blend) { cur_state.blend = m->blend; param_sets++; }
    } else {
        cur_state = *m;
        cur_valid = 1;
        param_sets += NFIELDS;
    }
}

static int clamp255(int v) { return v < 0 ? 0 : v > 255 ? 255 : v; }

static void shade(const MatParams* m, unsigned char c[3]) {
    int r = (int)(m->r * 255.0f) + m->texture_id * 13 + m->shader_id * 7;
    int g = (int)(m->g * 255.0f) + m->texture_id * 29 + m->shader_id * 3;
    int b = (int)(m->b * 255.0f) + m->texture_id * 7 + m->shader_id * 31;
    c[0] = (unsigned char)clamp255(r * (int)(m->a * 100) / 100);
    c[1] = (unsigned char)clamp255(g * (int)(m->a * 100) / 100);
    c[2] = (unsigned char)clamp255(b * (int)(m->a * 100) / 100);
}

static void draw_rect(const DrawItem* it) {
    unsigned char c[3];
    shade(&it->mat, c);
    int x1 = it->x + it->w, y1 = it->y + it->h;
    if (x1 > FB_W) x1 = FB_W;
    if (y1 > FB_H) y1 = FB_H;
    for (int y = it->y; y < y1; y++)
        for (int x = it->x; x < x1; x++) {
            if (it->z >= zb[y][x]) continue;
            if (it->mat.blend == 0) {
                zb[y][x] = it->z; /* only opaque writes depth */
                fb[y][x][0] = c[0]; fb[y][x][1] = c[1]; fb[y][x][2] = c[2];
            } else {
                fb[y][x][0] = (unsigned char)clamp255(fb[y][x][0] + c[0]);
                fb[y][x][1] = (unsigned char)clamp255(fb[y][x][1] + c[1]);
                fb[y][x][2] = (unsigned char)clamp255(fb[y][x][2] + c[2]);
            }
        }
}

static void fb_clear(void) {
    memset(fb, 0, sizeof(fb));
    for (int y = 0; y < FB_H; y++)
        for (int x = 0; x < FB_W; x++) zb[y][x] = 1e30;
}

/* naive: two passes, set every parameter of every item, no cache */
static void render_naive(const DrawItem* items, int n) {
    device_reset_stats();
    for (int pass = 0; pass < 2; pass++)
        for (int i = 0; i < n; i++) {
            if (items[i].mat.blend != pass) continue;
            apply_material(&items[i].mat, 0);
            draw_rect(&items[i]);
        }
}

/* batched: group by material, set state once per group (with cache);
   opaque groups emit before additive groups (pass order preserved) */
static long render_batched(const DrawItem* items, int n, Grouper* g) {
    grouper_init(g, n);
    for (int i = 0; i < n; i++) grouper_add(g, &items[i], i);
    device_reset_stats();
    for (int pass = 0; pass < 2; pass++)
        for (int gi = 0; gi < g->ngroups; gi++) {
            Group* gr = &g->groups[gi];
            if (gr->mat.blend != pass) continue;
            apply_material(&gr->mat, 1);
            for (int idx = gr->head; idx >= 0; idx = g->next[idx])
                draw_rect(&items[idx]);
        }
    return g->probes;
}

/* ---------------- scene generation ---------------- */

static unsigned rng_state = 12345;
static unsigned rng_next(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

static void gen_materials(MatParams* mats, int m) {
    for (int i = 0; i < m; i++) {
        mats[i].r = (float)(rng_next() % 100) / 100.0f;
        mats[i].g = (float)(rng_next() % 100) / 100.0f;
        mats[i].b = (float)(rng_next() % 100) / 100.0f;
        mats[i].a = 1.0f;
        mats[i].texture_id = (int)(rng_next() % 8);
        mats[i].shader_id = (int)(rng_next() % 4);
        mats[i].blend = (int)(rng_next() % 2);
    }
}

static void gen_items(DrawItem* items, int n, const MatParams* mats, int m,
                      const unsigned* keys) {
    for (int i = 0; i < n; i++) {
        int mi = (int)(rng_next() % (unsigned)m);
        items[i].mat = mats[mi];
        items[i].key = keys[mi];
        items[i].x = (int)(rng_next() % (FB_W - 8));
        items[i].y = (int)(rng_next() % (FB_H - 8));
        items[i].w = 4 + (int)(rng_next() % 9);
        items[i].h = 4 + (int)(rng_next() % 9);
        items[i].z = (double)i / (double)n; /* unique per item */
    }
}

/* ---------------- tests ---------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

int main(void) {
    /* 1. grouping: N items over M materials -> exactly M groups, stable order */
    {
        enum { N = 1000, M = 8 };
        MatParams mats[M];
        unsigned keys[M];
        DrawItem* items = malloc(N * sizeof(DrawItem));
        gen_materials(mats, M);
        for (int i = 0; i < M; i++) keys[i] = mat_hash(&mats[i]);
        gen_items(items, N, mats, M, keys);
        Grouper g;
        render_batched(items, N, &g);
        int total = 0, ordered = 1;
        for (int gi = 0; gi < g.ngroups; gi++) {
            total += g.groups[gi].count;
            for (int idx = g.groups[gi].head; g.next[idx] >= 0; idx = g.next[idx])
                if (g.next[idx] < idx) ordered = 0;
        }
        check("group_count", g.ngroups == M);
        check("group_total", total == N);
        check("group_stable_order", ordered);
        grouper_free(&g);
        free(items);
    }

    /* 2. key collision: same key, different params -> split into 2 groups */
    {
        enum { N = 200 };
        MatParams mats[2];
        unsigned keys[2] = { 42, 42 }; /* forced collision */
        DrawItem* items = malloc(N * sizeof(DrawItem));
        gen_materials(mats, 2);
        gen_items(items, N, mats, 2, keys);
        Grouper g;
        render_batched(items, N, &g);
        check("collision_splits", g.ngroups == 2);
        check("collision_params_kept",
              (mat_equal(&g.groups[0].mat, &mats[0]) && mat_equal(&g.groups[1].mat, &mats[1])) ||
              (mat_equal(&g.groups[0].mat, &mats[1]) && mat_equal(&g.groups[1].mat, &mats[0])));
        grouper_free(&g);
        free(items);
    }

    /* 3. pixel-identical: batched framebuffer == naive framebuffer */
    {
        enum { N = 5000, M = 37 };
        MatParams mats[M];
        unsigned keys[M];
        DrawItem* items = malloc(N * sizeof(DrawItem));
        gen_materials(mats, M);
        for (int i = 0; i < M; i++) keys[i] = mat_hash(&mats[i]);
        keys[1] = keys[0]; /* inject a real key collision into the scene */
        keys[2] = keys[0];
        gen_items(items, N, mats, M, keys);

        static unsigned char fb_naive[FB_H][FB_W][3];
        fb_clear();
        render_naive(items, N);
        memcpy(fb_naive, fb, sizeof(fb));
        long naive_sets = param_sets, naive_applies = mat_applies;

        fb_clear();
        Grouper g;
        render_batched(items, N, &g);
        long batched_sets = param_sets, batched_applies = mat_applies;

        check("pixel_identical", memcmp(fb_naive, fb, sizeof(fb)) == 0);
        check("state_sets_reduced", batched_sets < naive_sets);
        printf("naive_applies=%ld batched_applies=%ld\n", naive_applies, batched_applies);
        printf("naive_param_sets=%ld batched_param_sets=%ld reduction=%.1f%%\n",
               naive_sets, batched_sets, 100.0 * (1.0 - (double)batched_sets / naive_sets));
        grouper_free(&g);
        free(items);
    }

    /* 4. same params set once: two keys, identical params -> cache still
           writes only one material's worth of fields */
    {
        enum { N = 400 };
        MatParams mats[2];
        unsigned keys[2] = { 7, 9 }; /* different keys... */
        DrawItem* items = malloc(N * sizeof(DrawItem));
        gen_materials(mats, 2);
        mats[1] = mats[0];           /* ...but identical params */
        gen_items(items, N, mats, 2, keys);
        fb_clear();
        Grouper g;
        render_batched(items, N, &g);
        check("dup_params_two_groups", g.ngroups == 2);
        check("dup_params_set_once", param_sets == NFIELDS);
        printf("dup_params_applies=%ld dup_params_param_sets=%ld\n", mat_applies, param_sets);
        grouper_free(&g);
        free(items);
    }

    /* 5. merge cost linear in material count M (N fixed) */
    {
        enum { N = 100000 };
        DrawItem* items = malloc(N * sizeof(DrawItem));
        static const int ms[] = { 16, 64, 256, 1024, 4096 };
        double t0 = 0;
        int linear = 1;
        printf("merge_scaling: N=%d\n", N);
        for (int k = 0; k < 5; k++) {
            int m = ms[k];
            MatParams* mats = malloc((size_t)m * sizeof(MatParams));
            unsigned* keys = malloc((size_t)m * sizeof(unsigned));
            gen_materials(mats, m);
            for (int i = 0; i < m; i++) keys[i] = mat_hash(&mats[i]);
            gen_items(items, N, mats, m, keys);
            Grouper g;
            double a = now_ms();
            long probes = render_batched(items, N, &g);
            double dt = now_ms() - a;
            printf("  M=%4d groups=%d probes=%ld time=%.2fms ns_per_item=%.1f\n",
                   m, g.ngroups, probes, dt, dt * 1e6 / N);
            /* O(N+M): probes bounded by N plus a small multiple of M */
            if (probes > N + 4L * m) linear = 0;
            if (g.ngroups != m) linear = 0;
            if (k == 0) t0 = dt;
            (void)t0;
            grouper_free(&g);
            free(mats);
            free(keys);
        }
        check("merge_cost_linear_in_M", linear);
        free(items);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
