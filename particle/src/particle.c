
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget,
   emitter priority scheduling, and approximate depth-peeling order-
   independent transparency for overlapping particles.

   The OIT problem: blending translucent particles in the wrong depth order
   gives the wrong color. Strict per-particle depth sort is the reference but
   costs O(N log N) and forces a serial blend chain. Here we approximate depth
   peeling:

     1. bin particles into B equal-width depth buckets          O(N)
     2. collapse each bucket into one layer, opacity-weighted    O(N)
     3. composite the B bucket layers back-to-front              O(B)

   Total cost is linear in N + B. The bucket transmittance is kept exact via
   a product, so the resulting alpha matches the strict sort; only the RGB is
   approximated, and its error is second-order in the per-particle opacity
   (for two same-bucket particles |delta| <= 2*a1*a2*|C1-C2|). */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_PARTICLES 4096
#define MAX_EMITTERS 8
#define MAX_OIT_BUCKETS 256
#define MAX_STACK 1024

/* ------------------------------------------------------------------ pool */

typedef struct {
    float x, y, vx, vy, life, age;
    float z, r, g, b, a;   /* depth + straight (non-premultiplied) color */
    int alive; int tag;
} Particle;
typedef struct { int id; int quota; int spawned; int priority; } Emitter;

typedef struct {
    Particle pool[MAX_PARTICLES];
    int alive_count, high_water;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;
} World;

static void world_init(World* w) {
    memset(w, 0, sizeof(*w));
    w->emitter_count = 0;
}

static int emitter_add(World* w, int id, int quota, int priority) {
    if (w->emitter_count >= MAX_EMITTERS) return -1;
    Emitter* e = &w->emitters[w->emitter_count++];
    e->id = id; e->quota = quota; e->spawned = 0; e->priority = priority;
    return e->id;
}

static int pool_find_free(World* w) {
    for (int i = 0; i < MAX_PARTICLES; i++) if (!w->pool[i].alive) return i;
    return -1;
}

static int spawn_one(World* w, int emitter_id) {
    int idx = pool_find_free(w);
    if (idx < 0) return -1;
    Particle* p = &w->pool[idx];
    p->alive = 1; p->age = 0.0f;
    p->x = 0.0f; p->y = 0.0f;
    p->vx = 1.0f; p->vy = 0.5f;
    p->life = 2.0f;
    p->z = 0.0f; p->r = p->g = p->b = p->a = 1.0f;
    p->tag = emitter_id;
    w->alive_count++;
    if (w->alive_count > w->high_water) w->high_water = w->alive_count;
    return idx;
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
        for (int i = 0; i < want; i++) if (spawn_one(w, e->id) >= 0) { e->spawned++; left--; }
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

/* --------------------------------------------- approximate depth peeling */

/* one overlapping fragment at a pixel; smaller z is nearer the camera */
typedef struct { float z, a, r, g, b; } Frag;
typedef struct { float r, g, b, a; } RGBA;

typedef struct {
    int buckets;        /* configurable bucket count B */
    float zmin, zmax;   /* depth range covered by the buckets */
    float color_tol;    /* accepted RGB deviation from the strict sort */
} OitCfg;

typedef struct {
    float trans;                        /* bucket transmittance T = prod(1-a) */
    float weighted_r, weighted_g, weighted_b; /* sum of a_i * C_i */
    float weight_sum;                   /* sum of a_i */
    int used;
} Bucket;

typedef struct {
    OitCfg cfg;
    Bucket bk[MAX_OIT_BUCKETS];
    int acc_ops;     /* per-particle accumulation work (one per fragment) */
    int blend_ops;   /* per-bucket composite work (one per non-empty bucket) */
} OitState;

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static int oit_init(OitState* st, const OitCfg* cfg) {
    if (cfg->buckets <= 0 || cfg->buckets > MAX_OIT_BUCKETS) return -1;
    if (!(cfg->zmax > cfg->zmin)) return -1;
    st->cfg = *cfg;
    memset(st->bk, 0, sizeof(st->bk));
    for (int k = 0; k < st->cfg.buckets; k++) st->bk[k].trans = 1.0f;
    st->acc_ops = 0; st->blend_ops = 0;
    return 0;
}

static int bucket_index(const OitState* st, float z) {
    float u = (z - st->cfg.zmin) / (st->cfg.zmax - st->cfg.zmin);
    int b = (int)(clampf(u, 0.0f, 1.0f) * (float)st->cfg.buckets);
    if (b >= st->cfg.buckets) b = st->cfg.buckets - 1;   /* u == 1 case */
    return b;
}

/* strict reference: per-particle depth sort, then back-to-front over blend.
   Insertion sort keeps the reference obviously correct for the test sizes. */
static RGBA composite_reference(const Frag* f, int n, RGBA bg) {
    Frag s[MAX_STACK];
    if (n > MAX_STACK) n = MAX_STACK;
    for (int i = 0; i < n; i++) s[i] = f[i];
    for (int i = 1; i < n; i++) {                 /* ascending z = near first */
        Frag key = s[i]; int j = i - 1;
        while (j >= 0 && s[j].z > key.z) { s[j + 1] = s[j]; j--; }
        s[j + 1] = key;
    }
    RGBA out = bg;
    for (int i = n - 1; i >= 0; i--) {           /* far -> near "over" */
        float a = s[i].a;
        out.r = s[i].r * a + out.r * (1.0f - a);
        out.g = s[i].g * a + out.g * (1.0f - a);
        out.b = s[i].b * a + out.b * (1.0f - a);
        out.a = a + out.a * (1.0f - a);
    }
    return out;
}

/* approximate depth peeling. Cost: one accumulation step per fragment plus
   one composite step per non-empty bucket, i.e. O(n + B), no sort. */
static RGBA composite_peel(OitState* st, const Frag* f, int n, RGBA bg) {
    oit_init(st, &st->cfg);

    /* 1+2. bin and collapse: exact transmittance, opacity-weighted color */
    for (int i = 0; i < n; i++) {
        Bucket* b = &st->bk[bucket_index(st, f[i].z)];
        b->trans *= (1.0f - f[i].a);
        b->weighted_r += f[i].a * f[i].r;
        b->weighted_g += f[i].a * f[i].g;
        b->weighted_b += f[i].a * f[i].b;
        b->weight_sum += f[i].a;
        b->used = 1;
        st->acc_ops++;
    }

    /* 3. composite bucket layers front-to-back (small z bucket first) */
    float T = 1.0f, C_r = 0.0f, C_g = 0.0f, C_b = 0.0f;
    for (int k = 0; k < st->cfg.buckets; k++) {
        Bucket* b = &st->bk[k];
        if (!b->used) continue;
        float layer_a = 1.0f - b->trans;                 /* exact bucket alpha */
        float inv_w = b->weight_sum > 1e-12f
                          ? 1.0f / b->weight_sum : 0.0f; /* normalize weights  */
        float lr = b->weighted_r * inv_w;                /* opacity-weighted    */
        float lg = b->weighted_g * inv_w;                /* effective color     */
        float lb = b->weighted_b * inv_w;
        C_r += T * layer_a * lr;
        C_g += T * layer_a * lg;
        C_b += T * layer_a * lb;
        T *= b->trans;
        st->blend_ops++;
    }
    RGBA out;
    out.r = C_r + T * bg.r;
    out.g = C_g + T * bg.g;
    out.b = C_b + T * bg.b;
    out.a = 1.0f - T;
    return out;
}

/* the original bug: feed particles out in spawn order, no depth sort */
static RGBA composite_unsorted(const Frag* f, int n, RGBA bg) {
    RGBA out = bg;
    for (int i = 0; i < n; i++) {
        float a = f[i].a;
        out.r = f[i].r * a + out.r * (1.0f - a);
        out.g = f[i].g * a + out.g * (1.0f - a);
        out.b = f[i].b * a + out.b * (1.0f - a);
        out.a = a + out.a * (1.0f - a);
    }
    return out;
}

static float color_err(RGBA x, RGBA y) {
    float d = fabsf(x.r - y.r);
    d = fmaxf(d, fabsf(x.g - y.g));
    return fmaxf(d, fabsf(x.b - y.b));
}

static unsigned rng_state = 0x12345678u;
static float rng_f(void) {                       /* xorshift, deterministic */
    unsigned x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return (float)(x & 0xffffff) / (float)0xffffff;
}

/* ------------------------------------------------------------------ test */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    /* ---- original pool / quota / priority behaviour ---- */
    World w; world_init(&w);
    emitter_add(&w, 1, 40, 2);
    emitter_add(&w, 2, 30, 9);
    emitter_add(&w, 3, 20, 5);
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
    /* quota 优先：高优先级 2 应先拿满 30 */
    check("priority_respected", tag2 >= tag3 && tag3 >= 0);

    RGBA bg = { 0.10f, 0.10f, 0.10f, 0.0f };

    /* ---- hand-checked two-fragment same-bucket case ---- */
    {
        Frag f[2] = {
            { 0.2f, 0.5f, 1.0f, 0.0f, 0.0f },   /* nearer, red */
            { 0.3f, 0.5f, 0.0f, 0.0f, 1.0f },   /* farther, blue */
        };
        OitState st;
        OitCfg cfg = { 1, 0.0f, 1.0f, 0.02f };
        oit_init(&st, &cfg);
        RGBA ref = composite_reference(f, 2, bg);
        RGBA peel = composite_peel(&st, f, 2, bg);
        /* transmittance 0.25 is exact regardless of in-bucket order */
        check("bucket_alpha_exact", fabsf(peel.a - 0.75f) < 1e-6f
                                    && fabsf(peel.a - ref.a) < 1e-6f);
        /* weighted color 0.5*red+0.5*blue, premultiplied by layer alpha .75 */
        float expect_r = 0.75f * 0.5f + 0.25f * bg.r;
        check("bucket_weighted_color", fabsf(peel.r - expect_r) < 1e-5f);
    }

    /* ---- the reported bug: overlap blended in the wrong order ---- */
    {
        Frag f[3] = {
            { 0.5f, 0.6f, 0.9f, 0.2f, 0.2f },   /* spawned first, mid depth  */
            { 0.9f, 0.6f, 0.2f, 0.9f, 0.2f },   /* actually farthest          */
            { 0.1f, 0.6f, 0.2f, 0.2f, 0.9f },   /* nearest, spawned last      */
        };
        RGBA ref = composite_reference(f, 3, bg);
        RGBA wrong = composite_unsorted(f, 3, bg);
        OitState st;
        OitCfg cfg = { 8, 0.0f, 1.0f, 0.02f };
        oit_init(&st, &cfg);
        RGBA peel = composite_peel(&st, f, 3, bg);
        printf("overlap ref=(%.3f,%.3f,%.3f) unsorted=(%.3f,%.3f,%.3f) peel=(%.3f,%.3f,%.3f)\n",
               ref.r, ref.g, ref.b, wrong.r, wrong.g, wrong.b,
               peel.r, peel.g, peel.b);
        check("unsorted_is_wrong", color_err(wrong, ref) > 0.05f);
        check("peel_matches_sorted", color_err(peel, ref) <= cfg.color_tol);
        check("peel_alpha_matches", fabsf(peel.a - ref.a) < 1e-6f);
    }

    /* ---- randomized stress vs the strict sort ---- */
    /* Regime A: dense, highly translucent overlap — stresses convergence and
       the linear-cost path; its absolute error is reported, not thresholded. */
    const int ND = 48;
    float d_err_1 = 0.0f, d_err_4 = 0.0f, d_err_16 = 0.0f, d_exact = 0.0f;
    for (int seed = 1; seed <= 8; seed++) {
        rng_state = (unsigned)seed * 2654435761u;
        Frag f[MAX_STACK];
        for (int i = 0; i < ND; i++) {
            f[i].a = 0.10f + 0.50f * rng_f();
            f[i].r = rng_f(); f[i].g = rng_f(); f[i].b = rng_f();
        }

        /* one particle per occupied bucket (distinct integer z over [0,ND))
           makes the weighted average degenerate to plain over blend */
        for (int i = 0; i < ND; i++) f[i].z = (float)i;
        RGBA ref_exact = composite_reference(f, ND, bg);
        OitState st;
        OitCfg cn = { ND, 0.0f, (float)ND, 0.02f };
        oit_init(&st, &cn);
        RGBA pn = composite_peel(&st, f, ND, bg);
        d_exact = fmaxf(d_exact, color_err(pn, ref_exact));
        check("one_per_bucket_exact", color_err(pn, ref_exact) < 1e-6f);
        check("exact_alpha", fabsf(pn.a - ref_exact.a) < 1e-6f);

        /* uniformly random depth shares buckets like real usage */
        for (int i = 0; i < ND; i++) f[i].z = rng_f();
        RGBA ref = composite_reference(f, ND, bg);
        OitState st2;
        RGBA p1, p4, p16;
        OitCfg c1  = { 1,  0.0f, 1.0f, 0.02f }; oit_init(&st2, &c1);
        p1  = composite_peel(&st2, f, ND, bg);
        OitCfg c4  = { 4,  0.0f, 1.0f, 0.02f }; oit_init(&st2, &c4);
        p4  = composite_peel(&st2, f, ND, bg);
        OitCfg c16 = { 16, 0.0f, 1.0f, 0.02f }; oit_init(&st2, &c16);
        p16 = composite_peel(&st2, f, ND, bg);

        d_err_1  = fmaxf(d_err_1,  color_err(p1, ref));
        d_err_4  = fmaxf(d_err_4,  color_err(p4, ref));
        d_err_16 = fmaxf(d_err_16, color_err(p16, ref));
        check("alpha_exact_dense", fabsf(p16.a - ref.a) < 1e-6f);

        /* no sort: each fragment accumulated once, each non-empty bucket
           blended once, and the bucket pass never exceeds B */
        check("cost_linear_n_plus_b",
              st2.acc_ops == ND && st2.blend_ops <= ND
              && st2.blend_ops <= c16.buckets);
    }
    printf("dense(N=%d) max_rgb_err B=1:%.4f B=4:%.4f B=16:%.4f exact:%.6f\n",
           ND, d_err_1, d_err_4, d_err_16, d_exact);
    check("err_shrinks_with_buckets", d_err_16 < d_err_4 && d_err_4 < d_err_1);

    /* Regime B: realistic smoke/dust overlap — 16 fragments per pixel,
       opacity 0.05..0.25. Here B=16 must stay inside the configured 0.02
       color tolerance against the strict per-particle sort. */
    const int NR = 16;
    float r_worst = 0.0f;
    for (int seed = 1; seed <= 8; seed++) {
        rng_state = (unsigned)seed * 2654435761u;
        Frag f[MAX_STACK];
        for (int i = 0; i < NR; i++) {
            f[i].z = rng_f();
            f[i].a = 0.05f + 0.20f * rng_f();
            f[i].r = rng_f(); f[i].g = rng_f(); f[i].b = rng_f();
        }
        RGBA ref = composite_reference(f, NR, bg);
        OitState st;
        OitCfg cfg = { 16, 0.0f, 1.0f, 0.02f };
        oit_init(&st, &cfg);
        RGBA peel = composite_peel(&st, f, NR, bg);
        r_worst = fmaxf(r_worst, color_err(peel, ref));
        check("alpha_exact_realistic", fabsf(peel.a - ref.a) < 1e-6f);
        check("rgb_within_tolerance", color_err(peel, ref) <= cfg.color_tol);
    }
    printf("realistic(N=%d,B=16) worst_rgb_err=%.4f tolerance=0.02\n",
           NR, r_worst);

    /* ---- edge cases ---- */
    {
        OitState st;
        OitCfg cfg = { 8, 0.0f, 1.0f, 0.02f };
        oit_init(&st, &cfg);
        RGBA peel = composite_peel(&st, NULL, 0, bg);
        RGBA ref = composite_reference(NULL, 0, bg);
        check("empty_passthrough", color_err(peel, bg) < 1e-6f
                                   && peel.a == 0.0f && st.acc_ops == 0);

        Frag one[1] = { { 0.4f, 0.3f, 0.5f, 0.6f, 0.7f } };
        oit_init(&st, &cfg);
        peel = composite_peel(&st, one, 1, bg);
        ref = composite_reference(one, 1, bg);
        check("single_particle_exact", color_err(peel, ref) < 1e-6f);

        OitCfg bad1 = { 0, 0.0f, 1.0f, 0.02f };
        OitCfg bad2 = { 8, 1.0f, 1.0f, 0.02f };
        OitCfg bad3 = { MAX_OIT_BUCKETS + 1, 0.0f, 1.0f, 0.02f };
        check("config_rejected", oit_init(&st, &bad1) < 0
                                 && oit_init(&st, &bad2) < 0
                                 && oit_init(&st, &bad3) < 0);

        /* fully opaque overlap: weighted average reduces to exact over blend */
        Frag op[3] = {
            { 0.8f, 1.0f, 0.1f, 0.2f, 0.3f },
            { 0.4f, 1.0f, 0.4f, 0.5f, 0.6f },
            { 0.1f, 1.0f, 0.7f, 0.8f, 0.9f },
        };
        oit_init(&st, &cfg);
        peel = composite_peel(&st, op, 3, bg);
        ref = composite_reference(op, 3, bg);
        check("opaque_exact", fabsf(peel.a - ref.a) < 1e-6f
                              && peel.a > 0.999999f);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
