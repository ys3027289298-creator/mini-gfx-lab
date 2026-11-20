
/* postfx: post-processing chain with dependency-ordered scheduling,
   lifetime-based render-target reuse and a capped buffer pool.

   Two schedulers share one set of pixel kernels:
     naive - every pass allocates a brand-new fullscreen target and nothing is
             recycled until the frame ends (the "one buffer per pass" baseline
             and the pixel reference);
     pool  - targets are leased from a size-keyed pool; an intermediate goes
             back to the pool immediately after the last pass that reads it.
   Both execute identical kernels over identical inputs, so every produced
   resource compares bit-for-bit (memcmp on float pixels). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_RES   64
#define MAX_PASS  48
#define MAX_KIN   3
#define MAX_BUFS  64

enum {
    PF_OK = 0,
    PF_ERR_ARG,     /* bad graph / arguments                      */
    PF_ERR_CYCLE,   /* dependency cycle, no topological order      */
    PF_ERR_SPEC,    /* input/output size mismatch                 */
    PF_ERR_CAP,     /* pool cap would be exceeded                 */
    PF_ERR_NOSLOT   /* device buffer table full                   */
};

enum {
    K_SAT = 0, K_GAMMA, K_CONTRAST, K_INV, K_VIGN, K_BRIGHT,
    K_BLUR, K_DOWNSAMPLE, K_ADD, K_MUL, K_COUNT
};

typedef struct {
    int w, h;           /* size in pixels; 4 floats (RGBA) per pixel */
    int producer;       /* pass id, or -1 for an external resource   */
    int external;
} PFRes;

typedef struct {
    int kernel;
    int n_in;
    int in[MAX_KIN];
    int out;
} PFPass;

typedef struct {
    PFRes res[MAX_RES];
    int n_res;
    PFPass pass[MAX_PASS];
    int n_pass;
} PFGraph;

/* compile output: execution order, per-resource death ordinal and which
   resources are produced inside the frame (and thus pixel-checked). */
typedef struct {
    int order[MAX_PASS];
    int death_order[MAX_RES];  /* pass ordinal after which the res dies, -1 */
    int shot[MAX_RES];         /* 1 = produced by a pass, needs readback    */
    int err;
} PFCompile;

/* ---- device side: a PFBuf stands in for one GPU render target ---------- */

typedef struct {
    int w, h;
    int live;               /* 1 = leased out, 0 = on the pool free list */
    int gen;                /* allocation generation (debug/stats)       */
    float* px;
} PFBuf;

typedef struct {
    int pool_mode;
    int cap;                /* max simultaneously live pool buffers      */
    PFBuf buf[MAX_BUFS];
    int n_buf;
    int alloc_count;        /* backing allocations ever made             */
    int free_count;         /* releases back to pool / teardown          */
    int hits, misses;       /* free-list lease vs fresh allocation       */
    int live_bufs, peak_bufs;
    long long live_bytes, peak_bytes;
    int err;
} PFDevice;

typedef struct {
    int id, w, h, n;
    float* px;              /* host-side readback copy                   */
} PFShot;

static int kernel_arity(int k) {
    return (k == K_ADD || k == K_MUL) ? 2 : 1;
}

/* ---- compile: validate, topologically order, derive lifetimes ---------- */

static int pf_compile(const PFGraph* g, PFCompile* c) {
    memset(c, 0, sizeof(*c));
    for (int r = 0; r < MAX_RES; r++) c->death_order[r] = -1;

    if (g->n_res <= 0 || g->n_res > MAX_RES ||
        g->n_pass <= 0 || g->n_pass > MAX_PASS)
        return (c->err = PF_ERR_ARG);

    int producer[MAX_RES];
    for (int r = 0; r < g->n_res; r++) producer[r] = -1;

    for (int p = 0; p < g->n_pass; p++) {
        const PFPass* pa = &g->pass[p];
        if (pa->kernel < 0 || pa->kernel >= K_COUNT)
            return (c->err = PF_ERR_ARG);
        if (pa->n_in != kernel_arity(pa->kernel))
            return (c->err = PF_ERR_ARG);
        if (pa->out < 0 || pa->out >= g->n_res)
            return (c->err = PF_ERR_ARG);
        if (g->res[pa->out].external || producer[pa->out] != -1)
            return (c->err = PF_ERR_ARG);   /* external or double producer */
        producer[pa->out] = p;

        for (int i = 0; i < pa->n_in; i++) {
            int r = pa->in[i];
            if (r < 0 || r >= g->n_res || r == pa->out)
                return (c->err = PF_ERR_ARG);
            for (int j = 0; j < i; j++)
                if (pa->in[j] == r) return (c->err = PF_ERR_ARG); /* dup */
        }
    }
    for (int r = 0; r < g->n_res; r++) {
        if (g->res[r].external && g->res[r].producer != -1)
            return (c->err = PF_ERR_ARG);
        if (!g->res[r].external && producer[r] != g->res[r].producer)
            return (c->err = PF_ERR_ARG);
    }

    /* size rules: every kernel fully overwrites its output */
    for (int p = 0; p < g->n_pass; p++) {
        const PFPass* pa = &g->pass[p];
        const PFRes* o = &g->res[pa->out];
        const PFRes* a = &g->res[pa->in[0]];
        if (pa->kernel == K_DOWNSAMPLE) {
            if (o->w * 2 != a->w || o->h * 2 != a->h)
                return (c->err = PF_ERR_SPEC);
        } else {
            if (o->w != a->w || o->h != a->h)
                return (c->err = PF_ERR_SPEC);
        }
        for (int i = 1; i < pa->n_in; i++) {
            const PFRes* b = &g->res[pa->in[i]];
            if (b->w != o->w || b->h != o->h)
                return (c->err = PF_ERR_SPEC);
        }
    }

    /* Kahn's algorithm; ties break on pass index for a deterministic order */
    int indeg[MAX_PASS] = {0};
    for (int p = 0; p < g->n_pass; p++) {
        const PFPass* pa = &g->pass[p];
        for (int i = 0; i < pa->n_in; i++) {
            int pr = producer[pa->in[i]];
            if (pr != -1) indeg[p]++;
        }
    }
    int n_order = 0;
    for (;;) {
        int pick = -1;
        for (int p = 0; p < g->n_pass; p++) {
            if (indeg[p] == 0) {
                int already = 0;
                for (int k = 0; k < n_order; k++)
                    if (c->order[k] == p) already = 1;
                if (!already) { pick = p; break; }
            }
        }
        if (pick == -1) break;
        c->order[n_order++] = pick;
        indeg[pick] = -1;
        for (int p = 0; p < g->n_pass; p++) {
            if (indeg[p] <= 0) continue;
            const PFPass* pa = &g->pass[p];
            for (int i = 0; i < pa->n_in; i++)
                if (producer[pa->in[i]] == pick) indeg[p]--;
        }
    }
    if (n_order != g->n_pass)
        return (c->err = PF_ERR_CYCLE);

    int ordinal[MAX_PASS];
    for (int k = 0; k < g->n_pass; k++) ordinal[c->order[k]] = k;

    /* lifetime: resource dies right after its last consumer pass */
    for (int r = 0; r < g->n_res; r++) {
        c->shot[r] = !g->res[r].external;
        int last = -1;
        for (int p = 0; p < g->n_pass; p++) {
            const PFPass* pa = &g->pass[p];
            for (int i = 0; i < pa->n_in; i++)
                if (pa->in[i] == r && ordinal[p] > last) last = ordinal[p];
        }
        if (last == -1 && !g->res[r].external)
            last = ordinal[producer[r]];   /* unconsumed output */
        c->death_order[r] = last;
    }
    return (c->err = PF_OK);
}

/* ---- pixel kernels: pure functions of declared inputs only ------------- */

static inline float clamp01(float v) {
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

static void pf_exec(int kernel, int n_in, PFBuf* const* in, PFBuf* out,
                    float t) {
    const int w = out->w, h = out->h;
    const float* a = in[0]->px;
    const float* b = n_in > 1 ? in[1]->px : NULL;
    float* o = out->px;
    (void)t;

    switch (kernel) {
    case K_SAT:
        for (int i = 0; i < w * h * 4; i += 4) {
            float l = 0.299f * a[i] + 0.587f * a[i + 1] + 0.114f * a[i + 2];
            for (int c = 0; c < 3; c++) o[i + c] = clamp01(l + 1.6f * (a[i + c] - l));
            o[i + 3] = a[i + 3];
        }
        break;
    case K_GAMMA:
        for (int i = 0; i < w * h * 4; i++)
            o[i] = i % 4 == 3 ? a[i] : powf(clamp01(a[i]), 1.0f / 2.2f);
        break;
    case K_CONTRAST:
        for (int i = 0; i < w * h * 4; i++)
            o[i] = i % 4 == 3 ? a[i] : clamp01((a[i] - 0.5f) * 1.25f + 0.5f);
        break;
    case K_INV:
        for (int i = 0; i < w * h * 4; i++)
            o[i] = i % 4 == 3 ? a[i] : 1.0f - a[i];
        break;
    case K_VIGN:
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++) {
                float dx = (x / (float)w - 0.5f) * 2.0f;
                float dy = (y / (float)h - 0.5f) * 2.0f;
                float f = 1.0f - 0.5f * (dx * dx + dy * dy);
                int i = (y * w + x) * 4;
                for (int c = 0; c < 3; c++) o[i + c] = clamp01(a[i + c] * f);
                o[i + 3] = a[i + 3];
            }
        break;
    case K_BRIGHT:
        for (int i = 0; i < w * h * 4; i += 4) {
            float l = 0.299f * a[i] + 0.587f * a[i + 1] + 0.114f * a[i + 2];
            int keep = l > 0.6f;
            for (int c = 0; c < 3; c++) o[i + c] = keep ? a[i + c] : 0.0f;
            o[i + 3] = a[i + 3];
        }
        break;
    case K_BLUR: {
        const int iw = in[0]->w, ih = in[0]->h;
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++) {
                float acc[4] = {0, 0, 0, 0};
                for (int dy = -1; dy <= 1; dy++)
                    for (int dx = -1; dx <= 1; dx++) {
                        int sx = x + dx, sy = y + dy;
                        if (sx < 0) sx = 0; else if (sx >= iw) sx = iw - 1;
                        if (sy < 0) sy = 0; else if (sy >= ih) sy = ih - 1;
                        const float* s = &a[(sy * iw + sx) * 4];
                        for (int c = 0; c < 4; c++) acc[c] += s[c];
                    }
                float* d = &o[(y * w + x) * 4];
                for (int c = 0; c < 4; c++) d[c] = acc[c] / 9.0f;
            }
        break;
    }
    case K_DOWNSAMPLE:
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++) {
                const float* s0 = &a[((y * 2) * in[0]->w + x * 2) * 4];
                const float* s1 = &a[((y * 2 + 1) * in[0]->w + x * 2) * 4];
                float* d = &o[(y * w + x) * 4];
                for (int c = 0; c < 4; c++)
                    d[c] = (s0[c] + s0[c + 4] + s1[c] + s1[c + 4]) * 0.25f;
            }
        break;
    case K_ADD:
        for (int i = 0; i < w * h * 4; i++)
            o[i] = i % 4 == 3 ? a[i] : clamp01(a[i] + b[i]);
        break;
    case K_MUL:
        for (int i = 0; i < w * h * 4; i++)
            o[i] = i % 4 == 3 ? a[i] : clamp01(a[i] * b[i]);
        break;
    default:
        break;
    }
}

/* ---- device: acquire / release / pool accounting ----------------------- */

static long long buf_bytes(const PFBuf* b) {
    return (long long)b->w * b->h * 4 * (long long)sizeof(float);
}

static void pf_init(PFDevice* d, int pool_mode, int cap) {
    memset(d, 0, sizeof(*d));
    d->pool_mode = pool_mode;
    d->cap = cap;
}

static PFBuf* pf_acquire(PFDevice* d, int w, int h) {
    if (d->err) return NULL;
    PFBuf* b = NULL;
    if (d->pool_mode) {
        for (int i = 0; i < d->n_buf; i++) {
            PFBuf* q = &d->buf[i];
            if (!q->live && q->w == w && q->h == h) { b = q; break; }
        }
    }
    if (b) {
        d->hits++;
    } else {
        if (d->pool_mode && d->live_bufs >= d->cap) {
            d->err = PF_ERR_CAP;
            return NULL;
        }
        if (d->n_buf >= MAX_BUFS) {
            d->err = PF_ERR_NOSLOT;
            return NULL;
        }
        b = &d->buf[d->n_buf++];
        b->w = w; b->h = h; b->gen = d->alloc_count + 1;
        b->px = malloc((size_t)w * h * 4 * sizeof(float));
        if (!b->px) { d->err = PF_ERR_NOSLOT; return NULL; }
        d->alloc_count++;
        d->misses++;
    }
    b->live = 1;
    d->live_bufs++;
    d->live_bytes += buf_bytes(b);
    if (d->live_bufs > d->peak_bufs) d->peak_bufs = d->live_bufs;
    if (d->live_bytes > d->peak_bytes) d->peak_bytes = d->live_bytes;
    return b;
}

static void pf_release(PFDevice* d, PFBuf* b, int poison) {
    if (!b->live) return;
    b->live = 0;
    d->live_bufs--;
    d->live_bytes -= buf_bytes(b);
    d->free_count++;
    /* NaN canary: a recycled buffer must never be read before a kernel fully
       overwrites it; stale-read bugs surface as NaNs in the pixel compare. */
    if (poison)
        for (long long i = 0; i < (long long)b->w * b->h * 4; i++)
            b->px[i] = NAN;
}

static void pf_end_frame(PFDevice* d) {
    for (int i = 0; i < d->n_buf; i++)
        if (d->buf[i].live) pf_release(d, &d->buf[i], d->pool_mode);
}

static void pf_destroy(PFDevice* d) {
    for (int i = 0; i < d->n_buf; i++) free(d->buf[i].px);
    d->n_buf = 0;
}

/* ---- frame execution, shared by both schedulers ------------------------ */

static int pf_run(PFDevice* dev, const PFGraph* g, const PFCompile* c,
                  PFBuf* const* scene, int n_scene,
                  PFShot* shots, int* n_shots) {
    PFBuf* bound[MAX_RES] = {0};
    int si = 0;
    for (int r = 0; r < g->n_res; r++) {
        if (g->res[r].external) {
            if (si >= n_scene) return PF_ERR_ARG;
            bound[r] = scene[si++];
        }
    }

    int deaths[MAX_PASS][MAX_RES];
    int dcount[MAX_PASS];
    memset(dcount, 0, sizeof(dcount));
    for (int r = 0; r < g->n_res; r++)
        if (c->death_order[r] >= 0)
            deaths[c->death_order[r]][dcount[c->death_order[r]]++] = r;

    int got[MAX_RES] = {0};
    *n_shots = 0;

    for (int k = 0; k < g->n_pass; k++) {
        const PFPass* pa = &g->pass[c->order[k]];
        PFBuf* in[MAX_KIN];
        for (int i = 0; i < pa->n_in; i++) {
            in[i] = bound[pa->in[i]];
            if (!in[i]) return PF_ERR_ARG;
        }
        PFBuf* out = pf_acquire(dev, g->res[pa->out].w, g->res[pa->out].h);
        if (!out) return dev->err;

        pf_exec(pa->kernel, pa->n_in, in, out, 0.0f);
        bound[pa->out] = out;

        for (int j = 0; j < dcount[k]; j++) {
            int r = deaths[k][j];
            PFBuf* dead = bound[r];
            if (c->shot[r] && !got[r]) {      /* readback before recycle */
                PFShot* s = &shots[(*n_shots)++];
                s->id = r; s->w = dead->w; s->h = dead->h;
                s->n = dead->w * dead->h * 4;
                s->px = malloc((size_t)s->n * sizeof(float));
                memcpy(s->px, dead->px, (size_t)s->n * sizeof(float));
                got[r] = 1;
            }
            bound[r] = NULL;
            /* naive baseline keeps every target resident until the frame
               ends; only the pool scheduler recycles at the death point */
            if (dev->pool_mode && !g->res[r].external)
                pf_release(dev, dead, 1);
        }
    }
    /* final outputs: still live when the frame ends */
    for (int r = 0; r < g->n_res; r++) {
        if (bound[r] && c->shot[r] && !got[r]) {
            PFBuf* dead = bound[r];
            PFShot* s = &shots[(*n_shots)++];
            s->id = r; s->w = dead->w; s->h = dead->h;
            s->n = dead->w * dead->h * 4;
            s->px = malloc((size_t)s->n * sizeof(float));
            memcpy(s->px, dead->px, (size_t)s->n * sizeof(float));
            got[r] = 1;
        }
    }
    return PF_OK;
}

static void shots_free(PFShot* s, int n) {
    for (int i = 0; i < n; i++) free(s[i].px);
}

static int shots_equal(const PFShot* a, int na, const PFShot* b, int nb) {
    if (na != nb) return 0;
    for (int i = 0; i < na; i++) {
        const PFShot* x = &a[i];
        const PFShot* y = NULL;
        for (int j = 0; j < nb; j++)
            if (b[j].id == x->id) { y = &b[j]; break; }
        if (!y || y->w != x->w || y->h != x->h || y->n != x->n) return 0;
        if (memcmp(x->px, y->px, (size_t)x->n * sizeof(float)) != 0) return 0;
    }
    return 1;
}

/* ---- test graph builders ------------------------------------------------ */

#define W 64
#define H 64

static void fill_scene(float* px, int n) {
    for (int i = 0; i < n; i++) {
        float v = sinf((float)(i + 1) * 12.9898f) * 43758.5453f;
        px[i] = v - floorf(v);
    }
}

static PFBuf* make_scene(int w, int h) {
    PFBuf* b = calloc(1, sizeof(PFBuf));
    b->w = w; b->h = h; b->live = 1;
    b->px = malloc((size_t)w * h * 4 * sizeof(float));
    fill_scene(b->px, w * h * 4);
    return b;
}

/* linear chain: res0 external, pass i maps res i -> res i+1 */
static void build_linear(PFGraph* g, int n) {
    memset(g, 0, sizeof(*g));
    g->n_res = n + 1;
    g->n_pass = n;
    g->res[0] = (PFRes){W, H, -1, 1};
    for (int r = 1; r <= n; r++)
        g->res[r] = (PFRes){W, H, r - 1, 0};
    int cycle_kernels[] = {K_SAT, K_GAMMA, K_CONTRAST, K_INV, K_VIGN,
                           K_BRIGHT, K_BLUR};
    for (int p = 0; p < n; p++) {
        g->pass[p] = (PFPass){cycle_kernels[p % 7], 1, {p, 0, 0}, p + 1};
    }
}

/* bloom-style DAG with two size classes: full res and half res */
static void build_bloom(PFGraph* g, int blur_stages) {
    memset(g, 0, sizeof(*g));
    int hw = W / 2, hh = H / 2, s = blur_stages;
    /* ids: 0 scene, 1 bright, 2 low, 3..3+s-1 blur chain,
             3+s small scene, 4+s merge, 5+s final */
    int id_sf = 3 + s, id_merge = 4 + s, id_final = 5 + s;
    g->n_res = id_final + 1;
    g->n_pass = s + 5;
    g->res[0] = (PFRes){W, H, -1, 1};
    g->res[1] = (PFRes){W, H, 0, 0};
    g->res[2] = (PFRes){hw, hh, 1, 0};
    for (int i = 0; i < s; i++)
        g->res[3 + i] = (PFRes){hw, hh, 2 + i, 0};
    g->res[id_sf] = (PFRes){hw, hh, s + 2, 0};
    g->res[id_merge] = (PFRes){hw, hh, s + 3, 0};
    g->res[id_final] = (PFRes){hw, hh, s + 4, 0};

    int p = 0;
    g->pass[p++] = (PFPass){K_BRIGHT, 1, {0, 0, 0}, 1};
    g->pass[p++] = (PFPass){K_DOWNSAMPLE, 1, {1, 0, 0}, 2};
    for (int i = 0; i < s; i++) {
        int in = i == 0 ? 2 : 2 + i;
        g->pass[p++] = (PFPass){K_BLUR, 1, {in, 0, 0}, 3 + i};
    }
    g->pass[p++] = (PFPass){K_DOWNSAMPLE, 1, {0, 0, 0}, id_sf};
    g->pass[p++] = (PFPass){K_ADD, 2, {id_sf, 2 + s, 0}, id_merge};
    g->pass[p++] = (PFPass){K_BLUR, 1, {id_merge, 0, 0}, id_final};
}

/* fan-out / fan-in: one scene split three ways, then pairwise merged */
static void build_fan(PFGraph* g) {
    memset(g, 0, sizeof(*g));
    g->n_res = 6; g->n_pass = 5;
    g->res[0] = (PFRes){W, H, -1, 1};
    for (int r = 1; r < 6; r++) g->res[r] = (PFRes){W, H, r - 1, 0};
    g->pass[0] = (PFPass){K_SAT, 1, {0, 0, 0}, 1};
    g->pass[1] = (PFPass){K_GAMMA, 1, {0, 0, 0}, 2};
    g->pass[2] = (PFPass){K_INV, 1, {0, 0, 0}, 3};
    g->pass[3] = (PFPass){K_ADD, 2, {1, 2, 0}, 4};
    g->pass[4] = (PFPass){K_MUL, 2, {4, 3, 0}, 5};
    /* producer metadata for the merge outputs */
    g->res[4].producer = 3;
    g->res[5].producer = 4;
}

/* ---- harness ------------------------------------------------------------ */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int topo_respects_deps(const PFGraph* g, const PFCompile* c) {
    int ordinal[MAX_PASS];
    for (int k = 0; k < g->n_pass; k++) ordinal[c->order[k]] = k;
    for (int p = 0; p < g->n_pass; p++) {
        const PFPass* pa = &g->pass[p];
        for (int i = 0; i < pa->n_in; i++) {
            int pr = g->res[pa->in[i]].producer;
            if (pr != -1 && ordinal[pr] >= ordinal[p]) return 0;
        }
    }
    return 1;
}

/* run one graph under both schedulers and compare every produced resource */
static int run_pair(const PFGraph* g, const PFCompile* c, int pool_cap,
                    PFDevice* naive_dev, PFDevice* pool_dev,
                    PFShot* ns, int* nn, PFShot* ps, int* np) {
    PFBuf* scene1 = make_scene(g->res[0].w, g->res[0].h);
    PFBuf* scene2 = make_scene(g->res[0].w, g->res[0].h);
    memcpy(scene2->px, scene1->px,
           (size_t)scene1->w * scene1->h * 4 * sizeof(float));

    pf_init(naive_dev, 0, 0);
    int e1 = pf_run(naive_dev, g, c, &scene1, 1, ns, nn);
    if (e1 == PF_OK) pf_end_frame(naive_dev);

    pf_init(pool_dev, 1, pool_cap);
    int e2 = pf_run(pool_dev, g, c, &scene2, 1, ps, np);
    if (e2 == PF_OK) pf_end_frame(pool_dev);

    int ok = e1 == PF_OK && e2 == PF_OK && shots_equal(ns, *nn, ps, *np);
    free(scene1->px); free(scene1);
    free(scene2->px); free(scene2);
    return ok;
}

int main(void) {
    /* direct kernel sanity on a tiny known input */
    {
        PFBuf in = {2, 2, 1, 0, malloc(2 * 2 * 4 * sizeof(float))};
        PFBuf out = {2, 2, 0, 0, malloc(2 * 2 * 4 * sizeof(float))};
        for (int i = 0; i < 16; i++) in.px[i] = (i % 4 == 3) ? 1.0f : 0.25f;
        PFBuf* pin = &in;
        pf_exec(K_INV, 1, &pin, &out, 0.0f);
        int exact = 1;
        for (int i = 0; i < 16; i++) {
            float want = (i % 4 == 3) ? 1.0f : 0.75f;
            if (out.px[i] != want) exact = 0;
        }
        check("kernel_inv_exact", exact);
        free(in.px); free(out.px);
    }

    /* compile-time validation */
    {
        PFGraph g; PFCompile c;
        build_linear(&g, 3);
        check("compile_ok", pf_compile(&g, &c) == PF_OK);
        check("topo_linear", topo_respects_deps(&g, &c));

        PFGraph cyc = g;
        cyc.pass[0].in[0] = 3;              /* reads the chain's last output */
        check("cycle_rejected", pf_compile(&cyc, &c) == PF_ERR_CYCLE);

        PFGraph bad = g;
        bad.pass[2].kernel = K_ADD;        /* arity mismatch */
        bad.pass[2].n_in = 1;
        check("arity_rejected", pf_compile(&bad, &c) == PF_ERR_ARG);

        PFGraph spec;
        build_bloom(&spec, 1);
        check("bloom_compile_ok", pf_compile(&spec, &c) == PF_OK);
        spec.res[spec.n_res - 1].w = W;    /* break half-res size rule */
        check("spec_rejected", pf_compile(&spec, &c) == PF_ERR_SPEC);
    }

    /* linear chains of growing length: the headline scaling table */
    printf("\nlinear chain, %dx%d RGBA, %.1f KiB per buffer\n",
           W, H, (W * H * 4.0 * sizeof(float)) / 1024.0);
    printf("%6s %9s %9s %12s %12s %13s %13s %6s\n",
           "passes", "naive_al", "pool_al", "naive_peak", "pool_peak",
           "naive_KiB", "pool_KiB", "equal");
    int lens[] = {1, 2, 4, 8, 16};
    long long pool_peak_bytes = -1;
    int peak_flat_rows = 1;
    for (unsigned t = 0; t < sizeof(lens) / sizeof(lens[0]); t++) {
        int n = lens[t];
        PFGraph g; PFCompile c;
        build_linear(&g, n);
        if (pf_compile(&g, &c) != PF_OK) { check("linear_compile", 0); continue; }

        PFDevice nd, pd; PFShot ns[MAX_RES], ps[MAX_RES];
        int nn, np;
        int equal = run_pair(&g, &c, 2, &nd, &pd, ns, &nn, ps, &np);
        printf("%6d %9d %9d %12d %12d %13.1f %13.1f %6d\n",
               n, nd.alloc_count, pd.alloc_count,
               nd.peak_bufs, pd.peak_bufs,
               nd.peak_bytes / 1024.0, pd.peak_bytes / 1024.0, equal);
        check("linear_topological", topo_respects_deps(&g, &c));
        check("linear_pixels_equal", equal);
        if (n >= 2) {
            check("linear_pool_two_allocs", pd.alloc_count == 2);
            check("linear_pool_peak_two", pd.peak_bufs == 2);
        }
        check("linear_naive_n_allocs", nd.alloc_count == n);
        check("linear_naive_peak_n", nd.peak_bufs == n);
        check("linear_naive_freed", nd.free_count == n);
        check("linear_cap_respected", pd.peak_bufs <= 2 && pd.err == PF_OK);
        if (n >= 2) {
            if (pool_peak_bytes >= 0 && pd.peak_bytes != pool_peak_bytes)
                peak_flat_rows = 0;
            pool_peak_bytes = pd.peak_bytes;
        }

        pf_destroy(&nd); pf_destroy(&pd);
        shots_free(ns, nn); shots_free(ps, np);
    }
    check("pool_peak_does_not_grow", peak_flat_rows); /* asserted per-row above */

    /* bloom DAG: branching graph, mixed buffer sizes, longer blur tails */
    printf("\nbloom DAG, full %dx%d + half %dx%d\n", W, H, W / 2, H / 2);
    printf("%6s %9s %9s %12s %12s %13s %13s %6s\n",
           "stages", "naive_al", "pool_al", "naive_peak", "pool_peak",
           "naive_KiB", "pool_KiB", "equal");
    for (int s = 1; s <= 4; s++) {
        PFGraph g; PFCompile c;
        build_bloom(&g, s);
        if (pf_compile(&g, &c) != PF_OK) { check("bloom_compile", 0); continue; }
        PFDevice nd, pd; PFShot ns[MAX_RES], ps[MAX_RES];
        int nn, np;
        int equal = run_pair(&g, &c, 3, &nd, &pd, ns, &nn, ps, &np);
        printf("%6d %9d %9d %12d %12d %13.1f %13.1f %6d\n",
               s, nd.alloc_count, pd.alloc_count,
               nd.peak_bufs, pd.peak_bufs,
               nd.peak_bytes / 1024.0, pd.peak_bytes / 1024.0, equal);
        check("bloom_topological", topo_respects_deps(&g, &c));
        check("bloom_pixels_equal", equal);
        check("bloom_pool_peak_three", pd.peak_bufs == 3);
        /* one full-res target plus three half-res targets alive together at
           the add merge; size classes never share a backing buffer */
        check("bloom_pool_four_allocs", pd.alloc_count == 4);
        check("bloom_naive_allocs", nd.alloc_count == s + 5);
        pf_destroy(&nd); pf_destroy(&pd);
        shots_free(ns, nn); shots_free(ps, np);
    }

    /* fan-out / fan-in graph */
    {
        PFGraph g; PFCompile c;
        build_fan(&g);
        check("fan_compile_ok", pf_compile(&g, &c) == PF_OK);
        check("fan_topological", topo_respects_deps(&g, &c));
        PFDevice nd, pd; PFShot ns[MAX_RES], ps[MAX_RES];
        int nn, np;
        int equal = run_pair(&g, &c, 4, &nd, &pd, ns, &nn, ps, &np);
        check("fan_pixels_equal", equal);
        check("fan_pool_reuses", pd.hits > 0 && pd.alloc_count == 4);
        pf_destroy(&nd); pf_destroy(&pd);
        shots_free(ns, nn); shots_free(ps, np);
    }

    /* hard cap: cap below the graph's true peak must fail, exact cap works */
    {
        PFGraph g; PFCompile c;
        build_bloom(&g, 2);
        pf_compile(&g, &c);
        PFDevice pd; PFShot ps[MAX_RES]; int np;
        PFBuf* scene = make_scene(W, H);
        pf_init(&pd, 1, 2);
        int err = pf_run(&pd, &g, &c, &scene, 1, ps, &np);
        check("cap_rejects_oversubscription", err == PF_ERR_CAP && pd.err == PF_ERR_CAP);
        pf_end_frame(&pd); pf_destroy(&pd); shots_free(ps, np);

        pf_init(&pd, 1, 3);
        err = pf_run(&pd, &g, &c, &scene, 1, ps, &np);
        check("cap_exact_peak_succeeds", err == PF_OK && pd.peak_bufs == 3);
        pf_end_frame(&pd); pf_destroy(&pd); shots_free(ps, np);
        free(scene->px); free(scene);
    }

    /* persistent pool across frames: second frame leases, never allocates */
    {
        PFGraph g; PFCompile c;
        build_linear(&g, 8);
        pf_compile(&g, &c);
        PFDevice pd;
        pf_init(&pd, 1, 2);
        PFShot shots[MAX_RES]; int n;
        for (int frame = 0; frame < 2; frame++) {
            PFBuf* scene = make_scene(W, H);
            int err = pf_run(&pd, &g, &c, &scene, 1, shots, &n);
            check("cross_frame_ok", err == PF_OK);
            pf_end_frame(&pd);
            shots_free(shots, n);
            free(scene->px); free(scene);
        }
        check("cross_frame_two_backing_allocs", pd.alloc_count == 2);
        check("cross_frame_second_frame_leased", pd.hits == 14 && pd.misses == 2);
        int canary = 1;
        for (int i = 0; i < pd.n_buf; i++)
            if (!pd.buf[i].live && !isnan(pd.buf[i].px[0])) canary = 0;
        check("recycle_canary_poisoned", canary);
        pf_destroy(&pd);
    }

    printf("\nall_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
