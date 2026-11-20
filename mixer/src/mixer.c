
/* mixer: multi-bus stereo mixing with configurable per-bus plugin chains.
 *
 * Each bus (and the master output) owns an ordered chain of processing
 * nodes. Nodes carry typed parameters, can be reordered at runtime and
 * bypassed click-free (wet/dry crossfade). The whole mixer state — chain
 * order, node types, parameters, bypass flags and internal node state —
 * serializes to a versioned blob and can be loaded back bit-for-bit.
 *
 * The processing path uses fixed-capacity arrays and stack scratch only;
 * there is no allocation per block, so cost grows linearly with the total
 * node count.
 *
 * ref_mix() below is the original hardcoded signal path, kept verbatim as
 * the reference: a chain configured in the legacy order must match it
 * sample-for-sample. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <time.h>

#define MAX_SRC   8
#define FRAMES    128
#define BUS_COUNT 3                       /* 0 = sfx, 1 = music, 2 = voice */
#define MAX_NODES 32
#define MAX_PARAMS 4
#define MASTER_CHAIN BUS_COUNT            /* chains: 0..2 per bus, 3 master */
#define CHAIN_COUNT (BUS_COUNT + 1)
#define RAMP_SECONDS 0.005f               /* bypass crossfade time */
#define DEFAULT_SR   48000.0f

/* ---------------------------------------------------------------- nodes */

enum { NODE_GAIN = 0, NODE_DUCK, NODE_LOWPASS, NODE_SOFTCLIP, NODE_TYPES };

/* Node parameter layout:
 * GAIN     : [0] gain
 * DUCK     : [0] depth (0..1), [1] energy threshold, [2] sidechain bus
 * LOWPASS  : [0] coefficient a (0..1); state[0]/state[1] = L/R last output
 * SOFTCLIP : none */
typedef struct Node {
    int type;
    int bypass;                          /* logical target: 1 = bypassed */
    float wet;                           /* current crossfade gain 0..1 */
    float params[MAX_PARAMS];
    float state[MAX_PARAMS];
} Node;

typedef struct {
    Node nodes[MAX_NODES];
    int count;
} Chain;

typedef struct {
    Chain ch[CHAIN_COUNT];
} Mixer;

typedef struct {
    const float* const* raw;             /* bus signals before any chain */
    int buses;
    int frames;
    float sr;
} ProcCtx;

typedef void (*ProcFn)(Node*, const ProcCtx*, const float* in, float* out);
typedef void (*LatentFn)(Node*, const ProcCtx*, const float* in);

static float soft_clip(float v) {
    const float k = 0.9f;
    float a = fabsf(v);
    if (a <= k) return v;
    float s = v < 0 ? -1.0f : 1.0f;
    return s * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

static void proc_gain(Node* n, const ProcCtx* ctx, const float* in, float* out) {
    float g = n->params[0];
    for (int i = 0; i < ctx->frames * 2; i++) out[i] = in[i] * g;
}

static void proc_duck(Node* n, const ProcCtx* ctx, const float* in, float* out) {
    int sc = (int)n->params[2];
    float e = 0.0f;
    if (sc >= 0 && sc < ctx->buses)
        for (int i = 0; i < ctx->frames * 2; i++) e += fabsf(ctx->raw[sc][i]);
    e /= (float)(ctx->frames * 2);
    float dg = e > n->params[1] ? 1.0f - n->params[0] : 1.0f;
    for (int i = 0; i < ctx->frames * 2; i++) out[i] = in[i] * dg;
}

static void proc_lowpass(Node* n, const ProcCtx* ctx, const float* in, float* out) {
    float a = n->params[0];
    float l = n->state[0], r = n->state[1];
    for (int i = 0; i < ctx->frames * 2; i += 2) {
        l += a * (in[i] - l);     out[i] = l;
        r += a * (in[i + 1] - r); out[i + 1] = r;
    }
    n->state[0] = l;
    n->state[1] = r;
}

/* While bypassed, stateful nodes keep tracking the dry signal so that
 * un-bypassing resumes from a warm state instead of a stale one. */
static void latent_lowpass(Node* n, const ProcCtx* ctx, const float* in) {
    float a = n->params[0];
    float l = n->state[0], r = n->state[1];
    for (int i = 0; i < ctx->frames * 2; i += 2) {
        l += a * (in[i] - l);
        r += a * (in[i + 1] - r);
    }
    n->state[0] = l;
    n->state[1] = r;
}

static void proc_softclip(Node* n, const ProcCtx* ctx, const float* in, float* out) {
    (void)n;
    for (int i = 0; i < ctx->frames * 2; i++) out[i] = soft_clip(in[i]);
}

static const ProcFn NODE_PROC[NODE_TYPES] = {
    proc_gain, proc_duck, proc_lowpass, proc_softclip
};
static const LatentFn NODE_LATENT[NODE_TYPES] = {
    NULL, NULL, latent_lowpass, NULL
};
static const char* const NODE_NAME[NODE_TYPES] = {
    "gain", "duck", "lowpass", "softclip"
};

/* --------------------------------------------------------- chain control */

static void chain_clear(Chain* c) {
    memset(c, 0, sizeof(*c));
}

static void node_init(Node* n, int type, const float* p, int np, int bypass) {
    memset(n, 0, sizeof(*n));
    n->type = type;
    n->bypass = bypass ? 1 : 0;
    for (int i = 0; i < np; i++) n->params[i] = p[i];
    n->wet = bypass ? 0.0f : 1.0f;          /* steady state, no ramp */
}

/* Appends a node; its index is the chain position. Returns index or -1. */
static int chain_add(Chain* c, int type, const float* p, int np, int bypass) {
    if (type < 0 || type >= NODE_TYPES || np < 0 || np > MAX_PARAMS ||
        c->count >= MAX_NODES)
        return -1;
    node_init(&c->nodes[c->count], type, p, np, bypass);
    return c->count++;
}

static int chain_insert(Chain* c, int idx, int type,
                        const float* p, int np, int bypass) {
    if (idx < 0 || idx > c->count || c->count >= MAX_NODES ||
        type < 0 || type >= NODE_TYPES || np < 0 || np > MAX_PARAMS)
        return -1;
    memmove(&c->nodes[idx + 1], &c->nodes[idx],
            (size_t)(c->count - idx) * sizeof(Node));
    node_init(&c->nodes[idx], type, p, np, bypass);
    c->count++;
    return idx;
}

static int chain_remove(Chain* c, int idx) {
    if (idx < 0 || idx >= c->count) return -1;
    memmove(&c->nodes[idx], &c->nodes[idx + 1],
            (size_t)(c->count - 1 - idx) * sizeof(Node));
    c->count--;
    return 0;
}

static int chain_move(Chain* c, int from, int to) {
    if (from < 0 || from >= c->count || to < 0 || to >= c->count) return -1;
    if (from == to) return 0;
    Node tmp = c->nodes[from];
    if (from < to)
        memmove(&c->nodes[from], &c->nodes[from + 1],
                (size_t)(to - from) * sizeof(Node));
    else
        memmove(&c->nodes[to + 1], &c->nodes[to],
                (size_t)(from - to) * sizeof(Node));
    c->nodes[to] = tmp;
    return 0;
}

static int node_set_param(Node* n, int i, float v) {
    if (i < 0 || i >= MAX_PARAMS || !isfinite(v)) return -1;
    n->params[i] = v;
    return 0;
}

static void node_set_bypass(Node* n, int bypass) {
    n->bypass = bypass ? 1 : 0;              /* wet ramps on next process */
}

/* Runs one chain in place. scratch must hold frames*2 floats; it is the
 * only working memory and is reused across nodes. */
static void chain_process(Chain* c, const ProcCtx* ctx,
                          float* buf, float* scratch) {
    int m = ctx->frames * 2;
    float step = 1.0f / fmaxf(1.0f, RAMP_SECONDS * ctx->sr);

    for (int k = 0; k < c->count; k++) {
        Node* n = &c->nodes[k];
        float target = n->bypass ? 0.0f : 1.0f;

        if (n->wet == 1.0f && target == 1.0f) {
            NODE_PROC[n->type](n, ctx, buf, buf);            /* enabled */
        } else if (n->wet == 0.0f && target == 0.0f) {
            if (NODE_LATENT[n->type])                        /* bypassed */
                NODE_LATENT[n->type](n, ctx, buf);
        } else {
            /* Crossfade per sample; snap to the exact endpoint so steady
             * state stays bit-identical to a hard wet/dry path. */
            NODE_PROC[n->type](n, ctx, buf, scratch);
            float wet = n->wet;
            for (int i = 0; i < m; i++) {
                if (wet != target) {
                    float d = target - wet;
                    if (fabsf(d) <= step) wet = target;
                    else wet += d > 0.0f ? step : -step;
                }
                buf[i] = wet * scratch[i] + (1.0f - wet) * buf[i];
            }
            n->wet = wet;
        }
    }
}

/* ---------------------------------------------------------------- mixer */

typedef struct {
    int bus;
    float gain;
} Source;

static void mixer_process(Mixer* mx, const Source* srcs, int ns,
                          const float* const* in, float* out,
                          int frames, float sr) {
    if (frames <= 0 || frames > FRAMES) return;

    float raw[BUS_COUNT][FRAMES * 2];
    memset(raw, 0, sizeof(raw));
    for (int s = 0; s < ns; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b >= BUS_COUNT) continue;
        for (int i = 0; i < frames * 2; i++)
            raw[b][i] += in[s][i] * srcs[s].gain;
    }

    const float* rawp[BUS_COUNT];
    for (int b = 0; b < BUS_COUNT; b++) rawp[b] = raw[b];
    ProcCtx ctx = { rawp, BUS_COUNT, frames, sr };

    float busbuf[BUS_COUNT][FRAMES * 2];
    float scratch[FRAMES * 2];
    for (int b = 0; b < BUS_COUNT; b++) {
        memcpy(busbuf[b], raw[b], sizeof(float) * (size_t)frames * 2);
        chain_process(&mx->ch[b], &ctx, busbuf[b], scratch);
    }

    for (int i = 0; i < frames * 2; i++) {
        float v = busbuf[0][i];                 /* (b0 + b1) + b2, same   */
        v += busbuf[1][i];                      /* association as the     */
        v += busbuf[2][i];                      /* hardcoded expression   */
        out[i] = v;
    }
    chain_process(&mx->ch[MASTER_CHAIN], &ctx, out, scratch);
}

/* -------------------------------------------------------- serialization */

#define MIX_MAGIC   0x4D584331u                 /* 'MXC1' */
#define MIX_VERSION 1u

typedef struct {
    int32_t type;
    int32_t bypass;
    float params[MAX_PARAMS];
    float state[MAX_PARAMS];
} NodeSer;

typedef struct {
    int32_t count;
    int32_t pad;
    NodeSer nodes[MAX_NODES];
} ChainSer;

typedef struct {
    uint32_t magic;
    uint32_t version;
    int32_t chain_count;
    int32_t pad;
    ChainSer chains[CHAIN_COUNT];
} MixerBlob;

static size_t mixer_state_size(void) {
    return sizeof(MixerBlob);
}

static int mixer_save(const Mixer* mx, void* blob, size_t cap) {
    if (!blob || cap < sizeof(MixerBlob)) return -1;

    MixerBlob b;
    memset(&b, 0, sizeof(b));
    b.magic = MIX_MAGIC;
    b.version = MIX_VERSION;
    b.chain_count = CHAIN_COUNT;

    for (int c = 0; c < CHAIN_COUNT; c++) {
        b.chains[c].count = mx->ch[c].count;
        for (int k = 0; k < mx->ch[c].count; k++) {
            const Node* n = &mx->ch[c].nodes[k];
            NodeSer* s = &b.chains[c].nodes[k];
            s->type = n->type;
            s->bypass = n->bypass;
            memcpy(s->params, n->params, sizeof(s->params));
            memcpy(s->state, n->state, sizeof(s->state));
        }
    }
    memcpy(blob, &b, sizeof(b));
    return 0;
}

/* Validates everything into a local mixer first; on any error *mx is
 * left untouched. */
static int mixer_load(Mixer* mx, const void* blob, size_t cap) {
    if (!blob || cap < sizeof(MixerBlob)) return -1;

    MixerBlob b;
    memcpy(&b, blob, sizeof(b));
    if (b.magic != MIX_MAGIC || b.version != MIX_VERSION ||
        b.chain_count != CHAIN_COUNT)
        return -1;

    Mixer tmp;
    memset(&tmp, 0, sizeof(tmp));
    for (int c = 0; c < CHAIN_COUNT; c++) {
        int cnt = b.chains[c].count;
        if (cnt < 0 || cnt > MAX_NODES) return -1;
        for (int k = 0; k < cnt; k++) {
            NodeSer* s = &b.chains[c].nodes[k];
            if (s->type < 0 || s->type >= NODE_TYPES ||
                (s->bypass != 0 && s->bypass != 1))
                return -1;
            for (int i = 0; i < MAX_PARAMS; i++)
                if (!isfinite(s->params[i]) || !isfinite(s->state[i]))
                    return -1;
            Node* n = &tmp.ch[c].nodes[k];
            n->type = s->type;
            n->bypass = s->bypass;
            n->wet = s->bypass ? 0.0f : 1.0f;
            memcpy(n->params, s->params, sizeof(n->params));
            memcpy(n->state, s->state, sizeof(n->state));
        }
        tmp.ch[c].count = cnt;
    }
    *mx = tmp;
    return 0;
}

/* ------------------------------------------- hardcoded reference path */

typedef struct {
    float gain[3];
    float duck_depth;
    int duck_enabled;
} RefMixer;

static void ref_mix(const RefMixer* mx, const Source* srcs, int n,
                    const float* const* in, float* out, int frames) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    float bus[3][FRAMES * 2];
    memset(bus, 0, sizeof(bus));

    for (int s = 0; s < n; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b > 2) continue;
        for (int i = 0; i < frames * 2; i++)
            bus[b][i] += in[s][i] * srcs[s].gain;
    }

    float dg = 1.0f;
    if (mx->duck_enabled) {
        float e = 0.0f;
        for (int i = 0; i < frames * 2; i++) e += fabsf(bus[2][i]);
        e /= (float)(frames * 2);
        if (e > 0.01f) dg = 1.0f - mx->duck_depth;
    }

    for (int i = 0; i < frames * 2; i++) {
        float v = bus[0][i] * mx->gain[0] * dg
                + bus[1][i] * mx->gain[1]
                + bus[2][i] * mx->gain[2];
        out[i] = soft_clip(v);
    }
}

/* Builds the chain equivalent of the hardcoded path. Bus 0 order is
 * gain -> duck: float multiply is not associative, so matching the exact
 * operation order is required for sample-exact output. */
static void build_legacy(Mixer* m, const RefMixer* ref) {
    for (int c = 0; c < CHAIN_COUNT; c++) chain_clear(&m->ch[c]);
    for (int b = 0; b < BUS_COUNT; b++)
        chain_add(&m->ch[b], NODE_GAIN, &ref->gain[b], 1, 0);

    float dp[3] = { ref->duck_depth, 0.01f, (float)(BUS_COUNT - 1) };
    chain_add(&m->ch[0], NODE_DUCK, dp, 3, !ref->duck_enabled);
    chain_add(&m->ch[MASTER_CHAIN], NODE_SOFTCLIP, NULL, 0, 0);
}

/* ---------------------------------------------------------------- tests */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int buffers_equal(const float* a, const float* b, int n, const char* tag) {
    for (int i = 0; i < n; i++)
        if (a[i] != b[i]) {
            printf("  [%s] first diff at i=%d ref=%.9g got=%.9g\n",
                   tag, i, a[i], b[i]);
            return 0;           /* bit-exact, no epsilon */
        }
    return 1;
}

static float peak(const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(b[i]) > m) m = fabsf(b[i]);
    return m;
}

static uint32_t rng = 0x12345678u;
static float rnd(float lo, float hi) {
    rng = rng * 1664525u + 1013904223u;
    float t = (float)(rng >> 8) * (1.0f / 16777216.0f);
    return lo + (hi - lo) * t;
}

/* Chain output must equal the hardcoded reference output sample-by-sample
 * across all original scenarios plus randomized ones. */
static void test_equivalence(void) {
    float sfx[FRAMES * 2], music[FRAMES * 2], voice[FRAMES * 2];
    float ref_out[FRAMES * 2], ch_out[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++) {
        sfx[i] = 0.5f; music[i] = 0.25f; voice[i] = 0.0f;
    }
    const float* in[3] = { sfx, music, voice };
    Source srcs[3] = { {0, 1.0f}, {1, 1.0f}, {2, 1.0f} };

    RefMixer ref = { {1.0f, 1.0f, 1.0f}, 0.5f, 1 };
    Mixer m;
    build_legacy(&m, &ref);
    int exact = 1;

    ref_mix(&ref, srcs, 3, in, ref_out, FRAMES);
    mixer_process(&m, srcs, 3, in, ch_out, FRAMES, DEFAULT_SR);
    exact &= buffers_equal(ref_out, ch_out, FRAMES * 2, "s1_sum");
    check("sum_ok", fabsf(ch_out[0] - 0.75f) < 1e-5f);
    check("no_clip_when_low", peak(ch_out, FRAMES * 2) < 0.99f);

    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.3f;
    ref_mix(&ref, srcs, 3, in, ref_out, FRAMES);
    mixer_process(&m, srcs, 3, in, ch_out, FRAMES, DEFAULT_SR);
    exact &= buffers_equal(ref_out, ch_out, FRAMES * 2, "s2_duck_on");
    float with_duck = ch_out[0];

    /* duck off: hardcoded flag vs. node bypass must behave identically.
     * The bypass crossfade ramps over the block where the toggle happens;
     * the hardcoded reference switches instantly, so compare the next block
     * once the ramp has settled on its exact endpoint. Signals are held
     * constant and the reference is stateless, so re-running is valid. */
    ref.duck_enabled = 0;
    node_set_bypass(&m.ch[0].nodes[1], 1);
    mixer_process(&m, srcs, 3, in, ch_out, FRAMES, DEFAULT_SR);   /* ramp */
    ref_mix(&ref, srcs, 3, in, ref_out, FRAMES);
    mixer_process(&m, srcs, 3, in, ch_out, FRAMES, DEFAULT_SR);   /* dry */
    exact &= buffers_equal(ref_out, ch_out, FRAMES * 2, "s3_duck_off");
    float without_duck = ch_out[0];
    check("duck_lowers_sfx", with_duck < without_duck);

    ref.duck_enabled = 1;
    node_set_bypass(&m.ch[0].nodes[1], 0);
    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.0f;
    mixer_process(&m, srcs, 3, in, ch_out, FRAMES, DEFAULT_SR);   /* ramp */
    ref_mix(&ref, srcs, 3, in, ref_out, FRAMES);
    mixer_process(&m, srcs, 3, in, ch_out, FRAMES, DEFAULT_SR);   /* wet */
    exact &= buffers_equal(ref_out, ch_out, FRAMES * 2, "s4_duck_idle");
    check("duck_idle_passthrough", fabsf(ch_out[0] - 0.75f) < 1e-5f);

    for (int s = 0; s < 3; s++) srcs[s].gain = 4.0f;
    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.5f;
    ref_mix(&ref, srcs, 3, in, ref_out, FRAMES);
    mixer_process(&m, srcs, 3, in, ch_out, FRAMES, DEFAULT_SR);
    exact &= buffers_equal(ref_out, ch_out, FRAMES * 2, "s5_clip");
    check("soft_clip_bounds", peak(ch_out, FRAMES * 2) <= 1.0f + 1e-6f);

    srcs[0].bus = 7;
    ref_mix(&ref, srcs, 3, in, ref_out, FRAMES);
    mixer_process(&m, srcs, 3, in, ch_out, FRAMES, DEFAULT_SR);
    exact &= buffers_equal(ref_out, ch_out, FRAMES * 2, "s6_badbus");
    check("bad_bus_ignored", peak(ch_out, FRAMES * 2) <= 1.0f + 1e-6f);
    srcs[0].bus = 0;

    /* randomized parameters, source gains and signals, several blocks */
    for (int t = 0; t < 20 && exact; t++) {
        ref.gain[0] = rnd(0.1f, 2.0f);
        ref.gain[1] = rnd(0.1f, 2.0f);
        ref.gain[2] = rnd(0.1f, 2.0f);
        ref.duck_depth = rnd(0.0f, 0.95f);
        ref.duck_enabled = (t & 1);
        build_legacy(&m, &ref);
        for (int blk = 0; blk < 4; blk++) {
            for (int i = 0; i < FRAMES * 2; i++) {
                sfx[i] = rnd(-1.0f, 1.0f);
                music[i] = rnd(-1.0f, 1.0f);
                voice[i] = rnd(-1.0f, 1.0f);
            }
            for (int s = 0; s < 3; s++) srcs[s].gain = rnd(0.2f, 3.0f);
            ref_mix(&ref, srcs, 3, in, ref_out, FRAMES);
            mixer_process(&m, srcs, 3, in, ch_out, FRAMES, DEFAULT_SR);
            exact &= buffers_equal(ref_out, ch_out, FRAMES * 2, "rand");
        }
    }
    check("chain_matches_ref_exact", exact);
}

/* Nodes carry parameters and a configurable order. */
static void test_order_and_params(void) {
    Chain c;
    chain_clear(&c);
    int ok = 1;

    ok &= chain_add(&c, NODE_GAIN, (float[]){0.8f}, 1, 0) == 0;
    ok &= chain_add(&c, NODE_LOWPASS, (float[]){0.2f}, 1, 0) == 1;
    ok &= chain_add(&c, NODE_GAIN, (float[]){1.5f}, 1, 0) == 2;
    ok &= c.count == 3;
    ok &= c.nodes[0].type == NODE_GAIN && c.nodes[0].params[0] == 0.8f;
    ok &= c.nodes[1].type == NODE_LOWPASS && c.nodes[1].params[0] == 0.2f;
    ok &= c.nodes[2].type == NODE_GAIN && c.nodes[2].params[0] == 1.5f;

    chain_move(&c, 2, 0);                     /* gain1.5 moves to front */
    ok &= c.nodes[0].params[0] == 1.5f && c.nodes[0].type == NODE_GAIN;
    ok &= c.nodes[1].type == NODE_GAIN && c.nodes[1].params[0] == 0.8f;
    ok &= c.nodes[2].type == NODE_LOWPASS;

    chain_move(&c, 0, 2);                     /* and back */
    ok &= c.nodes[0].params[0] == 0.8f && c.nodes[2].params[0] == 1.5f;

    ok &= chain_insert(&c, 1, NODE_SOFTCLIP, NULL, 0, 1) == 1;
    ok &= c.count == 4 && c.nodes[1].type == NODE_SOFTCLIP;
    ok &= c.nodes[1].bypass == 1 && c.nodes[1].wet == 0.0f;
    ok &= node_set_param(&c.nodes[0], 0, 0.33f) == 0;
    ok &= c.nodes[0].params[0] == 0.33f;
    ok &= node_set_param(&c.nodes[0], 0, NAN) == -1;

    ok &= chain_remove(&c, 1) == 0;
    ok &= c.count == 3 && c.nodes[1].type == NODE_LOWPASS;
    ok &= strcmp(NODE_NAME[c.nodes[1].type], "lowpass") == 0;

    Chain full;
    chain_clear(&full);
    int filled = 0;
    while (chain_add(&full, NODE_GAIN, (float[]){1.0f}, 1, 0) >= 0) filled++;
    ok &= filled == MAX_NODES;
    ok &= chain_add(&full, NODE_GAIN, (float[]){1.0f}, 1, 0) == -1;
    ok &= chain_insert(&full, 0, NODE_GAIN, (float[]){1.0f}, 1, 0) == -1;
    ok &= chain_add(&c, 99, NULL, 0, 0) == -1;

    check("order_and_params", ok);
}

/* Whole-state save/load: order, types, every parameter, bypass flags and
 * internal node state come back identical, and processing continues
 * bit-exactly from the restored state. */
static void test_state_roundtrip(void) {
    Mixer a, b;
    memset(&b, 0x55, sizeof(b));              /* poison destination */

    for (int c = 0; c < CHAIN_COUNT; c++) chain_clear(&a.ch[c]);
    chain_add(&a.ch[0], NODE_GAIN, (float[]){0.7f}, 1, 0);
    chain_add(&a.ch[0], NODE_LOWPASS, (float[]){0.3f}, 1, 0);
    chain_add(&a.ch[1], NODE_GAIN, (float[]){1.2f}, 1, 0);
    chain_add(&a.ch[1], NODE_DUCK, (float[]){0.6f, 0.01f, 2.0f}, 3, 1);
    chain_add(&a.ch[2], NODE_GAIN, (float[]){0.9f}, 1, 0);
    chain_add(&a.ch[MASTER_CHAIN], NODE_SOFTCLIP, NULL, 0, 0);

    /* exercise the lowpass so its internal state is non-zero */
    float sig[FRAMES * 2], out_a[FRAMES * 2], out_b[FRAMES * 2];
    const float* in[1] = { sig };
    Source src = { 0, 1.0f };
    for (int blk = 0; blk < 5; blk++) {
        for (int i = 0; i < FRAMES * 2; i++) sig[i] = rnd(-1.0f, 1.0f);
        mixer_process(&a, &src, 1, in, out_a, FRAMES, DEFAULT_SR);
    }
    int lp_state_nonzero = a.ch[0].nodes[1].state[0] != 0.0f ||
                           a.ch[0].nodes[1].state[1] != 0.0f;

    static char blob[sizeof(MixerBlob) + 64];
    check("save_ok", mixer_save(&a, blob, mixer_state_size()) == 0);
    check("save_rejects_small_buffer",
          mixer_save(&a, blob, mixer_state_size() - 1) == -1);
    check("load_ok", mixer_load(&b, blob, mixer_state_size()) == 0);

    int ok = lp_state_nonzero;
    int params_checked = 0;
    for (int c = 0; c < CHAIN_COUNT; c++) {
        ok &= a.ch[c].count == b.ch[c].count;
        for (int k = 0; k < a.ch[c].count; k++) {
            const Node* x = &a.ch[c].nodes[k];
            const Node* y = &b.ch[c].nodes[k];
            ok &= x->type == y->type && x->bypass == y->bypass;
            ok &= y->wet == (y->bypass ? 0.0f : 1.0f);
            for (int i = 0; i < MAX_PARAMS; i++) {
                ok &= x->params[i] == y->params[i];
                ok &= x->state[i] == y->state[i];
                params_checked += 2;
            }
        }
    }
    ok &= params_checked ==
          /* 2 comparisons x 4 params x total node count */
          2 * MAX_PARAMS * 6;
    check("state_roundtrip_fields", ok);

    /* restored mixer renders the same next block bit-exactly */
    int cont = 1;
    for (int blk = 0; blk < 4; blk++) {
        for (int i = 0; i < FRAMES * 2; i++) sig[i] = rnd(-1.0f, 1.0f);
        mixer_process(&a, &src, 1, in, out_a, FRAMES, DEFAULT_SR);
        mixer_process(&b, &src, 1, in, out_b, FRAMES, DEFAULT_SR);
        cont &= buffers_equal(out_a, out_b, FRAMES * 2, "roundtrip");
    }
    check("state_roundtrip_processes_exact", cont);

    /* loading replaces previous configuration rather than merging */
    Mixer small;
    chain_clear(&small.ch[0]);
    chain_add(&small.ch[0], NODE_GAIN, (float[]){2.0f}, 1, 0);
    cont = mixer_load(&small, blob, mixer_state_size()) == 0;
    cont &= small.ch[0].count == a.ch[0].count;
    cont &= small.ch[0].nodes[0].params[0] == 0.7f;
    check("load_replaces_config", cont);

    /* corrupted / truncated blobs are rejected, target left untouched */
    Mixer guard;
    mixer_save(&a, &guard, sizeof(guard));
    int rejected = 1;

    MixerBlob bad;
    memcpy(&bad, blob, sizeof(bad));
    bad.magic ^= 1u;
    rejected &= mixer_load(&b, &bad, sizeof(bad)) == -1;

    memcpy(&bad, blob, sizeof(bad));
    bad.version = 999u;
    rejected &= mixer_load(&b, &bad, sizeof(bad)) == -1;

    memcpy(&bad, blob, sizeof(bad));
    bad.chain_count = 7;
    rejected &= mixer_load(&b, &bad, sizeof(bad)) == -1;

    memcpy(&bad, blob, sizeof(bad));
    bad.chains[0].count = MAX_NODES + 1;
    rejected &= mixer_load(&b, &bad, sizeof(bad)) == -1;

    memcpy(&bad, blob, sizeof(bad));
    bad.chains[0].nodes[0].type = -7;
    rejected &= mixer_load(&b, &bad, sizeof(bad)) == -1;

    memcpy(&bad, blob, sizeof(bad));
    bad.chains[0].nodes[0].params[0] = INFINITY;
    rejected &= mixer_load(&b, &bad, sizeof(bad)) == -1;

    rejected &= mixer_load(&b, blob, mixer_state_size() - 8) == -1;
    check("state_rejects_corrupt", rejected);
    check("corrupt_load_leaves_mixer_intact",
          b.ch[0].count == a.ch[0].count &&
          b.ch[0].nodes[0].params[0] == 0.7f);
}

/* Toggling bypass must not pop: the wet/dry crossfade ramps per sample,
 * lands on exact endpoints, and un-bypass restores the wet path exactly. */
static void test_bypass_no_click(void) {
    Chain c;
    chain_clear(&c);
    chain_add(&c, NODE_GAIN, (float[]){4.0f}, 1, 0);

    float in[FRAMES * 2], scratch[FRAMES * 2];
    float raw[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++) { in[i] = 0.25f; raw[i] = 0.25f; }
    const float* rp[1] = { raw };
    ProcCtx ctx = { rp, 1, FRAMES, DEFAULT_SR };

    /* hard switch for contrast: instant 4x <-> 1x jump of 0.75 */
    float hard_max = 0.0f, hard_prev = 1.0f;
    float ramp_max = 0.0f, ramp_prev = 1.0f;   /* block 0 is steady wet */
    int wet_exact_before = 1, dry_exact = 1, wet_exact_after = 1;

    for (int blk = 0; blk < 10; blk++) {
        for (int i = 0; i < FRAMES * 2; i++) in[i] = 0.25f;
        if (blk == 3) node_set_bypass(&c.nodes[0], 1);
        if (blk == 6) node_set_bypass(&c.nodes[0], 0);
        chain_process(&c, &ctx, in, scratch);          /* in-place on in */

        for (int i = 0; i < FRAMES * 2; i++) {
            if (blk + i > 0) {
                float d = fabsf(in[i] - ramp_prev);
                if (d > ramp_max) ramp_max = d;
            }
            ramp_prev = in[i];

            int bypassed = blk >= 3 && blk < 6;
            float hard = bypassed ? 0.25f : 1.0f;
            if (blk + i > 0) {
                float d = fabsf(hard - hard_prev);
                if (d > hard_max) hard_max = d;
            }
            hard_prev = hard;
        }
        if (blk == 2) wet_exact_before &= in[FRAMES] == 1.0f;
        if (blk == 5) dry_exact &= in[FRAMES] == 0.25f;
        if (blk == 9) wet_exact_after &= in[FRAMES] == 1.0f;
    }

    printf("hard_switch_max_delta=%.6f ramp_max_delta=%.6f\n",
           hard_max, ramp_max);
    check("hard_switch_pops", hard_max == 0.75f);
    check("bypass_no_click", ramp_max < hard_max / 20.0f);
    check("bypass_endpoints_exact",
          wet_exact_before && dry_exact && wet_exact_after);
}

/* Bypassed stateful nodes keep their state warm and resume cleanly;
 * the bypass flag itself survives a state round-trip. The continuity
 * measurements use a constant input so every sample-to-sample delta is
 * attributable to the bypass machinery, never to the signal. */
static void test_bypass_restores(void) {
    Chain c;
    chain_clear(&c);
    chain_add(&c, NODE_LOWPASS, (float[]){0.2f}, 1, 0);

    float in[FRAMES * 2], scratch[FRAMES * 2];
    float raw[FRAMES * 2];
    const float* rp[1] = { raw };
    ProcCtx ctx = { rp, 1, FRAMES, DEFAULT_SR };

    for (int i = 0; i < FRAMES * 2; i++) { in[i] = 0.5f; raw[i] = 0.5f; }
    for (int blk = 0; blk < 3; blk++)
        chain_process(&c, &ctx, in, scratch);
    check("warmup_reaches_input", fabsf(in[FRAMES * 2 - 1] - 0.5f) < 1e-5f);

    /* bypass: first block ramps, following blocks are dry passthrough */
    node_set_bypass(&c.nodes[0], 1);
    float max_delta = 0.0f, prev = in[FRAMES * 2 - 1];
    for (int blk = 0; blk < 3; blk++) {
        for (int i = 0; i < FRAMES * 2; i++) { in[i] = 0.5f; raw[i] = 0.5f; }
        chain_process(&c, &ctx, in, scratch);
        for (int i = 0; i < FRAMES * 2; i++) {
            float d = fabsf(in[i] - prev);
            if (d > max_delta) max_delta = d;
            prev = in[i];
        }
    }
    int bypass_passthrough = 1;
    for (int i = 0; i < FRAMES * 2; i++) bypass_passthrough &= in[i] == raw[i];
    float latent_state = c.nodes[0].state[0];
    check("latent_state_tracks", fabsf(latent_state - 0.5f) < 1e-5f);

    /* bypass flag and the warm latent state round-trip through the blob */
    Mixer a, b;
    memset(&a, 0, sizeof(a));
    a.ch[0].count = 1;
    a.ch[0].nodes[0] = c.nodes[0];
    MixerBlob blob;
    int ok = mixer_save(&a, &blob, sizeof(blob)) == 0;
    ok &= mixer_load(&b, &blob, sizeof(blob)) == 0;
    ok &= b.ch[0].nodes[0].bypass == 1;
    ok &= b.ch[0].nodes[0].state[0] == latent_state;
    ok &= b.ch[0].nodes[0].wet == 0.0f;

    /* un-bypass with the same constant input: smooth re-entry */
    node_set_bypass(&c.nodes[0], 0);
    for (int i = 0; i < FRAMES * 2; i++) { in[i] = 0.5f; raw[i] = 0.5f; }
    chain_process(&c, &ctx, in, scratch);       /* ramp block */
    for (int i = 0; i < FRAMES * 2; i++) {
        float d = fabsf(in[i] - prev);
        if (d > max_delta) max_delta = d;
        prev = in[i];
    }
    check("bypass_resume_continuous", max_delta < 0.05f);

    /* filter is active again: after a settling block, a square wave is
     * smoothed (mean absolute derivative well below the input's) */
    chain_process(&c, &ctx, in, scratch);       /* settle back to steady wet */
    for (int i = 0; i < FRAMES * 2; i++) {
        in[i] = (i & 2) ? 1.0f : -1.0f;
        raw[i] = in[i];
    }
    chain_process(&c, &ctx, in, scratch);
    float in_mad = 0.0f, out_mad = 0.0f;
    for (int i = 1; i < FRAMES * 2; i++) {
        in_mad += fabsf(raw[i] - raw[i - 1]);
        out_mad += fabsf(in[i] - in[i - 1]);
    }
    in_mad /= (FRAMES * 2 - 1);
    out_mad /= (FRAMES * 2 - 1);

    check("bypass_passthrough_dry", bypass_passthrough);
    check("bypass_state_persists", ok);
    check("filter_alive_after_restore", out_mad < in_mad * 0.5f);
}

/* Doubling chain length must roughly double cost: no per-block allocation
 * or super-linear work. min-of-5 timing absorbs scheduler noise. */
static double now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e9 + (double)ts.tv_nsec;
}

static volatile float sink;

static double bench_ns_per_block(int nodes, int iters) {
    Mixer m;
    memset(&m, 0, sizeof(m));
    for (int k = 0; k < nodes; k++)
        chain_add(&m.ch[0], NODE_GAIN, (float[]){1.0f}, 1, 0);

    float inbuf[FRAMES * 2], out[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++) inbuf[i] = 0.1f;
    const float* in[1] = { inbuf };
    Source s = { 0, 1.0f };

    double best = 1e300;
    for (int r = 0; r < 5; r++) {
        double t0 = now_ns();
        for (int it = 0; it < iters; it++) {
            mixer_process(&m, &s, 1, in, out, FRAMES, DEFAULT_SR);
            sink += out[0];
        }
        double per = (now_ns() - t0) / iters;
        if (per < best) best = per;
    }
    return best;
}

static void test_linear_cost(void) {
    enum { K = 8, ITERS = 30000 };
    double t_k = bench_ns_per_block(K, ITERS);
    double t_2k = bench_ns_per_block(2 * K, ITERS);
    double ratio = t_2k / t_k;
    printf("chain_%d_ns_per_block=%.1f chain_%d_ns_per_block=%.1f ratio=%.2f\n",
           K, t_k, 2 * K, t_2k, ratio);
    check("linear_cost", ratio > 0.0 && ratio < 3.0f);
}

int main(void) {
    test_equivalence();
    test_order_and_params();
    test_state_roundtrip();
    test_bypass_no_click();
    test_bypass_restores();
    test_linear_cost();
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
