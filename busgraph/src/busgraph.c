
/* busgraph: data-driven signal bus graph.

   The old mixer hardwired three buses and a duck sidechain in C, so adding a
   send meant editing code. Here the topology is data: nodes and edges are
   declared in config arrays, the graph is rebuilt into an executable schedule,
   and processing just walks that schedule.

   Capabilities:
     - nodes: summing buses, source inputs, delay/return nodes, a sidechain
       duck (control) node and a master output; edges are audio fan-outs with
       per-edge gain, or control links that carry no audio;
     - Kahn topological sort in declaration order; any leftover nodes form a
       cycle and the build fails naming them;
     - send/return are plain fan-outs, sidechains are plain control edges;
     - plug-in delay compensation: every edge is delayed so signals that
       travel paths of different intrinsic latency arrive aligned;
     - rebuild (topology) cost and per-block execute cost are timed apart.

   All buffers are interleaved stereo float, matching the mixer module. */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define FRAMES        128
#define CHANS         2
#define SAMP          (FRAMES * CHANS)
#define MAX_NODES     32
#define MAX_EDGES     64
#define MAX_DELAY     4096   /* frames, bounds the ring buffers */
#define ERR_MAX       160
#define CYCLE_MAX     16

typedef enum {
    BG_SOURCE = 0,   /* external interleaved stereo input, multiplied by gain */
    BG_BUS,          /* zeroed each block, sums incoming audio edges */
    BG_DELAY,        /* intrinsic latency: delays its input by d frames */
    BG_DUCK,         /* control node: control-edge energy sets a scalar gain */
    BG_MASTER        /* sums incoming audio, applies soft clip, graph output */
} BgNodeType;

typedef struct {
    int         id;
    BgNodeType  type;
    float       gain;        /* BG_SOURCE: input gain */
    int         delay;       /* BG_DELAY: intrinsic latency in frames */
    float       duck_depth;  /* BG_DUCK: 0..1 attenuation while control active */
    float       duck_thresh; /* BG_DUCK: mean |sample| activity threshold */
    int         target;      /* BG_DUCK: node id whose audio gets attenuated */
} BgNodeCfg;

typedef enum { BG_AUDIO = 0, BG_CONTROL } BgEdgeType;

typedef struct {
    int         from;
    int         to;
    BgEdgeType  type;
    float       gain;
} BgEdgeCfg;

typedef struct {
    BgNodeCfg cfg;
    int       indeg, outdeg;
    int       edge_in[MAX_EDGES], n_in;
    int       edge_out[MAX_EDGES], n_out;
    int       ctrl_edge;             /* edge feeding a duck node, or -1 */
    int       total_latency;         /* D(n): longest source-to-n latency */
    float     duck_gain;             /* scalar computed each block */
    float     buf[SAMP];
    float     dline[MAX_DELAY * CHANS];  /* BG_DELAY intrinsic delay line */
    float     dscratch[SAMP];        /* private wrap-staging for dline reads */
    int       dw;
} BgNode;

typedef struct {
    BgEdgeCfg cfg;
    int       comp_delay;            /* compensation frames (PDC) */
    float     line[MAX_DELAY * CHANS];  /* per-edge compensation line */
    float     scratch[SAMP];         /* private wrap-staging, consumed at once */
    int       w;
} BgEdge;

typedef struct {
    BgNode nodes[MAX_NODES];
    int    n_nodes;
    BgEdge edges[MAX_EDGES];
    int    n_edges;
    int    order[MAX_NODES];
    int    n_order;
    int    master;                   /* node index, -1 if absent */
    int    ready;
    char   err[ERR_MAX];
} BgGraph;

/* ---------------- build ---------------- */

static int find_node(const BgGraph* g, int id) {
    for (int i = 0; i < g->n_nodes; i++)
        if (g->nodes[i].cfg.id == id) return i;
    return -1;
}

static void bg_seterr(BgGraph* g, const char* msg) {
    if (g->err[0] == '\0') {
        strncpy(g->err, msg, ERR_MAX - 1);
        g->err[ERR_MAX - 1] = '\0';
    }
}

/* Rebuild the executable schedule from a topology description. Returns -1 and
   fills g->err on any structural problem; 0 on success. */
static int bg_build(BgGraph* g, const BgNodeCfg* nc, int nn,
                    const BgEdgeCfg* ec, int ne) {
    memset(g, 0, sizeof(*g));
    g->master = -1;

    if (nn <= 0 || nn > MAX_NODES) { bg_seterr(g, "node count out of range"); return -1; }
    if (ne < 0 || ne > MAX_EDGES)  { bg_seterr(g, "edge count out of range"); return -1; }

    g->n_nodes = nn;
    g->n_edges = ne;
    for (int i = 0; i < nn; i++) {
        g->nodes[i].cfg = nc[i];
        g->nodes[i].ctrl_edge = -1;
    }
    for (int i = 0; i < ne; i++) g->edges[i].cfg = ec[i];

    for (int i = 0; i < nn; i++) {
        for (int j = i + 1; j < nn; j++)
            if (nc[i].id == nc[j].id) { bg_seterr(g, "duplicate node id"); return -1; }
        if (nc[i].type == BG_MASTER) {
            if (g->master != -1) { bg_seterr(g, "more than one master node"); return -1; }
            g->master = i;
        }
    }
    if (g->master == -1) { bg_seterr(g, "graph has no master node"); return -1; }

    for (int e = 0; e < ne; e++) {
        int u = find_node(g, ec[e].from);
        int v = find_node(g, ec[e].to);
        if (u < 0 || v < 0) { bg_seterr(g, "edge references unknown node"); return -1; }
        if (u == v)         { bg_seterr(g, "self loop on node"); return -1; }
        BgNode* src = &g->nodes[u];
        BgNode* dst = &g->nodes[v];
        if (src->n_out >= MAX_EDGES || dst->n_in >= MAX_EDGES) {
            bg_seterr(g, "node degree exceeds capacity"); return -1;
        }
        src->edge_out[src->n_out++] = e;
        dst->edge_in[dst->n_in++] = e;
        src->outdeg++;
        dst->indeg++;
        if (ec[e].type == BG_CONTROL) {
            if (dst->cfg.type != BG_DUCK) { bg_seterr(g, "control edge must target a duck node"); return -1; }
            if (dst->ctrl_edge != -1)     { bg_seterr(g, "duck node has more than one control edge"); return -1; }
            dst->ctrl_edge = e;
        } else {
            if (dst->cfg.type == BG_DUCK) { bg_seterr(g, "duck node takes only a control edge"); return -1; }
            if (src->cfg.type == BG_DUCK || src->cfg.type == BG_MASTER) {
                bg_seterr(g, "duck/master nodes emit no audio"); return -1;
            }
        }
    }
    for (int i = 0; i < nn; i++) {
        if (nc[i].type == BG_DUCK && g->nodes[i].ctrl_edge == -1) {
            bg_seterr(g, "duck node missing control edge"); return -1;
        }
        if (nc[i].type == BG_SOURCE && g->nodes[i].n_in != 0) {
            bg_seterr(g, "source node cannot have inputs"); return -1;
        }
        if (nc[i].type == BG_DELAY && (nc[i].delay < 0 || nc[i].delay > MAX_DELAY)) {
            bg_seterr(g, "delay out of range"); return -1;
        }
    }

    /* Kahn topological sort; declaration-order scan keeps it deterministic. */
    int indeg[MAX_NODES];
    for (int i = 0; i < nn; i++) indeg[i] = g->nodes[i].indeg;
    g->n_order = 0;
    for (;;) {
        int pick = -1;
        for (int i = 0; i < nn; i++)
            if (indeg[i] == 0) { pick = i; break; }
        if (pick < 0) break;
        indeg[pick] = -1;
        g->order[g->n_order++] = pick;
        for (int k = 0; k < g->nodes[pick].n_out; k++) {
            int e = g->nodes[pick].edge_out[k];
            indeg[find_node(g, g->edges[e].cfg.to)]--;
        }
    }
    if (g->n_order != nn) {
        const char* prefix = "topological cycle detected among nodes: ";
        enum { PFX_LEN = 38 };
        char cyc[ERR_MAX - PFX_LEN];   /* prefix + cyc always fits in g->err */
        int pos = 0, listed = 0;
        cyc[0] = '\0';
        for (int i = 0; i < nn && listed < CYCLE_MAX; i++) {
            if (indeg[i] >= 0) {
                int room = (int)sizeof(cyc) - pos - 1;
                if (room <= 4) break;                 /* leave room for "..." */
                pos += snprintf(cyc + pos, (size_t)room, "%s%d",
                                listed ? "->" : "", nc[i].id);
                listed++;
            }
        }
        snprintf(g->err, ERR_MAX, "%s%s", prefix, cyc);
        return -1;
    }

    /* D(n): longest intrinsic latency, in frames, from any source to n's
       output. Forward topo order keeps every predecessor's D known; a node's
       own delay is added once on its inputs. */
    for (int k = 0; k < nn; k++) {
        BgNode* n = &g->nodes[g->order[k]];
        int own = n->cfg.type == BG_DELAY ? n->cfg.delay : 0;
        int best = 0;
        for (int j = 0; j < n->n_in; j++) {
            int e = n->edge_in[j];
            int u = find_node(g, g->edges[e].cfg.from);
            int cand = g->nodes[u].total_latency + own;
            if (cand > best) best = cand;
        }
        n->total_latency = best;   /* source nodes have no inputs and stay 0 */
    }

    /* PDC on edge u->v: hold u's signal so every input of v arrives aligned
       to v's depth. d = D(v) - L(v) - D(u), non-negative by construction. */
    for (int e = 0; e < ne; e++) {
        int u = find_node(g, ec[e].from);
        int v = find_node(g, ec[e].to);
        int l_v = g->nodes[v].cfg.type == BG_DELAY ? g->nodes[v].cfg.delay : 0;
        int d = g->nodes[v].total_latency - l_v - g->nodes[u].total_latency;
        if (d < 0 || d > MAX_DELAY) { bg_seterr(g, "delay compensation out of range"); return -1; }
        g->edges[e].comp_delay = d;
    }

    g->ready = 1;
    return 0;
}

/* ---------------- execution ---------------- */

static float soft_clip(float v) {
    const float k = 0.9f;
    float a = fabsf(v);
    if (a <= k) return v;
    float s = v < 0 ? -1.0f : 1.0f;
    return s * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

/* Compensated view of an upstream node buffer through edge e. A zero-delay
   edge returns the buffer directly, so delay-free paths do no buffering at all
   (this is what keeps them bit-identical to the hardcoded mixer). */
static const float* edge_pull(BgGraph* g, int e, const float* src) {
    BgEdge* ed = &g->edges[e];
    int d = ed->comp_delay * CHANS;
    if (d == 0) return src;
    float* line = ed->line;
    int cap = MAX_DELAY * CHANS;
    int w = ed->w;
    for (int i = 0; i < SAMP; i++) {
        line[w] = src[i];
        if (++w == cap) w = 0;
    }
    ed->w = w;
    /* d history samples sit SAMP+d slots behind the just-written cursor */
    int r = (w - SAMP - d) % cap;
    if (r < 0) r += cap;
    if (r + SAMP <= cap) return line + r;
    /* wrap straddles the ring: stage a contiguous copy in this edge's scratch */
    int first = cap - r;
    memcpy(ed->scratch, line + r, (size_t)first * sizeof(float));
    memcpy(ed->scratch + first, line, (size_t)(SAMP - first) * sizeof(float));
    return ed->scratch;
}

/* Gain of an audio edge leaving node u: declared edge gain times the scalar
   published by any duck node targeting u. */
static float edge_gain(const BgGraph* g, int u, float base) {
    float gain = base;
    for (int i = 0; i < g->n_nodes; i++) {
        const BgNode* d = &g->nodes[i];
        if (d->cfg.type == BG_DUCK && d->cfg.target == g->nodes[u].cfg.id)
            gain *= d->duck_gain;
    }
    return gain;
}

/* Process one block. in[] is one pointer per node index; only BG_SOURCE
   entries are read (NULL means silence). Master output is written to out. */
static int bg_execute(BgGraph* g, const float* const* in, float* out) {
    if (!g->ready) { bg_seterr(g, "graph not built"); return -1; }

    for (int i = 0; i < g->n_nodes; i++) g->nodes[i].duck_gain = 1.0f;

    for (int k = 0; k < g->n_order; k++) {
        BgNode* n = &g->nodes[g->order[k]];
        switch (n->cfg.type) {
        case BG_SOURCE: {
            const float* src = in ? in[n - g->nodes] : NULL;
            if (src) {
                for (int i = 0; i < SAMP; i++) n->buf[i] = src[i] * n->cfg.gain;
            } else {
                memset(n->buf, 0, sizeof(n->buf));
            }
            break;
        }
        case BG_BUS: {
            memset(n->buf, 0, sizeof(n->buf));
            for (int j = 0; j < n->n_in; j++) {
                int e = n->edge_in[j];
                int u = find_node(g, g->edges[e].cfg.from);
                const float* s = edge_pull(g, e, g->nodes[u].buf);
                float ge = g->edges[e].cfg.gain;
                for (int i = 0; i < SAMP; i++) n->buf[i] += s[i] * ge;
            }
            break;
        }
        case BG_DELAY: {
            /* sum inputs like a bus, then apply the intrinsic delay */
            float sum[SAMP];
            memset(sum, 0, sizeof(sum));
            for (int j = 0; j < n->n_in; j++) {
                int e = n->edge_in[j];
                int u = find_node(g, g->edges[e].cfg.from);
                const float* s = edge_pull(g, e, g->nodes[u].buf);
                float ge = g->edges[e].cfg.gain;
                for (int i = 0; i < SAMP; i++) sum[i] += s[i] * ge;
            }
            int d = n->cfg.delay * CHANS;
            int cap = MAX_DELAY * CHANS;
            int w = n->dw;
            for (int i = 0; i < SAMP; i++) {
                n->dline[w] = sum[i];
                if (++w == cap) w = 0;
            }
            n->dw = w;
            if (d == 0) {
                memcpy(n->buf, sum, sizeof(sum));
            } else {
                int r = (w - SAMP - d) % cap;
                if (r < 0) r += cap;
                if (r + SAMP <= cap) {
                    memcpy(n->buf, n->dline + r, SAMP * sizeof(float));
                } else {
                    int first = cap - r;
                    memcpy(n->dscratch, n->dline + r, (size_t)first * sizeof(float));
                    memcpy(n->dscratch + first, n->dline, (size_t)(SAMP - first) * sizeof(float));
                    memcpy(n->buf, n->dscratch, SAMP * sizeof(float));
                }
            }
            break;
        }
        case BG_DUCK: {
            int e = n->ctrl_edge;
            int u = find_node(g, g->edges[e].cfg.from);
            const float* s = edge_pull(g, e, g->nodes[u].buf);
            float energy = 0.0f;
            for (int i = 0; i < SAMP; i++) energy += fabsf(s[i]);
            energy /= (float)SAMP;
            if (energy > n->cfg.duck_thresh) n->duck_gain = 1.0f - n->cfg.duck_depth;
            break;
        }
        case BG_MASTER: {
            memset(n->buf, 0, sizeof(n->buf));
            for (int j = 0; j < n->n_in; j++) {
                int e = n->edge_in[j];
                int u = find_node(g, g->edges[e].cfg.from);
                const float* s = edge_pull(g, e, g->nodes[u].buf);
                float ge = edge_gain(g, u, g->edges[e].cfg.gain);
                for (int i = 0; i < SAMP; i++) n->buf[i] += s[i] * ge;
            }
            for (int i = 0; i < SAMP; i++) out[i] = soft_clip(n->buf[i]);
            break;
        }
        }
    }
    return 0;
}

/* ===================================================================== */
/* self test                                                              */
/* ===================================================================== */

/* ---- exact copy of the original hardcoded mixer, the parity reference ---- */
typedef struct { int bus; float gain; } RefSource;
typedef struct { float gain[3]; float duck_depth; int duck_enabled; } RefMixer;

static float ref_soft_clip(float v) {
    const float k = 0.9f;
    float a = fabsf(v);
    if (a <= k) return v;
    float s = v < 0 ? -1.0f : 1.0f;
    return s * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

static void ref_mix(const RefMixer* mx, const RefSource* srcs, int n,
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
        out[i] = ref_soft_clip(v);
    }
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static long long now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/* node ids for the mixer-equivalent topology */
enum { N_SFX = 1, N_MUSIC = 2, N_VOICE = 3,
       B_SFX = 4, B_MUSIC = 5, B_VOICE = 6,
       DUCK = 7, OUT = 8 };

static int build_mixer_graph(BgGraph* g, int duck_enabled, float depth) {
    BgNodeCfg nc[] = {
        { N_SFX,   BG_SOURCE, 1.0f, 0, 0, 0, 0 },
        { N_MUSIC, BG_SOURCE, 1.0f, 0, 0, 0, 0 },
        { N_VOICE, BG_SOURCE, 1.0f, 0, 0, 0, 0 },
        { B_SFX,   BG_BUS, 0, 0, 0, 0, 0 },
        { B_MUSIC, BG_BUS, 0, 0, 0, 0, 0 },
        { B_VOICE, BG_BUS, 0, 0, 0, 0, 0 },
        { DUCK,    BG_DUCK, 0, 0, duck_enabled ? depth : 0.0f, 0.01f, B_SFX },
        { OUT,     BG_MASTER, 0, 0, 0, 0, 0 },
    };
    BgEdgeCfg ec[] = {
        { N_SFX,   B_SFX,   BG_AUDIO,   1.0f },
        { N_MUSIC, B_MUSIC, BG_AUDIO,   1.0f },
        { N_VOICE, B_VOICE, BG_AUDIO,   1.0f },
        { B_SFX,   OUT,     BG_AUDIO,   1.0f },
        { B_MUSIC, OUT,     BG_AUDIO,   1.0f },
        { B_VOICE, OUT,     BG_AUDIO,   1.0f },
        { N_VOICE, DUCK,    BG_CONTROL, 0.0f },
    };
    /* a disabled duck never trips: give it an unreachable threshold */
    if (!duck_enabled) nc[6].duck_thresh = 1.0e30f;
    return bg_build(g, nc, (int)(sizeof(nc) / sizeof(nc[0])),
                    ec, (int)(sizeof(ec) / sizeof(ec[0])));
}

int main(void) {
    static BgGraph g;
    float sfx[SAMP], music[SAMP], voice[SAMP];
    float gout[SAMP], rout[SAMP];

    /* ---------- 1. parity against the hardcoded mixer ---------- */
    RefSource ref_src[3] = { {0, 1.0f}, {1, 1.0f}, {2, 1.0f} };
    const float* ref_in[3] = { sfx, music, voice };

    struct { float sg, mg, vg, vval, depth; int duck; } cases[] = {
        { 1.0f, 1.0f, 1.0f, 0.0f,  0.5f, 1 },  /* plain sum */
        { 1.0f, 1.0f, 1.0f, 0.3f,  0.5f, 1 },  /* duck active */
        { 1.0f, 1.0f, 1.0f, 0.3f,  0.5f, 0 },  /* duck disabled */
        { 4.0f, 4.0f, 4.0f, 0.5f,  0.5f, 1 },  /* soft clip region */
        { 0.7f, 1.3f, 0.2f, 0.0f,  0.5f, 1 },  /* uneven gains */
        { 2.5f, 0.4f, 1.1f, 0.09f, 0.35f, 1 }, /* just below threshold */
        { 2.5f, 0.4f, 1.1f, 0.11f, 0.35f, 1 }, /* just above threshold */
    };
    int parity_ok = 1;
    for (int c = 0; c < (int)(sizeof(cases) / sizeof(cases[0])); c++) {
        for (int i = 0; i < SAMP; i++) {
            sfx[i] = 0.5f; music[i] = 0.25f; voice[i] = cases[c].vval;
        }
        RefMixer rm = { {1.0f, 1.0f, 1.0f}, cases[c].depth, cases[c].duck };
        ref_src[0].gain = cases[c].sg;
        ref_src[1].gain = cases[c].mg;
        ref_src[2].gain = cases[c].vg;
        ref_mix(&rm, ref_src, 3, ref_in, rout, FRAMES);

        if (build_mixer_graph(&g, cases[c].duck, cases[c].depth) != 0) { parity_ok = 0; break; }
        /* graph source gains mirror the reference source gains */
        g.nodes[find_node(&g, N_SFX)].cfg.gain = cases[c].sg;
        g.nodes[find_node(&g, N_MUSIC)].cfg.gain = cases[c].mg;
        g.nodes[find_node(&g, N_VOICE)].cfg.gain = cases[c].vg;
        const float* gin[8];
        memset(gin, 0, sizeof(gin));
        gin[find_node(&g, N_SFX)] = sfx;
        gin[find_node(&g, N_MUSIC)] = music;
        gin[find_node(&g, N_VOICE)] = voice;
        bg_execute(&g, gin, gout);
        if (memcmp(gout, rout, sizeof(gout)) != 0) {
            parity_ok = 0;
            printf("  parity mismatch on case %d\n", c);
            break;
        }
    }
    check("parity_per_sample_exact", parity_ok);

    /* fuzz: random gains and per-sample values, rebuilt once, streamed blocks */
    srand(1234);
    int fuzz_ok = 1;
    build_mixer_graph(&g, 1, 0.5f);
    for (int blk = 0; blk < 64 && fuzz_ok; blk++) {
        float gs = 0.2f + 3.0f * (float)rand() / (float)RAND_MAX;
        float gm = 0.2f + 3.0f * (float)rand() / (float)RAND_MAX;
        float gv = 0.2f + 3.0f * (float)rand() / (float)RAND_MAX;
        for (int i = 0; i < SAMP; i++) {
            sfx[i]   = 2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f;
            music[i] = 2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f;
            voice[i] = 2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f;
        }
        RefMixer rm = { {1.0f, 1.0f, 1.0f}, 0.5f, 1 };
        ref_src[0].gain = gs; ref_src[1].gain = gm; ref_src[2].gain = gv;
        ref_mix(&rm, ref_src, 3, ref_in, rout, FRAMES);

        g.nodes[find_node(&g, N_SFX)].cfg.gain = gs;
        g.nodes[find_node(&g, N_MUSIC)].cfg.gain = gm;
        g.nodes[find_node(&g, N_VOICE)].cfg.gain = gv;
        const float* gin[8];
        memset(gin, 0, sizeof(gin));
        gin[find_node(&g, N_SFX)] = sfx;
        gin[find_node(&g, N_MUSIC)] = music;
        gin[find_node(&g, N_VOICE)] = voice;
        bg_execute(&g, gin, gout);
        if (memcmp(gout, rout, sizeof(gout)) != 0) fuzz_ok = 0;
    }
    check("parity_fuzz_stream_exact", fuzz_ok);

    /* invalid reference bus is ignored in the mixer; here an unbound source
       node simply emits silence, which must leave the output well-behaved. */
    build_mixer_graph(&g, 0, 0.5f);
    {
        const float* gin[8];
        memset(gin, 0, sizeof(gin));   /* all sources silent */
        bg_execute(&g, gin, gout);
        float pk = 0.0f;
        for (int i = 0; i < SAMP; i++) if (fabsf(gout[i]) > pk) pk = fabsf(gout[i]);
        check("silent_sources_zero", pk == 0.0f);
    }

    /* ---------- 2. cycle and structural validation ---------- */
    {
        static BgGraph bad;
        BgNodeCfg nc[] = {
            { 1, BG_BUS, 0, 0, 0, 0, 0 },
            { 2, BG_BUS, 0, 0, 0, 0, 0 },
            { 3, BG_BUS, 0, 0, 0, 0, 0 },
            { 9, BG_MASTER, 0, 0, 0, 0, 0 },
        };
        BgEdgeCfg ec[] = {
            { 1, 2, BG_AUDIO, 1 }, { 2, 3, BG_AUDIO, 1 },
            { 3, 1, BG_AUDIO, 1 }, { 3, 9, BG_AUDIO, 1 },
        };
        int rc = bg_build(&bad, nc, 4, ec, 4);
        check("cycle_rejected", rc == -1 && strstr(bad.err, "cycle") != NULL);
        printf("cycle_err=%s\n", bad.err);
    }
    {
        static BgGraph bad;
        BgNodeCfg nc[] = { { 1, BG_BUS, 0, 0, 0, 0, 0 }, { 9, BG_MASTER, 0, 0, 0, 0, 0 } };
        BgEdgeCfg ec[] = { { 1, 77, BG_AUDIO, 1 } };
        check("unknown_node_rejected", bg_build(&bad, nc, 2, ec, 1) == -1);
    }
    {
        static BgGraph bad;
        BgNodeCfg nc[] = {
            { 1, BG_BUS, 0, 0, 0, 0, 0 }, { 1, BG_BUS, 0, 0, 0, 0, 0 },
            { 9, BG_MASTER, 0, 0, 0, 0, 0 },
        };
        BgEdgeCfg ec[] = { { 1, 9, BG_AUDIO, 1 } };
        check("duplicate_id_rejected", bg_build(&bad, nc, 3, ec, 1) == -1);
    }
    {
        static BgGraph bad;
        BgNodeCfg nc[] = {
            { 1, BG_SOURCE, 1, 0, 0, 0, 0 },
            { 2, BG_BUS, 0, 0, 0, 0, 0 },
            { 3, BG_DUCK, 0, 0, 0.5f, 0.01f, 2 },
            { 9, BG_MASTER, 0, 0, 0, 0, 0 },
        };
        BgEdgeCfg ec[] = { { 1, 3, BG_AUDIO, 1 }, { 2, 9, BG_AUDIO, 1 } };
        check("audio_into_duck_rejected", bg_build(&bad, nc, 4, ec, 2) == -1);
    }
    {
        static BgGraph bad;
        BgNodeCfg nc[] = { { 1, BG_BUS, 0, 0, 0, 0, 0 } };
        check("missing_master_rejected", bg_build(&bad, nc, 1, NULL, 0) == -1);
    }

    /* ---------- 3. send/return with an extra bus (no code change) ---------- */
    {
        static BgGraph sr;
        enum { S1 = 1, MAIN = 2, RET = 3, SEND_BUS = 4, M = 5 };
        BgNodeCfg nc[] = {
            { S1,       BG_SOURCE, 1.0f, 0, 0, 0, 0 },
            { MAIN,     BG_BUS, 0, 0, 0, 0, 0 },
            { RET,      BG_DELAY, 0, 2, 0, 0, 0 },      /* 2-frame return */
            { SEND_BUS, BG_BUS, 0, 0, 0, 0, 0 },
            { M,        BG_MASTER, 0, 0, 0, 0, 0 },
        };
        BgEdgeCfg ec[] = {
            { S1, MAIN,     BG_AUDIO, 1.0f },  /* dry path  */
            { S1, SEND_BUS, BG_AUDIO, 1.0f },  /* send tap  */
            { SEND_BUS, RET, BG_AUDIO, 1.0f }, /* into return */
            { RET, M,       BG_AUDIO, 1.0f },
            { MAIN, M,      BG_AUDIO, 1.0f },
        };
        int sr_rc = bg_build(&sr, nc, 5, ec, 5);
        check("send_return_builds", sr_rc == 0);
        /* PDC: at the master the dry input is held the return's 2 frames so it
           aligns with the wet path; edges upstream of the fork stay delay-free */
        check("send_return_pdc",
              sr.edges[0].comp_delay == 0 && sr.edges[1].comp_delay == 0 &&
              sr.edges[2].comp_delay == 0 && sr.edges[3].comp_delay == 0 &&
              sr.edges[4].comp_delay == 2);

        float in[SAMP], out[SAMP];
        memset(in, 0, sizeof(in));
        in[0] = 0.4f; in[1] = 0.4f;           /* sub-knee impulse at frame 0 */
        const float* gin[5] = {0};
        gin[0] = in;
        bg_execute(&sr, gin, out);
        /* dry (delayed 2 by PDC) and wet (intrinsic 2) land together: frame 2 */
        check("send_return_aligned",
              out[4] == 0.8f && out[5] == 0.8f && out[0] == 0.0f && out[2] == 0.0f);

        /* stream a second silent block: ring state must not leak the impulse */
        memset(in, 0, sizeof(in));
        bg_execute(&sr, gin, out);
        float pk = 0.0f;
        for (int i = 0; i < SAMP; i++) if (fabsf(out[i]) > pk) pk = fabsf(out[i]);
        check("send_return_settles", pk == 0.0f);
    }

    /* ---------- 4. unequal-latency buses auto-align across block edges ------ */
    {
        static BgGraph pdc;
        enum { PS = 1, FAST = 2, SLOW = 3, PM = 4 };
        BgNodeCfg nc[] = {
            { PS,   BG_SOURCE, 1.0f, 0, 0, 0, 0 },
            { FAST, BG_BUS, 0, 0, 0, 0, 0 },
            { SLOW, BG_DELAY, 0, 10, 0, 0, 0 },     /* 10-frame intrinsic lag */
            { PM,   BG_MASTER, 0, 0, 0, 0, 0 },
        };
        BgEdgeCfg ec[] = {
            { PS, FAST, BG_AUDIO, 1.0f },
            { PS, SLOW, BG_AUDIO, 1.0f },
            { FAST, PM, BG_AUDIO, 1.0f },
            { SLOW, PM, BG_AUDIO, 1.0f },
        };
        int pdc_rc = bg_build(&pdc, nc, 4, ec, 4);
        check("pdc_builds", pdc_rc == 0);
        check("pdc_compensation",
              pdc.edges[2].comp_delay == 10 &&   /* fast bus held 10 frames at master */
              pdc.edges[3].comp_delay == 0 &&    /* slow bus plays directly */
              pdc.nodes[find_node(&pdc, SLOW)].total_latency == 10 &&
              pdc.nodes[find_node(&pdc, PM)].total_latency == 10);

        float in[SAMP], out[SAMP];
        const float* gin[4] = {0};
        gin[0] = in;
        memset(in, 0, sizeof(in));
        in[0] = in[1] = 0.4f;                  /* sub-knee impulse frame 0, block 0 */
        bg_execute(&pdc, gin, out);
        int aligned = (out[20] == 0.8f && out[21] == 0.8f);   /* frame 10 */
        for (int i = 0; i < SAMP; i++)
            if (i != 20 && i != 21 && out[i] != 0.0f) aligned = 0;
        check("pdc_aligned_in_block", aligned);

        /* impulse near the block end crosses the boundary, still aligned */
        memset(in, 0, sizeof(in));
        in[SAMP - 2] = in[SAMP - 1] = 0.4f;    /* impulse at last frame */
        bg_execute(&pdc, gin, out);            /* block 1: fully consumed */
        float b1pk = 0.0f;
        for (int i = 0; i < SAMP; i++) if (fabsf(out[i]) > b1pk) b1pk = fabsf(out[i]);
        check("pdc_block_end_silent", b1pk == 0.0f);
        memset(in, 0, sizeof(in));
        bg_execute(&pdc, gin, out);            /* block 2: lands at frame 9 */
        int cross = (out[18] == 0.8f && out[19] == 0.8f);
        for (int i = 0; i < SAMP; i++)
            if (i != 18 && i != 19 && out[i] != 0.0f) cross = 0;
        check("pdc_aligned_across_blocks", cross);
    }

    /* ---------- 5. rebuild vs execute timed separately ---------- */
    {
        static BgGraph tg;
        BgNodeCfg nc_a[] = {
            { 1, BG_SOURCE, 1, 0, 0, 0, 0 }, { 2, BG_BUS, 0, 0, 0, 0, 0 },
            { 3, BG_BUS, 0, 0, 0, 0, 0 },    { 9, BG_MASTER, 0, 0, 0, 0, 0 },
        };
        BgEdgeCfg ec_a[] = {
            { 1, 2, BG_AUDIO, 1 }, { 2, 9, BG_AUDIO, 1 }, { 3, 9, BG_AUDIO, 1 },
        };
        /* topology B: one extra send/return node and edge */
        BgNodeCfg nc_b[] = {
            { 1, BG_SOURCE, 1, 0, 0, 0, 0 }, { 2, BG_BUS, 0, 0, 0, 0, 0 },
            { 3, BG_BUS, 0, 0, 0, 0, 0 },    { 4, BG_DELAY, 0, 4, 0, 0, 0 },
            { 9, BG_MASTER, 0, 0, 0, 0, 0 },
        };
        BgEdgeCfg ec_b[] = {
            { 1, 2, BG_AUDIO, 1 }, { 2, 9, BG_AUDIO, 1 }, { 3, 9, BG_AUDIO, 1 },
            { 1, 4, BG_AUDIO, 1 }, { 4, 9, BG_AUDIO, 1 },
        };
        enum { REPS = 500, BLOCKS = 100000 };
        long long t0 = now_ns();
        for (int r = 0; r < REPS; r++) bg_build(&tg, nc_a, 4, ec_a, 3);
        long long t1 = now_ns();
        for (int r = 0; r < REPS; r++) bg_build(&tg, nc_b, 5, ec_b, 5);
        long long t2 = now_ns();

        float in[SAMP], out[SAMP];
        for (int i = 0; i < SAMP; i++) in[i] = 0.3f;
        const float* gin[5] = {0};
        gin[0] = in;
        bg_build(&tg, nc_b, 5, ec_b, 5);
        long long t3 = now_ns();
        for (int b = 0; b < BLOCKS; b++) bg_execute(&tg, gin, out);
        long long t4 = now_ns();
        printf("rebuild_topologyA_avg_ns=%lld\n", (t1 - t0) / REPS);
        printf("rebuild_topologyB_avg_ns=%lld\n", (t2 - t1) / REPS);
        printf("execute_per_block_avg_ns=%lld\n", (t4 - t3) / BLOCKS);
        check("timing_separated", (t1 - t0) > 0 && (t4 - t3) > 0);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
