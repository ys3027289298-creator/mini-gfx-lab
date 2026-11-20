/* mini-render-chain: full binaural render chain.

   Four stages run in series, in a configurable order and with a per-stage
   gain:

       HRTF convolution  (ITD + IID + pinna tap, built from the azimuth)
       room reverb       (4 combs / 2 allpasses per ear, wet/dry)
       Doppler           (radial velocity -> variable-rate delay line)
       distance          (inverse-distance law, ref / max_gain clamps)

   All stages keep persistent state, so a configuration renders the same
   whatever block size it is driven with; that is what makes the serial chain
   click-free at block seams. Every stage is linear, so per-stage gains
   multiply through the chain.

   The self-test compares the streaming chain against a reference that applies
   each stage once over the whole buffer ("by hand" per stage), checks stage
   semantics against closed forms, and reports the cost of the four stages
   together versus each stage alone.

   Build:  make            (from spatial/) -> ./render_chain
   Run:    ./render_chain
*/
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define SR 48000
#define MAXF 2048
#define MAXTAPS 64
#define HIST 8192

#define NCOMB 4
#define NAP 2
#define COMBLEN 1600
#define APLEN 800

#define DOP_P0 256.0f   /* initial delay: headroom for approaching sources (5.3 ms) */

/* ---------------- configuration ---------------- */

typedef enum { ST_HRTF = 0, ST_REVERB, ST_DOPPLER, ST_DISTANCE, ST_N } Stage;

#define MODE_HRTF    (1u << ST_HRTF)
#define MODE_REVERB  (1u << ST_REVERB)
#define MODE_DOPPLER (1u << ST_DOPPLER)
#define MODE_DIST    (1u << ST_DISTANCE)
#define MODE_ALL     (MODE_HRTF | MODE_REVERB | MODE_DOPPLER | MODE_DIST)

typedef struct {
    float x, y, z;
    float vx, vy, vz;
    float gain;
} Source;

typedef struct {
    float x, y, z;
    float vx, vy, vz;
    float yaw;
} Listener;

typedef struct {
    float itd_ms;       /* maximum interaural delay, ms */
    float pinna;        /* strength of the pinna reflection tap */
    int   pinna_offset; /* tap offset after the main tap, samples */
} HrtfCfg;

typedef struct {
    float wet;          /* 0..1 */
    float decay;        /* comb feedback, < 1 */
} ReverbCfg;

typedef struct {
    float speed;        /* speed of sound, m/s */
    float smoothing;    /* one-pole rate for the shift, 0 = instant, <1 */
} DopplerCfg;

typedef struct {
    float ref;          /* unity-gain distance */
    float rolloff;      /* 0 disables distance attenuation */
    float max_gain;     /* near-field ceiling */
} DistanceCfg;

typedef struct {
    unsigned stages;    /* MODE_* bitmask */
    Stage    order[ST_N];
    float    gain[ST_N];    /* applied after each stage */
    HrtfCfg     hrtf;
    ReverbCfg   reverb;
    DopplerCfg  doppler;
    DistanceCfg distance;
} ChainConfig;

typedef struct {
    float comb[2][NCOMB][COMBLEN];
    int   cidx[2][NCOMB];
    int   clen[2][NCOMB];
    float ap[2][NAP][APLEN];
    int   aidx[2][NAP];
    int   alen[2][NAP];
} ReverbState;

typedef struct {
    float buf[2][HIST];
    int   w;
    float p;            /* read offset behind the write head, samples */
    float ratio;        /* smoothed playback rate */
    int   init;         /* p seeded from the propagation delay yet? */
} DopState;

typedef struct {
    float hx[2][MAXTAPS];   /* HRTF input history per ear */
    int   hw[2];
    ReverbState rev;
    DopState    dop;
} ChainState;

/* ---------------- geometry (matches spatial.c) ---------------- */

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* 0 = front, +pi/2 = right. Listener yaw rotates the frame. */
static float azimuth_of(const Source* s, const Listener* l) {
    float dx = s->x - l->x, dz = s->z - l->z;
    float c = cosf(l->yaw), sn = sinf(l->yaw);
    float xr =  c * dx - sn * dz;
    float zr =  sn * dx + c * dz;
    return atan2f(xr, -zr);
}

static float source_distance(const Source* s, const Listener* l) {
    float dx = s->x - l->x, dy = s->y - l->y, dz = s->z - l->z;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

/* radial closing speed: positive when source and listener approach */
static float radial_speed(const Source* s, const Listener* l) {
    float dx = s->x - l->x, dy = s->y - l->y, dz = s->z - l->z;
    float d = sqrtf(dx * dx + dy * dy + dz * dz);
    if (d < 1e-6f) return 0.0f;
    dx /= d; dy /= d; dz /= d;
    float vx = s->vx - l->vx, vy = s->vy - l->vy, vz = s->vz - l->vz;
    return -(vx * dx + vy * dy + vz * dz);
}

static float iid_gain_left(float az)  { return sqrtf(0.5f * (1.0f - sinf(az))); }
static float iid_gain_right(float az) { return sqrtf(0.5f * (1.0f + sinf(az))); }

static void iid_gains(float az, float* gl, float* gr) {
    *gl = iid_gain_left(az);
    *gr = iid_gain_right(az);
}

/* ---------------- HRTF ---------------- */

/* Builds the per-ear FIR for the current azimuth. Both ears get the same
   shape: a main tap at the ear's ITD and a small inverted pinna reflection
   after it. Returns the tap count. */
static int hrtf_kernel(const HrtfCfg* cfg, const Listener* l, const Source* s,
                       float h[2][MAXTAPS]) {
    float az = azimuth_of(s, l);
    float max_delay = cfg->itd_ms * 0.001f * (float)SR;
    int d = (int)lroundf(sinf(az) * max_delay);
    if (d > MAXTAPS - cfg->pinna_offset - 2) d = MAXTAPS - cfg->pinna_offset - 2;
    if (d < -(MAXTAPS - cfg->pinna_offset - 2)) d = -(MAXTAPS - cfg->pinna_offset - 2);
    int dl = d > 0 ? d : 0;
    int dr = d < 0 ? -d : 0;
    int off = cfg->pinna_offset < 1 ? 1 : cfg->pinna_offset;
    float gl, gr;
    iid_gains(az, &gl, &gr);

    memset(h, 0, sizeof(float) * 2 * MAXTAPS);
    h[0][dl] = gl;
    h[1][dr] = gr;
    h[0][dl + off] = -cfg->pinna * gl;
    h[1][dr + off] = -cfg->pinna * gr;
    return (dl > dr ? dl : dr) + off + 1;
}

/* Streaming convolution: each ear has a ring of recent input samples. */
static void hrtf_apply(const HrtfCfg* cfg, const Listener* l, const Source* s,
                       ChainState* st, const float (*in)[MAXF], float (*out)[MAXF], int n) {
    float h[2][MAXTAPS];
    int len = hrtf_kernel(cfg, l, s, h);
    for (int ch = 0; ch < 2; ch++) {
        float* hist = st->hx[ch];
        int w = st->hw[ch];
        for (int i = 0; i < n; i++) {
            hist[w] = in[ch][i];
            float acc = 0.0f;
            int idx = w;
            for (int k = 0; k < len; k++) {
                acc += h[ch][k] * hist[idx];
                if (--idx < 0) idx += MAXTAPS;
            }
            out[ch][i] = acc;
            if (++w >= MAXTAPS) w = 0;
        }
        st->hw[ch] = w;
    }
}

/* ---------------- reverb ---------------- */

static const int COMB_BASE[NCOMB] = { 1214, 1292, 1389, 1475 };
static const int AP_BASE[NAP]     = { 605, 480 };
static const float AP_COEF = 0.5f;

static void reverb_reset(ReverbState* r) {
    memset(r, 0, sizeof(*r));
    for (int e = 0; e < 2; e++) {
        int spread = e ? 23 : 0;
        for (int c = 0; c < NCOMB; c++) {
            int len = COMB_BASE[c] + spread;
            r->clen[e][c] = len > COMBLEN ? COMBLEN : len;
        }
        for (int a = 0; a < NAP; a++) {
            int len = AP_BASE[a] + spread;
            r->alen[e][a] = len > APLEN ? APLEN : len;
        }
    }
}

/* One sample through the comb/allpass network. Shared by both ears. */
static float reverb_ear(ReverbState* r, int e, float x, const ReverbCfg* cfg) {
    float acc = 0.0f;
    for (int c = 0; c < NCOMB; c++) {
        int i = r->cidx[e][c];
        float y = r->comb[e][c][i];
        r->comb[e][c][i] = x + y * cfg->decay;
        if (++i >= r->clen[e][c]) i = 0;
        r->cidx[e][c] = i;
        acc += y;
    }
    acc *= 1.0f / (float)NCOMB;
    for (int a = 0; a < NAP; a++) {
        int i = r->aidx[e][a];
        float d = r->ap[e][a][i];
        float y = d - acc;
        r->ap[e][a][i] = acc + d * AP_COEF;
        if (++i >= r->alen[e][a]) i = 0;
        r->aidx[e][a] = i;
        acc = y;
    }
    return acc;
}

static void reverb_apply(const ReverbCfg* cfg, ReverbState* r,
                         const float (*in)[MAXF], float (*out)[MAXF], int n) {
    for (int i = 0; i < n; i++) {
        for (int e = 0; e < 2; e++) {
            float w = reverb_ear(r, e, in[e][i], cfg);
            out[e][i] = in[e][i] * (1.0f - cfg->wet) + w * cfg->wet;
        }
    }
}

/* ---------------- Doppler ---------------- */

/* playback rate > 1 means the source approaches and the pitch rises */
static float doppler_ratio(const DopplerCfg* cfg, const Listener* l, const Source* s) {
    float vr = radial_speed(s, l);
    float r = cfg->speed / (cfg->speed - vr);
    return clampf(r, 0.25f, 4.0f);
}

/* propagation delay in samples: how far back the source's wavefront is */
static float dop_seed(const DopplerCfg* cfg, const Listener* l, const Source* s) {
    float speed = cfg->speed > 1.0f ? cfg->speed : 343.0f;
    return source_distance(s, l) / speed * (float)SR;
}

static int imod(int a, int m) {
    int r = a % m;
    return r < 0 ? r + m : r;
}

/* Fractional-delay resampling. The read offset drifts by 1-ratio per sample;
   the same recurrence runs whether the caller feeds one block or many. */
static void dop_process(DopState* st, float target, float smoothing, float seed,
                        const float (*in)[MAXF], float (*out)[MAXF], int n) {
    float rate = smoothing;
    if (rate <= 0.0f) rate = 1.0f;     /* 0 = follow the target immediately */
    if (rate > 1.0f) rate = 1.0f;
    /* Seed the read offset from the propagation delay (distance / speed of
       sound). An approaching source then consumes exactly that delay as it
       closes in, instead of exhausting a fixed amount of headroom. */
    if (!st->init) {
        st->p = clampf(seed, 1.0f, (float)(HIST - 2));
        st->init = 1;
    }
    for (int i = 0; i < n; i++) {
        st->ratio += (target - st->ratio) * rate;
        for (int ch = 0; ch < 2; ch++) {
            float pos = (float)st->w - st->p;
            int ip = (int)floorf(pos);
            float frac = pos - (float)ip;
            float s0 = st->buf[ch][imod(ip, HIST)];
            float s1 = st->buf[ch][imod(ip + 1, HIST)];
            out[ch][i] = s0 + (s1 - s0) * frac;
            st->buf[ch][st->w] = in[ch][i];
        }
        st->w = (st->w + 1) % HIST;
        st->p += 1.0f - st->ratio;
        st->p = clampf(st->p, 0.1f, (float)(HIST - 2));
    }
}

/* ---------------- distance ---------------- */

static float distance_gain(const DistanceCfg* cfg, const Listener* l, const Source* s) {
    if (cfg->rolloff <= 0.0f) return 1.0f;
    float ref = cfg->ref > 0.0f ? cfg->ref : 1.0f;
    float d = source_distance(s, l);
    float g = ref / (ref + (d - ref) * cfg->rolloff);
    if (g > cfg->max_gain) g = cfg->max_gain;
    return g;
}

/* ---------------- chain ---------------- */

/* last executed stage order, for the self-test */
static int chain_trace[ST_N];
static int chain_trace_len;

static void chain_reset(ChainState* st) {
    memset(st, 0, sizeof(*st));
    reverb_reset(&st->rev);
    st->dop.p = DOP_P0;
    st->dop.ratio = 1.0f;
    st->dop.init = 0;
}

/* Renders n frames of a mono source into interleaved stereo, applying each
   enabled stage in cfg->order with its gain. */
static void chain_apply(const ChainConfig* c, ChainState* st,
                        const Listener* l, const Source* s,
                        const float* mono, int n, float* out) {
    static float a[2][MAXF], b[2][MAXF];
    for (int i = 0; i < n; i++) {
        a[0][i] = mono[i];
        a[1][i] = mono[i];
    }
    float (*cur)[MAXF] = a;
    float (*nxt)[MAXF] = b;
    chain_trace_len = 0;

    for (int k = 0; k < ST_N; k++) {
        Stage stage = c->order[k];
        if (stage < 0 || stage >= ST_N) continue;
        if (!(c->stages & (1u << stage))) continue;
        chain_trace[chain_trace_len++] = (int)stage;
        switch (stage) {
        case ST_HRTF:
            hrtf_apply(&c->hrtf, l, s, st, cur, nxt, n);
            break;
        case ST_REVERB:
            reverb_apply(&c->reverb, &st->rev, cur, nxt, n);
            break;
        case ST_DOPPLER:
            dop_process(&st->dop, doppler_ratio(&c->doppler, l, s),
                        c->doppler.smoothing, dop_seed(&c->doppler, l, s), cur, nxt, n);
            break;
        case ST_DISTANCE: {
            float g = distance_gain(&c->distance, l, s);
            for (int ch = 0; ch < 2; ch++)
                for (int i = 0; i < n; i++) nxt[ch][i] = cur[ch][i] * g;
            break;
        }
        default: break;
        }
        float g = c->gain[stage];
        if (g != 1.0f)
            for (int ch = 0; ch < 2; ch++)
                for (int i = 0; i < n; i++) nxt[ch][i] *= g;
        float (*t)[MAXF] = cur; cur = nxt; nxt = t;
    }

    for (int i = 0; i < n; i++) {
        out[2 * i + 0] = cur[0][i];
        out[2 * i + 1] = cur[1][i];
    }
}

/* ---------------- reference: independent per-stage recomputation -------- */

/* This is the "do it by hand" path. It intentionally shares no stage code
   with the streaming chain: the azimuth/ITD/IID/pinna formula, the inverse
   distance law and the Doppler shift are each written out again from their
   definitions, the reverb is a direct triple loop over the delay lines, and
   only the canonical order is executed (the order test above covers
   permutations with the streaming engine). */

/* Recomputes the HRTF tap set from the azimuth definition, independently of
   hrtf_kernel(). Returns the filter length. */
static int ref_kernel(const HrtfCfg* cfg, const Listener* l, const Source* s,
                      float h[2][MAXTAPS]) {
    float az = atan2f((s->x - l->x) * cosf(l->yaw) - (s->z - l->z) * sinf(l->yaw),
                      -((s->x - l->x) * sinf(l->yaw) + (s->z - l->z) * cosf(l->yaw)));
    float max_delay = cfg->itd_ms * 0.001f * (float)SR;
    int d = (int)lroundf(sinf(az) * max_delay);
    int lim = MAXTAPS - (cfg->pinna_offset < 1 ? 1 : cfg->pinna_offset) - 2;
    if (d > lim) d = lim;
    if (d < -lim) d = -lim;
    int dl = d > 0 ? d : 0;
    int dr = d < 0 ? -d : 0;
    int off = cfg->pinna_offset < 1 ? 1 : cfg->pinna_offset;
    float t = (sinf(az) + 1.0f) * 0.5f;          /* 0 left .. 1 right */
    float gl = sqrtf(1.0f - t), gr = sqrtf(t);
    memset(h, 0, sizeof(float) * 2 * MAXTAPS);
    h[0][dl] = gl;
    h[1][dr] = gr;
    h[0][dl + off] = -cfg->pinna * gl;
    h[1][dr + off] = -cfg->pinna * gr;
    return (dl > dr ? dl : dr) + off + 1;
}

/* Inverse-square (actually 1/d with a 1 m reference) from first principles. */
static float ref_distance_gain(const DistanceCfg* cfg, const Listener* l, const Source* s) {
    if (cfg->rolloff <= 0.0f) return 1.0f;
    float dx = s->x - l->x, dy = s->y - l->y, dz = s->z - l->z;
    float d = sqrtf(dx*dx + dy*dy + dz*dz);
    float ref = cfg->ref > 0.0f ? cfg->ref : 1.0f;
    float g = ref / (d <= 0.0f ? ref : ref + (d - ref) * cfg->rolloff);
    return g > cfg->max_gain ? cfg->max_gain : g;
}

/* Propagation delay from distance and the speed of sound. */
static float ref_distance_samples(const DopplerCfg* cfg, const Listener* l, const Source* s) {
    float dx = s->x - l->x, dy = s->y - l->y, dz = s->z - l->z;
    float speed = cfg->speed > 1.0f ? cfg->speed : 343.0f;
    return sqrtf(dx*dx + dy*dy + dz*dz) / speed * (float)SR;
}

/* Fractional Doppler delay computed from the round trip distance change. */
static float ref_doppler_ratio(const DopplerCfg* cfg, const Listener* l, const Source* s) {
    float dx = s->x - l->x, dy = s->y - l->y, dz = s->z - l->z;
    float d = sqrtf(dx*dx + dy*dy + dz*dz);
    if (d < 1e-6f) return 1.0f;
    float vx = s->vx - l->vx, vy = s->vy - l->vy, vz = s->vz - l->vz;
    float radial = (vx*dx + vy*dy + vz*dz) / d;   /* + = receding */
    float r = cfg->speed / (cfg->speed + radial);
    return r < 0.25f ? 0.25f : (r > 4.0f ? 4.0f : r);
}

/* Reference renderer. Copies the chain's canonical stage order but every
   transfer is recomputed from the definitions above. */
static void ref_render(const ChainConfig* c, const Listener* l, const Source* s,
                       const float* mono, int n, float* out) {
    static float a[2][MAXF], b[2][MAXF];
    for (int i = 0; i < n; i++) {
        a[0][i] = mono[i];
        a[1][i] = mono[i];
    }
    float (*cur)[MAXF] = a;
    float (*nxt)[MAXF] = b;

    /* Reverb state built here, with the same deterministic lengths. */
    static float comb[2][NCOMB][COMBLEN];
    static float ap[2][NAP][APLEN];
    int cptr[2][NCOMB], aptr[2][NAP];
    memset(comb, 0, sizeof(comb));
    memset(ap, 0, sizeof(ap));
    memset(cptr, 0, sizeof(cptr));
    memset(aptr, 0, sizeof(aptr));
    int clen[2][NCOMB], alen[2][NAP];
    for (int e = 0; e < 2; e++)
        for (int k = 0; k < NCOMB; k++)
            clen[e][k] = COMB_BASE[k] + (e ? 23 : 0) > COMBLEN ? COMBLEN : COMB_BASE[k] + (e ? 23 : 0);
    for (int e = 0; e < 2; e++)
        for (int k = 0; k < NAP; k++)
            alen[e][k] = AP_BASE[k] + (e ? 23 : 0) > APLEN ? APLEN : AP_BASE[k] + (e ? 23 : 0);

    /* Doppler line. */
    static float dbuf[2][HIST];
    memset(dbuf, 0, sizeof(dbuf));
    int dw = 0;
    float dp = clampf(ref_distance_samples(&c->doppler, l, s), 1.0f, (float)(HIST - 2));
    float dratio = ref_doppler_ratio(&c->doppler, l, s);
    float drate = 1.0f;

    for (int k = 0; k < ST_N; k++) {
        Stage stage = c->order[k];
        if (stage < 0 || stage >= ST_N) continue;
        if (!(c->stages & (1u << stage))) continue;
        switch (stage) {
        case ST_HRTF: {
            float h[2][MAXTAPS];
            int len = ref_kernel(&c->hrtf, l, s, h);
            for (int ch = 0; ch < 2; ch++)
                for (int i = 0; i < n; i++) {
                    float acc = 0.0f;
                    for (int t = 0; t < len && t <= i; t++)
                        acc += h[ch][t] * cur[ch][i - t];
                    nxt[ch][i] = acc;
                }
            break;
        }
        case ST_REVERB:
            for (int i = 0; i < n; i++)
                for (int e = 0; e < 2; e++) {
                    float acc = 0.0f;
                    for (int k2 = 0; k2 < NCOMB; k2++) {
                        float y = comb[e][k2][cptr[e][k2]];
                        comb[e][k2][cptr[e][k2]] = cur[e][i] + y * c->reverb.decay;
                        if (++cptr[e][k2] >= clen[e][k2]) cptr[e][k2] = 0;
                        acc += y * 0.25f;
                    }
                    for (int k2 = 0; k2 < NAP; k2++) {
                        float d2 = ap[e][k2][aptr[e][k2]];
                        float y = d2 - acc;
                        ap[e][k2][aptr[e][k2]] = acc + d2 * AP_COEF;
                        if (++aptr[e][k2] >= alen[e][k2]) aptr[e][k2] = 0;
                        acc = y;
                    }
                    float wet = acc;
                    nxt[e][i] = cur[e][i] * (1.0f - c->reverb.wet) + wet * c->reverb.wet;
                }
            break;
        case ST_DOPPLER:
            for (int i = 0; i < n; i++) {
                for (int ch = 0; ch < 2; ch++) {
                    float pos = (float)dw - dp;
                    int ip = (int)floorf(pos);
                    float frac = pos - (float)ip;
                    int i0 = ((ip % HIST) + HIST) % HIST;
                    int i1 = (((ip + 1) % HIST) + HIST) % HIST;
                    nxt[ch][i] = dbuf[ch][i0] + (dbuf[ch][i1] - dbuf[ch][i0]) * frac;
                    dbuf[ch][dw] = cur[ch][i];
                }
                drate += (dratio - drate) * (c->doppler.smoothing <= 0.0f ? 1.0f : c->doppler.smoothing);
                dw = (dw + 1) % HIST;
                dp += 1.0f - drate;
                dp = clampf(dp, 0.1f, (float)(HIST - 2));
            }
            break;
        case ST_DISTANCE: {
            float g = ref_distance_gain(&c->distance, l, s);
            for (int ch = 0; ch < 2; ch++)
                for (int i = 0; i < n; i++) nxt[ch][i] = cur[ch][i] * g;
            break;
        }
        default: break;
        }
        float g = c->gain[stage];
        if (g != 1.0f)
            for (int ch = 0; ch < 2; ch++)
                for (int i = 0; i < n; i++) nxt[ch][i] *= g;
        float (*t)[MAXF] = cur; cur = nxt; nxt = t;
    }

    for (int i = 0; i < n; i++) {
        out[2 * i + 0] = cur[0][i];
        out[2 * i + 1] = cur[1][i];
    }
}

/* ---------------- test helpers ---------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static float peak_of(const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(b[i]) > m) m = fabsf(b[i]);
    return m;
}

static float max_abs_diff(const float* x, const float* y, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = fabsf(x[i] - y[i]);
        if (d > m) m = d;
    }
    return m;
}

/* Runs the streaming chain over a full buffer in fixed-size blocks. */
static void render_blocked(const ChainConfig* c, const Listener* l, const Source* s,
                           const float* mono, int n, int block, float* out) {
    ChainState st;
    chain_reset(&st);
    float tmp[2 * MAXF];
    int off = 0;
    while (off < n) {
        int m = n - off;
        if (m > block) m = block;
        chain_apply(c, &st, l, s, mono + off, m, tmp);
        memcpy(out + 2 * off, tmp, sizeof(float) * (size_t)m * 2);
        off += m;
    }
}

static ChainConfig config_canonical(void) {
    ChainConfig c;
    memset(&c, 0, sizeof(c));
    c.stages = MODE_ALL;
    c.order[0] = ST_HRTF;
    c.order[1] = ST_REVERB;
    c.order[2] = ST_DOPPLER;
    c.order[3] = ST_DISTANCE;
    for (int i = 0; i < ST_N; i++) c.gain[i] = 1.0f;
    c.hrtf.itd_ms = 0.6f;
    c.hrtf.pinna = 0.25f;
    c.hrtf.pinna_offset = 3;
    c.reverb.wet = 0.3f;
    c.reverb.decay = 0.7f;
    c.doppler.speed = 343.0f;
    c.doppler.smoothing = 0.0f;
    c.distance.ref = 1.0f;
    c.distance.rolloff = 1.0f;
    c.distance.max_gain = 4.0f;
    return c;
}

/* ---------------- cost probe ---------------- */

static volatile double sink;

/* Repeated render. After the first block the chain is steady state (reverb
   and Doppler state settled), so the per-block cost is what a running stream
   pays, not include the one-time warm-up. The chain state is reset per
   configuration so the comparison is fair. */
static double bench(const ChainConfig* c, const Listener* l, const Source* s,
                    const float* mono, int n, int iters, float* last_out) {
    static ChainState st;
    static float out[2 * MAXF];
    chain_reset(&st);
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (int it = 0; it < iters; it++) {
        chain_apply(c, &st, l, s, mono, n, out);
        sink += (double)out[0] + (double)sink * 0.0;
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    double ms = (t1.tv_sec - t0.tv_sec) * 1000.0
              + (t1.tv_nsec - t0.tv_nsec) / 1e6;
    if (last_out) memcpy(last_out, out, sizeof(float) * (size_t)n * 2);
    return ms / (double)iters;
}

/* ---------------- main ---------------- */

int main(void) {
    printf("render_chain: four-stage binaural chain at %d Hz\n", SR);

    static float mono[MAXF], ref[2 * MAXF], out[2 * MAXF], blocked[2 * MAXF];
    for (int i = 0; i < MAXF; i++) {
        float t = (float)i / (float)SR;
        mono[i] = 0.40f * sinf(2.0f * 3.14159265f * 220.0f * t)
                + 0.20f * sinf(2.0f * 3.14159265f * 1310.0f * t);
    }

    Listener l;
    memset(&l, 0, sizeof(l));
    l.x = 0.0f; l.y = 0.0f; l.z = 0.0f;
    Source s;
    memset(&s, 0, sizeof(s));
    s.x = 2.0f; s.z = 2.0f;
    s.vx = -20.0f;          /* moving toward the listener */
    s.gain = 1.0f;

    /* -- 1. stage semantics against closed forms ------------------------- */

    /* azimuth 0.5236 rad -> ITD of round(sin*28.8)=14 samples on the far ear */
    {
        Source a;
        memset(&a, 0, sizeof(a));
        float az = 0.5236f;
        a.x = 3.0f * sinf(az);
        a.z = -3.0f * cosf(az);
        float h[2][MAXTAPS];
        HrtfCfg hc = { 0.6f, 0.25f, 3 };
        int len = hrtf_kernel(&hc, &l, &a, h);
        int peakL = 0, peakR = 0;
        for (int t = 1; t < len; t++) {
            if (fabsf(h[0][t]) > fabsf(h[0][peakL])) peakL = t;
            if (fabsf(h[1][t]) > fabsf(h[1][peakR])) peakR = t;
        }
        check("hrtf_itd_left_tap", peakL == 14);
        check("hrtf_itd_right_tap", peakR == 0);
        check("hrtf_iid_balance", fabsf(h[0][14] - 0.5f) < 1e-4f
                               && fabsf(h[1][0] - 0.86603f) < 1e-3f);
        check("hrtf_pinna_tap", len >= 17 && fabsf(h[0][17] + 0.125f) < 1e-4f);
    }

    /* distance: 1/(1+(d-ref)) for ref=1, rolloff=1 */
    {
        DistanceCfg dc = { 1.0f, 1.0f, 4.0f };
        Source a;
        memset(&a, 0, sizeof(a));
        a.x = 3.0f;
        check("distance_law", fabsf(distance_gain(&dc, &l, &a) - 1.0f / 3.0f) < 1e-5f);
    }

    /* Menu-style: the classic law for source approaching at 20 m/s */
    {
        DopplerCfg dop = { 343.0f, 0.0f };
        Source a;
        memset(&a, 0, sizeof(a));
        a.x = 10.0f;
        a.vx = -20.0f;
        float r = doppler_ratio(&dop, &l, &a);
        check("doppler_ratio_approach", fabsf(r - 343.0f / 323.0f) < 1e-4f);
    }

    /* Doppler must actually transpose by the closed-form ratio */
    {
        ChainConfig c = config_canonical();
        c.stages = MODE_DOPPLER;
        enum { DN = 8 * MAXF, WIN = 8192 };
        static float in[DN], o2[2 * DN];
        for (int i = 0; i < DN; i++)
            in[i] = sinf(2.0f * 3.14159265f * 1000.0f * (float)i / (float)SR);
        ChainState st;
        chain_reset(&st);
        Source mv;
        memset(&mv, 0, sizeof(mv));
        mv.x = 10.0f;
        mv.vx = -20.0f;                /* approaching: pitch rises */
        int off = 0;
        while (off < DN) {
            int m = DN - off;
            if (m > MAXF) m = MAXF;
            chain_apply(&c, &st, &l, &mv, in + off, m, o2 + 2 * off);
            off += m;
        }
        int a = 2 * (DN - WIN), zc = 0;
        for (int i = a + 2; i < 2 * DN; i += 2)
            if (o2[i - 2] < 0.0f && o2[i] >= 0.0f) zc++;
        float measured = (float)zc / ((float)WIN / (float)SR);
        float expected = 1000.0f * 343.0f / 323.0f;
        printf("doppler_approach_measured=%.1f expected=%.1f\n", measured, expected);
        check("doppler_approach_pitch", fabsf(measured - expected) / expected < 0.01f);

        chain_reset(&st);
        mv.vx = 20.0f;                 /* receding: pitch falls */
        off = 0;
        while (off < DN) {
            int m = DN - off;
            if (m > MAXF) m = MAXF;
            chain_apply(&c, &st, &l, &mv, in + off, m, o2 + 2 * off);
            off += m;
        }
        zc = 0;
        for (int i = a + 2; i < 2 * DN; i += 2)
            if (o2[i - 2] < 0.0f && o2[i] >= 0.0f) zc++;
        measured = (float)zc / ((float)WIN / (float)SR);
        expected = 1000.0f * 343.0f / 363.0f;
        printf("doppler_recede_measured=%.1f expected=%.1f\n", measured, expected);
        check("doppler_recede_pitch", fabsf(measured - expected) / expected < 0.01f);
    }

    /* reverb wet=0 must be an exact passthrough */
    {
        ChainConfig c = config_canonical();
        c.stages = MODE_REVERB;
        c.reverb.wet = 0.0f;
        render_blocked(&c, &l, &s, mono, MAXF, 128, out);
        float expect[2 * MAXF];
        for (int i = 0; i < MAXF; i++) { expect[2*i] = mono[i]; expect[2*i+1] = mono[i]; }
        check("reverb_wet0_passthrough", max_abs_diff(out, expect, 2 * MAXF) < 1e-6f);
    }

    /* reverb wet=1 must leave a decaying tail after the input stops */
    {
        ChainConfig c = config_canonical();
        c.stages = MODE_REVERB;
        c.reverb.wet = 1.0f;
        float imp[MAXF];
        memset(imp, 0, sizeof(imp));
        imp[0] = 1.0f;
        render_blocked(&c, &l, &s, imp, MAXF, 128, out);
        float late = 0.0f;
        for (int i = 2 * (MAXF - 400); i < 2 * MAXF; i++) late += fabsf(out[i]);
        check("reverb_has_tail", late > 1e-4f);
        check("reverb_stable", peak_of(out, 2 * MAXF) < 4.0f);

        /* feedback only shows after one comb round-trip, so run the chain
           for several buffers of silence and integrate the tail energy */
        float zeros[MAXF];
        memset(zeros, 0, sizeof(zeros));
        ChainState st2;
        double tail_long = 0.0, tail_short = 0.0;
        for (int pass = 0; pass < 2; pass++) {
            c.reverb.decay = pass == 0 ? 0.7f : 0.2f;
            chain_reset(&st2);
            chain_apply(&c, &st2, &l, &s, imp, MAXF, ref);
            for (int b = 0; b < 4; b++) {
                chain_apply(&c, &st2, &l, &s, zeros, MAXF, ref);
                for (int i = 0; i < 2 * MAXF; i++)
                    if (pass == 0) tail_long += fabsf(ref[i]);
                    else           tail_short += fabsf(ref[i]);
            }
        }
        printf("tail_decay0.7=%.3f tail_decay0.2=%.3f\n", tail_long, tail_short);
        check("reverb_decay_controls_tail", tail_long > tail_short * 2.0);
    }

    /* literal arithmetic: HRTF main tap * distance, written out by hand for
       a single frame instead of looped */
    {
        ChainConfig c = config_canonical();
        c.stages = MODE_HRTF | MODE_DIST;
        c.hrtf.pinna = 0.0f;            /* isolate the main tap */
        Listener ll;
        memset(&ll, 0, sizeof(ll));
        Source ss;
        memset(&ss, 0, sizeof(ss));
        ss.x = 1.0f; ss.z = -1.0f;      /* az = atan2(1,1) = +pi/4 */
        float buf[64];
        memset(buf, 0, sizeof(buf));
        buf[0] = 1.0f;
        ChainState st;
        chain_reset(&st);
        chain_apply(&c, &st, &ll, &ss, buf, 32, out);

        float dist = sqrtf(2.0f);
        float g = 1.0f / dist;                       /* rolloff=1, ref=1 */
        float gl = sqrtf(0.5f * (1.0f - 0.70710678f));
        float gr = sqrtf(0.5f * (1.0f + 0.70710678f));
        int itd = (int)lroundf(0.70710678f * 0.0006f * SR);   /* 20 samples */
        check("hand_arith_itd20", itd == 20);
        check("hand_arith_right_frame0", fabsf(out[1] - gr * g) < 1e-5f);
        check("hand_arith_left_frame0", fabsf(out[0]) < 1e-6f);
        check("hand_arith_left_at_itd", fabsf(out[2 * itd] - gl * g) < 1e-5f);
        check("hand_arith_right_at_itd_silent", fabsf(out[2 * itd + 1]) < 1e-6f);
    }

    /* -- 2. streaming chain versus per-stage whole-buffer reference ------ */

    ChainConfig canon = config_canonical();
    ref_render(&canon, &l, &s, mono, MAXF, ref);
    render_blocked(&canon, &l, &s, mono, MAXF, 128, blocked);
    float seam = max_abs_diff(ref, blocked, 2 * MAXF);
    check("chain_matches_reference_all4", seam < 1e-4f);
    printf("reference_max_diff=%.3e\n", seam);

    /* whole-buffer chain vs blocked chain: block seams add nothing */
    float whole[2 * MAXF];
    render_blocked(&canon, &l, &s, mono, MAXF, MAXF, whole);
    float seam2 = max_abs_diff(whole, blocked, 2 * MAXF);
    printf("block_seam_max_diff=%.3e\n", seam2);
    check("block_seam_identical", seam2 < 1e-6f);

    {
        int n = 2 * MAXF;
        float boundary = 0.0f, interior = 0.0f;
        for (int i = 1; i < n; i++) {
            float d = fabsf(blocked[i] - blocked[i - 1]);
            if (i % 256 == 0) { if (d > boundary) boundary = d; }
            else              { if (d > interior) interior = d; }
        }
        printf("boundary_max_step=%.4f interior_max_step=%.4f\n", boundary, interior);
        check("no_seam_click", boundary <= interior * 1.5f + 1e-5f);
    }

    /* a moving source changes the per-sample HRTF/Doppler/ITD values; make
       sure that still renders identically in one block and in small blocks */
    {
        Source moving = s;
        moving.x = 6.0f; moving.z = -1.0f;
        moving.vx = -40.0f;
        ChainConfig c = config_canonical();
        static float one[2 * MAXF], many[2 * MAXF];
        render_blocked(&c, &l, &moving, mono, MAXF, MAXF, one);
        render_blocked(&c, &l, &moving, mono, MAXF, 61, many);
        check("moving_source_block_seam_identical",
              max_abs_diff(one, many, 2 * MAXF) < 1e-6f);
        ref_render(&c, &l, &moving, mono, MAXF, one);
        check("moving_source_matches_reference",
              max_abs_diff(one, many, 2 * MAXF) < 1e-4f);
    }

    /* velocity changes between blocks must not put a step in the signal:
       drive the chain block by block with the source slowing down, and look
       at the sample-to-sample deltas across block boundaries */
    {
        ChainConfig c = config_canonical();
        c.doppler.smoothing = 0.02f;
        ChainState st;
        chain_reset(&st);
        Source dyn = s;
        dyn.x = 8.0f; dyn.z = -1.0f; dyn.vx = -30.0f;
        float in[MAXF], tmp[2 * MAXF];
        for (int i = 0; i < MAXF; i++) in[i] = mono[i];
        float prevL = 0.0f, prevR = 0.0f;
        float max_delta = 0.0f;
        for (int b = 0; b < 8; b++) {
            dyn.vx *= -0.55f;           /* direction/speed flips per block */
            chain_apply(&c, &st, &l, &dyn, in, 128, tmp);
            for (int i = 0; i < 128; i++) {
                float dl = fabsf(tmp[2*i] - prevL);
                float dr = fabsf(tmp[2*i+1] - prevR);
                if (dl > max_delta) max_delta = dl;
                if (dr > max_delta) max_delta = dr;
                prevL = tmp[2*i]; prevR = tmp[2*i+1];
            }
        }
        printf("velocity_change_max_delta=%.5f\n", max_delta);
        check("velocity_change_no_click", max_delta < 0.10f);
    }

    /* equality for the deterministic three stages alone */
    {
        ChainConfig c = config_canonical();
        c.stages = MODE_HRTF | MODE_DOPPLER | MODE_DIST;
        ref_render(&c, &l, &s, mono, MAXF, ref);
        render_blocked(&c, &l, &s, mono, MAXF, 97, blocked);   /* odd block size */
        check("chain_matches_reference_no_reverb", max_abs_diff(ref, blocked, 2 * MAXF) < 1e-4f);
    }

    /* -- 3. configurable stage order ------------------------------------ */

    {
        /* All four stages are linear and time-invariant, so a permutation is
           mathematically commutative; what must hold is that the configured
           order is the order actually executed. */
        ChainConfig swapped = canon;
        swapped.order[0] = ST_REVERB;
        swapped.order[1] = ST_HRTF;
        swapped.order[2] = ST_DISTANCE;
        swapped.order[3] = ST_DOPPLER;
        render_blocked(&swapped, &l, &s, mono, MAXF, 128, out);
        int traced = chain_trace_len == 4
                  && chain_trace[0] == ST_REVERB
                  && chain_trace[1] == ST_HRTF
                  && chain_trace[2] == ST_DISTANCE
                  && chain_trace[3] == ST_DOPPLER;
        check("order_is_executed", traced);

        /* the hand reference follows the same permutation */
        ref_render(&swapped, &l, &s, mono, MAXF, ref);
        check("order_matches_reference", max_abs_diff(ref, out, 2 * MAXF) < 1e-4f);

        /* subset + permutation */
        ChainConfig sub = canon;
        sub.stages = MODE_REVERB | MODE_HRTF;
        sub.order[0] = ST_DOPPLER; sub.order[1] = ST_REVERB;
        sub.order[2] = ST_DISTANCE; sub.order[3] = ST_HRTF;
        render_blocked(&sub, &l, &s, mono, MAXF, 128, out);
        check("order_subset_skips_disabled", chain_trace_len == 2
              && chain_trace[0] == ST_REVERB && chain_trace[1] == ST_HRTF);

        ChainConfig a = canon, b = canon;
        a.stages = MODE_HRTF | MODE_DIST;
        b.stages = MODE_HRTF | MODE_DIST;
        a.order[0] = ST_HRTF; a.order[1] = ST_DISTANCE; a.order[2] = ST_REVERB; a.order[3] = ST_DOPPLER;
        b.order[0] = ST_DISTANCE; b.order[1] = ST_HRTF; b.order[2] = ST_REVERB; b.order[3] = ST_DOPPLER;
        render_blocked(&a, &l, &s, mono, MAXF, 128, out);
        render_blocked(&b, &l, &s, mono, MAXF, 128, blocked);
        check("order_commutes_for_scalar_stages", max_abs_diff(out, blocked, 2 * MAXF) < 1e-5f);
    }

    /* -- 4. configurable gain allocation -------------------------------- */

    {
        ChainConfig unity = canon;
        ChainConfig doubled = canon;
        for (int i = 0; i < ST_N; i++) doubled.gain[i] = 2.0f;
        render_blocked(&unity, &l, &s, mono, MAXF, 128, ref);
        render_blocked(&doubled, &l, &s, mono, MAXF, 128, out);
        int ok = 1;
        for (int i = 0; i < 2 * MAXF; i++)
            if (fabsf(out[i] - 16.0f * ref[i]) > 1e-4f * (1.0f + fabsf(ref[i]))) ok = 0;
        check("stage_gains_multiply", ok);

        ChainConfig half = canon;
        half.stages = MODE_DIST;
        half.gain[ST_DISTANCE] = 0.5f;
        render_blocked(&half, &l, &s, mono, MAXF, 128, out);
        render_blocked(&canon, &l, &s, mono, MAXF, 128, ref);
        /* canon has more stages; compare against distance-only at unity */
        ChainConfig base = canon;
        base.stages = MODE_DIST;
        render_blocked(&base, &l, &s, mono, MAXF, 128, ref);
        int ok2 = 1;
        for (int i = 0; i < 2 * MAXF; i++)
            if (fabsf(out[i] - 0.5f * ref[i]) > 1e-6f) ok2 = 0;
        check("single_stage_gain_scales", ok2);
    }

    /* -- 5. cost: each stage alone vs all four -------------------------- */

    {
        const int n = 256, iters = 1500;
        printf("cost: %d-frame blocks, %d iterations per configuration\n", n, iters);
        printf("%-14s %10s %10s\n", "config", "us/block", "realtime");
        const unsigned masks[ST_N] = { MODE_HRTF, MODE_REVERB, MODE_DOPPLER, MODE_DIST };
        const char* names[ST_N] = { "hrtf", "reverb", "doppler", "distance" };
        double sum = 0.0, audio_ms = (double)n * 1000.0 / (double)SR;
        float last[2 * MAXF];
        for (int st = 0; st < ST_N; st++) {
            ChainConfig c = canon;
            c.stages = masks[st];
            double ms = bench(&c, &l, &s, mono, n, iters, NULL);
            sum += ms;
            printf("%-14s %10.3f %9.1fx\n", names[st], ms * 1000.0, audio_ms / ms);
        }
        double all = bench(&canon, &l, &s, mono, n, iters, last);
        printf("%-14s %10.3f %9.1fx\n", "all four", all * 1000.0, audio_ms / all);
        printf("%-14s %10.3f %9.1fx\n", "sum of single", sum * 1000.0, audio_ms / sum);
        printf("all/sum_of_singles=%.2f\n", all / sum);
        printf("all_four_peak=%.4f\n", peak_of(last, 2 * n));
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
