/* vocalchain: configurable vocal chain --- denoise (downward expander),
   auto gain, and limiter, in a user-selectable order.

   Design invariants the tests pin down:

   - Each stage is a standalone streaming processor with its own state. The
     chain just wires them; `manual_run` below calls the same stage functions
     in an explicit straight-line order, and the two outputs are identical
     sample for sample. There is no hidden glue in the chain path.

   - Detectors run at block rate (64 frames), which makes the state evolution
     deterministic for a fixed block partition. Gain is smoothed per stage, so
     the processing is continuous across block boundaries.

   - Bypass is a per-stage dry/wet crossfade, not a hard switch. The target is
     reached over a 4 ms ramp (step-limited per sample), and the stage keeps
     processing while bypassed, so re-enabling resumes from the state it would
     have had anyway: after the ramp completes the output is identical to a
     chain that was never bypassed.

   - The order is configurable at any block boundary. Because each stage sees
     the previous stage's output, the order changes the result; the self test
     measures that change in dBFS instead of pretending it is zero.

   All buffers are mono; the signal model is a voiced source with syllable
   modulation, short plosive accents, and a noise floor between phrases. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define SR 48000
#define BLOCK 64
#define MAX_STAGES 3
#define FULL_FRAMES SR                  /* self-test signal: 1 s */
#define BYPASS_RAMP_MS 10.0f            /* de-click crossfade time */
#define PI 3.14159265358979323846f

/* ---- small helpers ------------------------------------------------------ */

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float db_to_lin(float db) {
    return powf(10.0f, db / 20.0f);
}

static float lin_to_db(float lin) {
    return 20.0f * log10f(lin > 1e-9f ? lin : 1e-9f);
}

static float rms_of(const float* b, int n) {
    float s = 0.0f;
    for (int i = 0; i < n; i++) s += b[i] * b[i];
    return n > 0 ? sqrtf(s / (float)n) : 0.0f;
}

static float peak_of(const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(b[i]) > m) m = fabsf(b[i]);
    return m;
}

static double rms_db_of(const float* b, int n) {
    double s = 0.0;
    for (int i = 0; i < n; i++) s += (double)b[i] * (double)b[i];
    double r = sqrt(s / (double)n);
    return 20.0 * log10(r > 1e-12 ? r : 1e-12);
}

static double rms_db_range(const float* b, int n, float t0, float t1) {
    int i0 = (int)(t0 * (float)SR);
    int i1 = (int)(t1 * (float)SR);
    if (i0 < 0) i0 = 0;
    if (i1 > n) i1 = n;
    if (i1 <= i0) return -180.0;
    return rms_db_of(b + i0, i1 - i0);
}

/* one-pole coefficient for a time constant, evaluated once per block */
static float coef(float ms, int block) {
    if (ms <= 0.0f) return 1.0f;
    return 1.0f - expf(-(float)block / ((float)SR * (ms * 0.001f)));
}

/* dry/wet crossfade for one sample inside a block */
static float blended_gain(float wet, int i, float start, float step) {
    float mix = clampf(start + step * (float)i, 0.0f, 1.0f);
    return 1.0f + (wet - 1.0f) * mix;
}

/* ---- stage 1: denoise / downward expander ------------------------------ */

typedef struct {
    float threshold_db;   /* open above this block RMS */
    float ratio;          /* expansion ratio below threshold (1 = no-op) */
    float floor_db;       /* maximum attenuation */
    float attack_ms;      /* closing time constant */
    float release_ms;     /* opening time constant */
    float env_db;         /* smoothed block envelope */
    float gain_db;        /* current applied gain */
} GateStage;

static void gate_init(GateStage* g) {
    g->threshold_db = -38.0f;
    g->ratio = 4.0f;
    g->floor_db = -18.0f;
    g->attack_ms = 1.0f;
    g->release_ms = 40.0f;
    g->env_db = -90.0f;
    g->gain_db = 0.0f;    /* a cold chain starts open */
}

static void gate_process(GateStage* g, const float* in, float* out, int n,
                         float mix_start, float mix_step) {
    float env_db = lin_to_db(rms_of(in, n));
    g->env_db += (env_db - g->env_db) * coef(20.0f, n);

    float want_db = 0.0f;
    if (g->env_db < g->threshold_db) {
        want_db = -(g->threshold_db - g->env_db) * (g->ratio - 1.0f);
        if (want_db < g->floor_db) want_db = g->floor_db;
    }
    /* close fast, open slowly */
    float c = (want_db < g->gain_db) ? coef(g->attack_ms, n) : coef(g->release_ms, n);
    g->gain_db += (want_db - g->gain_db) * c;

    float wet = db_to_lin(g->gain_db);
    for (int i = 0; i < n; i++)
        out[i] = in[i] * blended_gain(wet, i, mix_start, mix_step);
}

/* ---- stage 2: auto gain ------------------------------------------------- */

typedef struct {
    float target_db;      /* desired output RMS */
    float range_db;       /* gain clamp, symmetric */
    float attack_ms;      /* gain reduction time constant */
    float release_ms;     /* gain restoration time constant */
    float gain_db;
} AgcStage;

static void agc_init(AgcStage* a) {
    a->target_db = -16.0f;
    a->range_db = 24.0f;
    a->attack_ms = 8.0f;
    a->release_ms = 200.0f;
    a->gain_db = 0.0f;
}

static void agc_process(AgcStage* a, const float* in, float* out, int n,
                        float mix_start, float mix_step) {
    float want_db = clampf(a->target_db - lin_to_db(rms_of(in, n)),
                           -a->range_db, a->range_db);
    float c = (want_db < a->gain_db) ? coef(a->attack_ms, n) : coef(a->release_ms, n);
    a->gain_db += (want_db - a->gain_db) * c;

    float wet = db_to_lin(a->gain_db);
    for (int i = 0; i < n; i++)
        out[i] = in[i] * blended_gain(wet, i, mix_start, mix_step);
}

/* ---- stage 3: limiter --------------------------------------------------- */

typedef struct {
    float ceiling;        /* linear, -1 dBFS by default */
    float release_ms;
    float env;            /* current gain, <= 1 */
} LimiterStage;

static void limiter_init(LimiterStage* l) {
    l->ceiling = db_to_lin(-1.0f);
    l->release_ms = 60.0f;
    l->env = 1.0f;
}

static void limiter_process(LimiterStage* l, const float* in, float* out, int n,
                            float mix_start, float mix_step) {
    float rel = 1.0f - expf(-1.0f / (l->release_ms * 0.001f * (float)SR));
    for (int i = 0; i < n; i++) {
        float target = 1.0f;
        float a = fabsf(in[i]);
        if (a > l->ceiling) target = l->ceiling / a;
        if (target < l->env) l->env = target;          /* instant attack */
        else l->env += (target - l->env) * rel;        /* smooth release */
        out[i] = in[i] * blended_gain(l->env, i, mix_start, mix_step);
    }
}

/* ---- the chain ---------------------------------------------------------- */

typedef enum { STAGE_GATE = 0, STAGE_AGC = 1, STAGE_LIMITER = 2 } StageId;

typedef struct {
    StageId order[MAX_STAGES];
    int order_len;
    int bypass[MAX_STAGES];       /* indexed by StageId */
    float mix[MAX_STAGES];        /* ramp position per slot */
    GateStage gate;
    AgcStage agc;
    LimiterStage limiter;
    float buf_a[BLOCK];
    float buf_b[BLOCK];
} Chain;

static void chain_init(Chain* c) {
    memset(c, 0, sizeof(*c));
    c->order[0] = STAGE_GATE;
    c->order[1] = STAGE_AGC;
    c->order[2] = STAGE_LIMITER;
    c->order_len = MAX_STAGES;
    memset(c->bypass, 0, sizeof(c->bypass));
    for (int i = 0; i < MAX_STAGES; i++) c->mix[i] = 1.0f;
    gate_init(&c->gate);
    agc_init(&c->agc);
    limiter_init(&c->limiter);
}

/* Accept any permutation of the stages. Mix positions follow their stage, so
   re-ordering mid-stream does not restart a bypass ramp that is in flight. */
static void chain_set_order(Chain* c, const StageId* order, int len) {
    if (len <= 0 || len > MAX_STAGES) return;
    unsigned seen = 0;
    for (int i = 0; i < len; i++) {
        int id = (int)order[i];
        if (id < 0 || id >= MAX_STAGES) return;
        if (seen & (1u << id)) return;
        seen |= 1u << id;
    }
    float carry[MAX_STAGES];
    for (int i = 0; i < MAX_STAGES; i++)
        carry[i] = c->bypass[i] ? 0.0f : 1.0f;
    for (int i = 0; i < c->order_len; i++)
        carry[c->order[i]] = c->mix[i];
    for (int i = 0; i < len; i++) {
        c->order[i] = order[i];
        c->mix[i] = carry[order[i]];
    }
    c->order_len = len;
}

static void chain_set_bypass(Chain* c, StageId id, int on) {
    if ((int)id < 0 || (int)id >= MAX_STAGES) return;
    c->bypass[id] = on ? 1 : 0;
}

/* start value and per-sample step for one stage slot in this block */
static void mix_plan(const Chain* c, int slot, int n, float* start, float* step) {
    float target = c->bypass[c->order[slot]] ? 0.0f : 1.0f;
    float cur = c->mix[slot];
    float s = (target - cur) / (float)n;                  /* finish in one block */
    float max_step = 1.0f / (BYPASS_RAMP_MS * 0.001f * (float)SR);
    if (s > max_step) s = max_step;
    if (s < -max_step) s = -max_step;
    *start = cur;
    *step = s;
}

/* commit exactly to the target once the ramp reaches it, so a re-enabled
   stage returns to bit-exact dry/wet=1 rather than 0.99999994 */
static void mix_commit(Chain* c, int slot, float start, float step, int n) {
    float target = c->bypass[c->order[slot]] ? 0.0f : 1.0f;
    float v = start + step * (float)n;
    if ((target - start) * (target - v) <= 0.0f) v = target;
    c->mix[slot] = clampf(v, 0.0f, 1.0f);
}

/* Process one block (n <= BLOCK) through the configured order. Every stage
   writes a scratch buffer, so no stage can alias its own input. */
static void chain_process(Chain* c, const float* in, float* out, int n) {
    if (n <= 0 || n > BLOCK) return;
    const float* cur = in;
    float* dst = c->buf_a;
    for (int slot = 0; slot < c->order_len; slot++) {
        float start, step;
        mix_plan(c, slot, n, &start, &step);
        switch (c->order[slot]) {
        case STAGE_GATE:    gate_process(&c->gate, cur, dst, n, start, step); break;
        case STAGE_AGC:     agc_process(&c->agc, cur, dst, n, start, step); break;
        case STAGE_LIMITER: limiter_process(&c->limiter, cur, dst, n, start, step); break;
        }
        mix_commit(c, slot, start, step, n);
        cur = dst;
        dst = (dst == c->buf_a) ? c->buf_b : c->buf_a;
    }
    if (cur != out) memcpy(out, cur, sizeof(float) * (size_t)n);
}

/* ---- manual reference: the three stages wired by hand ------------------- */

typedef struct {
    GateStage gate;
    AgcStage agc;
    LimiterStage limiter;
    float a[BLOCK], b[BLOCK];
} ManualChain;

static void manual_init(ManualChain* m) {
    memset(m, 0, sizeof(*m));
    gate_init(&m->gate);
    agc_init(&m->agc);
    limiter_init(&m->limiter);
}

static void manual_process(ManualChain* m, const float* in, float* out, int n) {
    gate_process(&m->gate, in, m->a, n, 1.0f, 0.0f);
    agc_process(&m->agc, m->a, m->b, n, 1.0f, 0.0f);
    limiter_process(&m->limiter, m->b, out, n, 1.0f, 0.0f);
}

static void manual_run(ManualChain* m, const StageId* order,
                       const float* in, float* out, int n) {
    float* dst = m->a;
    const float* cur = in;
    for (int i = 0; i < MAX_STAGES; i++) {
        switch (order[i]) {
        case STAGE_GATE:    gate_process(&m->gate, cur, dst, n, 1.0f, 0.0f); break;
        case STAGE_AGC:     agc_process(&m->agc, cur, dst, n, 1.0f, 0.0f); break;
        case STAGE_LIMITER: limiter_process(&m->limiter, cur, dst, n, 1.0f, 0.0f); break;
        }
        cur = dst;
        dst = (dst == m->a) ? m->b : m->a;
    }
    memcpy(out, cur, sizeof(float) * (size_t)n);
}

/* ---- test signal -------------------------------------------------------- */

static float window_env(float t, float on, float off) {
    const float fade = 0.015f;
    if (t <= on || t >= off) return 0.0f;
    float w = 1.0f;
    if (t < on + fade) w = (t - on) / fade;
    if (t > off - fade) w = (off - t) / fade;
    return w * w * (3.0f - 2.0f * w);
}

/* voiced source: syllables, two plosive accents, noise floor between phrases */
static void gen_voice(float* buf, int n) {
    unsigned st = 0x1234567u;
    for (int i = 0; i < n; i++) {
        float t = (float)i / (float)SR;
        float phrase = window_env(t, 0.02f, 0.38f) + window_env(t, 0.68f, 0.98f);
        if (phrase > 1.0f) phrase = 1.0f;
        float syll = 0.55f + 0.45f * sinf(2.0f * PI * 4.5f * t - 1.2f);
        float accent = ((t > 0.20f && t < 0.215f) || (t > 0.80f && t < 0.815f))
                       ? 3.5f : 1.0f;
        float body = sinf(2.0f * PI * 180.0f * t)
                   + 0.45f * sinf(2.0f * PI * 360.0f * t + 0.4f)
                   + 0.18f * sinf(2.0f * PI * 540.0f * t + 1.1f);
        st ^= st << 13; st ^= st >> 17; st ^= st << 5;
        float noise = (float)(st & 0xffffu) / 32768.0f - 1.0f;
        buf[i] = phrase * syll * accent * 0.45f * body + 0.006f * noise;
    }
}

/* steady voiced tone, used by the click tests */
static void gen_steady(float* buf, int n, float amp) {
    for (int i = 0; i < n; i++) {
        float t = (float)i / (float)SR;
        buf[i] = amp * (sinf(2.0f * PI * 180.0f * t)
                        + 0.40f * sinf(2.0f * PI * 360.0f * t + 0.4f)
                        + 0.15f * sinf(2.0f * PI * 540.0f * t + 1.1f));
    }
}

/* ---- self test ---------------------------------------------------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static const StageId PERMS[6][3] = {
    { STAGE_GATE,    STAGE_AGC,     STAGE_LIMITER },
    { STAGE_GATE,    STAGE_LIMITER, STAGE_AGC },
    { STAGE_AGC,     STAGE_GATE,    STAGE_LIMITER },
    { STAGE_AGC,     STAGE_LIMITER, STAGE_GATE },
    { STAGE_LIMITER, STAGE_GATE,    STAGE_AGC },
    { STAGE_LIMITER, STAGE_AGC,     STAGE_GATE },
};

static const char* stage_ch(StageId id) {
    return id == STAGE_GATE ? "g" : (id == STAGE_AGC ? "a" : "l");
}

static void order_name(const StageId* o, int n, char* out) {
    for (int i = 0; i < n; i++) {
        out[2 * i] = stage_ch(o[i])[0];
        out[2 * i + 1] = (i + 1 < n) ? '-' : '\0';
    }
    out[2 * n] = '\0';
}

static void run_order(const float* sig, int frames, const StageId* order,
                      float* out) {
    Chain c;
    chain_init(&c);
    chain_set_order(&c, order, MAX_STAGES);
    for (int f = 0; f + BLOCK <= frames; f += BLOCK)
        chain_process(&c, sig + f, out + f, BLOCK);
}

/* Bypass one stage mid-stream at a point where the input is at a local peak,
   so a hard switch has a full-amplitude step to show and the measurement does
   not accidentally land on a zero crossing.

   Click score = largest sample-to-sample jump within 1500 samples of the
   toggle, compared against the largest jump the same run produces in settled
   windows on either side (which includes the signal's own slope and the
   detectors' normal movement). A de-clicked toggle must not raise the jump
   above that; a hard switch does, by a wide margin. */
static int align_to_peak(const float* x, int frames, int f) {
    const int period = (int)((float)SR / 180.0f + 0.5f);
    float best = -1.0f;
    int best_i = f;
    for (int i = f; i < f + period && i < frames; i++) {
        float a = fabsf(x[i]);
        if (a > best) { best = a; best_i = i; }
    }
    return best_i;
}

static float max_jump(const float* y, int i0, int i1) {
    float m = 0.0f;
    for (int i = i0 + 1; i < i1; i++) {
        float d = fabsf(y[i] - y[i - 1]);
        if (d > m) m = d;
    }
    return m;
}

/* By default the AGC would pull a loud test signal well below the limiter
   ceiling, so the limiter would have nothing to do and a bypass toggle of an
   idle stage is silently click-free. The limiter click test therefore runs
   with the AGC already bypassed, so the limiter is genuinely gain-reducing. */
static void run_toggles(const float* sig, int frames, StageId id,
                        int hard, int pre_bypass_agc, float* y) {
    Chain c;
    chain_init(&c);
    int slot = 0;
    for (int s = 0; s < c.order_len; s++) if (c.order[s] == id) slot = s;
    if (pre_bypass_agc) {
        int agc_slot = -1;
        for (int s = 0; s < c.order_len; s++)
            if (c.order[s] == STAGE_AGC) agc_slot = s;
        if (agc_slot >= 0) {
            c.bypass[STAGE_AGC] = 1;
            c.mix[agc_slot] = 0.0f;
        }
    }

    int off = align_to_peak(sig, frames, 15000);
    int on  = align_to_peak(sig, frames, 30000);
    off = (off / BLOCK) * BLOCK;      /* toggles take effect on block edges */
    on  = (on / BLOCK) * BLOCK;

    for (int f = 0; f + BLOCK <= frames; f += BLOCK) {
        if (f == off) {
            if (hard) { c.bypass[id] = 1; c.mix[slot] = 0.0f; }
            else chain_set_bypass(&c, id, 1);
        }
        if (f == on) {
            if (hard) { c.bypass[id] = 0; c.mix[slot] = 1.0f; }
            else chain_set_bypass(&c, id, 0);
        }
        chain_process(&c, sig + f, y + f, BLOCK);
    }
}

/* returns the toggle jump and the settled slope in the same run */
static void click_scores(StageId id, float amp, int pre_bypass_agc,
                         float* ramp_jump, float* ramp_slope,
                         float* hard_jump) {
    const int frames = FULL_FRAMES;
    static float sig[FULL_FRAMES];
    static float y[FULL_FRAMES];
    gen_steady(sig, frames, amp);

    int off = align_to_peak(sig, frames, 15000);
    int on  = align_to_peak(sig, frames, 30000);
    off = (off / BLOCK) * BLOCK;
    on  = (on / BLOCK) * BLOCK;

    run_toggles(sig, frames, id, 0, pre_bypass_agc, y);
    float slope = max_jump(y, off - 9000, off - 1500);
    float d = max_jump(y, off + 1500, off + 9000);
    if (d > slope) slope = d;
    d = max_jump(y, on - 9000, on - 1500);
    if (d > slope) slope = d;
    d = max_jump(y, on + 1500, on + 9000);
    if (d > slope) slope = d;

    float jump = max_jump(y, off - 500, off + 1500);
    d = max_jump(y, on - 500, on + 1500);
    if (d > jump) jump = d;
    *ramp_jump = jump;
    *ramp_slope = slope;

    run_toggles(sig, frames, id, 1, pre_bypass_agc, y);
    float hj = max_jump(y, off - 500, off + 1500);
    d = max_jump(y, on - 500, on + 1500);
    if (d > hj) hj = d;
    *hard_jump = hj;
}

int main(void) {
    static float sig[FULL_FRAMES];
    static float out_chain[FULL_FRAMES];
    static float out_manual[FULL_FRAMES];
    static float out_perm[FULL_FRAMES];

    gen_voice(sig, FULL_FRAMES);

    /* 1. chain == same stages called by hand, canonical order */
    Chain ch;
    ManualChain man;
    chain_init(&ch);
    manual_init(&man);
    float maxdiff = 0.0f;
    for (int f = 0; f + BLOCK <= FULL_FRAMES; f += BLOCK) {
        chain_process(&ch, sig + f, out_chain + f, BLOCK);
        manual_process(&man, sig + f, out_manual + f, BLOCK);
        for (int i = 0; i < BLOCK; i++) {
            float d = fabsf(out_chain[f + i] - out_manual[f + i]);
            if (d > maxdiff) maxdiff = d;
        }
    }
    printf("identity_maxdiff=%.3g\n", (double)maxdiff);
    check("chain_matches_manual", maxdiff == 0.0f);

    /* 1b. the same identity holds for every order, and the order API rejects
       bad permutations without disturbing the chain */
    static float out_hand[FULL_FRAMES];
    float perm_diff = 0.0f;
    for (int k = 0; k < 6; k++) {
        Chain pc;
        ManualChain pm;
        chain_init(&pc);
        chain_set_order(&pc, PERMS[k], MAX_STAGES);
        manual_init(&pm);
        for (int f = 0; f + BLOCK <= FULL_FRAMES; f += BLOCK) {
            chain_process(&pc, sig + f, out_perm + f, BLOCK);
            manual_run(&pm, PERMS[k], sig + f, out_hand + f, BLOCK);
            for (int i = 0; i < BLOCK; i++) {
                float d = fabsf(out_perm[f + i] - out_hand[f + i]);
                if (d > perm_diff) perm_diff = d;
            }
        }
    }
    printf("all_orders_manual_maxdiff=%.3g\n", (double)perm_diff);
    check("all_orders_match_manual", perm_diff == 0.0f);

    Chain ord;
    chain_init(&ord);
    const StageId dup[3] = { STAGE_GATE, STAGE_GATE, STAGE_LIMITER };
    const StageId bad[3] = { STAGE_AGC, (StageId)7, STAGE_LIMITER };
    float mix_before = ord.mix[0];
    chain_set_order(&ord, dup, 3);
    chain_set_order(&ord, bad, 3);
    check("reject_duplicate_order", ord.order[1] == STAGE_AGC);
    check("reject_invalid_order", ord.order_len == MAX_STAGES &&
                                  ord.mix[0] == mix_before);

    /* 2. every ordering, with the resulting level quantified in dBFS */
    double rms[6], peak[6], gap[6];
    printf("--- order sweep (delta vs g-a-l) ---\n");
    for (int k = 0; k < 6; k++) {
        char name[16];
        order_name(PERMS[k], MAX_STAGES, name);
        run_order(sig, FULL_FRAMES, PERMS[k], out_perm);
        rms[k] = rms_db_of(out_perm, FULL_FRAMES);
        peak[k] = 20.0 * log10((double)peak_of(out_perm, FULL_FRAMES));
        gap[k] = rms_db_range(out_perm, FULL_FRAMES, 0.55f, 0.66f);
        printf("order=%-6s rms=%+7.2fdB peak=%+7.2fdBFS gap=%+7.2fdB delta=%+6.2fdB\n",
               name, rms[k], peak[k], gap[k], rms[k] - rms[0]);
    }
    double lo = rms[0], hi = rms[0];
    for (int k = 1; k < 6; k++) {
        if (rms[k] < lo) lo = rms[k];
        if (rms[k] > hi) hi = rms[k];
    }
    printf("order_rms_spread=%.2fdB\n", hi - lo);
    check("order_changes_level", hi - lo > 0.5);

    /* AGC ahead of the gate lifts the noise floor above the gate threshold,
       so the 300 ms pause is no longer cleaned up. */
    printf("gap_canonical=%.2fdB gap_agc_first=%.2fdB\n", gap[0], gap[2]);
    check("order_changes_gate", gap[2] - gap[0] > 8.0);

    /* the limiter only guarantees the ceiling when it is the last stage */
    check("limiter_last_caps_peak", peak[0] <= -1.0 + 1e-3);
    check("limiter_last_caps_peak_2", peak[2] <= -1.0 + 1e-3);

    /* 3. bypass must not click, and a hard switch must show up as a click */
    const float amps[3] = { 0.012f, 0.02f, 1.2f };   /* gate, agc, limiter */
    const int pre_agc[3] = { 0, 0, 1 };             /* limiter test needs GR */
    const char* names[3] = { "bypass_click_gate", "bypass_click_agc",
                             "bypass_click_limiter" };
    for (int s = 0; s < MAX_STAGES; s++) {
        StageId id = (StageId)s;
        float jump = 0.0f, slope = 0.0f, hard = 0.0f;
        click_scores(id, amps[s], pre_agc[s], &jump, &slope, &hard);
        printf("%s: ramp_jump=%.5f settled_slope=%.5f hard_jump=%.5f\n",
               names[s], (double)jump, (double)slope, (double)hard);
        check(names[s], jump <= slope * 1.25f + 0.002f);
        check(s == 0 ? "hard_switch_click_gate" :
              s == 1 ? "hard_switch_click_agc" :
                       "hard_switch_click_limiter",
              hard > jump * 4.0f);
    }

    /* 4. bypass is recoverable: after the ramp the chain is sample-identical
       to one that was never bypassed, because the stage kept its state */
    static float ref[FULL_FRAMES];
    static float toggled[FULL_FRAMES];
    Chain a, b;
    chain_init(&a);
    chain_init(&b);
    int nb = FULL_FRAMES / BLOCK;
    for (int k = 0; k < nb; k++) {
        if (k == 300) chain_set_bypass(&b, STAGE_AGC, 1);
        if (k == 500) chain_set_bypass(&b, STAGE_AGC, 0);
        chain_process(&a, sig + k * BLOCK, ref + k * BLOCK, BLOCK);
        chain_process(&b, sig + k * BLOCK, toggled + k * BLOCK, BLOCK);
    }
    float restore_diff = 0.0f;
    for (int i = 510 * BLOCK; i < FULL_FRAMES; i++)
        if (fabsf(ref[i] - toggled[i]) > restore_diff)
            restore_diff = fabsf(ref[i] - toggled[i]);
    float during_diff = 0.0f;
    for (int i = 320 * BLOCK; i < 480 * BLOCK; i++)
        if (fabsf(ref[i] - toggled[i]) > during_diff)
            during_diff = fabsf(ref[i] - toggled[i]);
    printf("bypass_restore_diff=%.3g (during_bypass_diff=%.3g)\n",
           (double)restore_diff, (double)during_diff);
    check("bypass_restore_exact", restore_diff == 0.0f);
    check("bypass_actually_bypassed", during_diff > 0.002f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
