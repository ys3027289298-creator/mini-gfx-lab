
/* fsm: condition-driven state machine on top of tween easing.
   States hold target parameter vectors; transitions carry a duration and
   ease. current is the logical state the machine sits in: while a blend is
   in flight it stays at the source, and only becomes the destination when
   the blend lands. Conditions keep being evaluated during a blend, so a
   different transition out of the source can interrupt it; the replacement
   blend starts from the live output and therefore never jumps. A transition
   already blending toward its target is skipped, so a held condition cannot
   restart the blend every tick. Every fsm_update is bounded to
   FSM_MAX_STEPS transition firings, so zero-duration transition cycles
   cannot hang. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define FSM_PARAM_DIM 4
#define FSM_MAX_STATES 16
#define FSM_MAX_TRANS 64
#define FSM_MAX_STEPS 32

typedef enum { EASE_LINEAR = 0, EASE_QUAD_IN, EASE_QUAD_OUT, EASE_CUBIC_IN_OUT, EASE_STEP } Ease;

typedef int (*FsmCond)(void* user);

typedef struct {
    float target[FSM_PARAM_DIM];
} FsmState;

typedef struct {
    int from, to;          /* state indices */
    float duration;        /* >= 0; 0 fires and settles immediately */
    Ease ease;
    FsmCond cond;
} FsmTrans;

typedef struct {
    FsmState states[FSM_MAX_STATES];
    FsmTrans trans[FSM_MAX_TRANS];
    int nstates;
    int ntrans;
    int current;           /* logical state: source while blending, -1 before fsm_start */
    int dest;              /* blend destination while blending, else -1 */
    int active;            /* index of the transition whose blend is in flight, else -1 */
    float elapsed;         /* time spent in the current blend */
    float blend_dur;
    Ease blend_ease;
    float from_val[FSM_PARAM_DIM];
    float out[FSM_PARAM_DIM];
    int last_steps;        /* transition firings consumed by last update */
} Fsm;

static float ease_apply(Ease e, float u) {
    switch (e) {
    case EASE_QUAD_IN: return u * u;
    case EASE_QUAD_OUT: return 1.0f - (1.0f - u) * (1.0f - u);
    case EASE_CUBIC_IN_OUT: return u < 0.5f ? 4.0f * u * u * u : 1.0f - powf(-2.0f * u + 2.0f, 3.0f) / 2.0f;
    case EASE_STEP: return u < 1.0f ? 0.0f : 1.0f;
    case EASE_LINEAR:
    default: return u;
    }
}

static void fsm_init(Fsm* fsm) {
    memset(fsm, 0, sizeof(*fsm));
    fsm->current = -1;
    fsm->dest = -1;
    fsm->active = -1;
}

static int fsm_add_state(Fsm* fsm, const float params[FSM_PARAM_DIM]) {
    if (fsm->nstates >= FSM_MAX_STATES) return -1;
    int id = fsm->nstates++;
    memcpy(fsm->states[id].target, params, sizeof(fsm->states[id].target));
    return id;
}

/* zero-duration self loops are rejected: they would fire forever without
   moving the machine and only burn the per-update step budget. */
static int fsm_add_trans(Fsm* fsm, int from, int to, float dur, Ease e, FsmCond cond) {
    if (fsm->ntrans >= FSM_MAX_TRANS) return -1;
    if (from < 0 || from >= fsm->nstates || to < 0 || to >= fsm->nstates) return -1;
    if (dur < 0.0f || cond == NULL) return -1;
    if (from == to && dur == 0.0f) return -1;
    FsmTrans* t = &fsm->trans[fsm->ntrans++];
    t->from = from; t->to = to; t->duration = dur; t->ease = e; t->cond = cond;
    return 0;
}

static int fsm_validate(const Fsm* fsm) {
    if (fsm->nstates <= 0) return 0;
    for (int i = 0; i < fsm->ntrans; i++) {
        const FsmTrans* t = &fsm->trans[i];
        if (t->from < 0 || t->from >= fsm->nstates) return 0;
        if (t->to < 0 || t->to >= fsm->nstates) return 0;
        if (t->duration < 0.0f || t->cond == NULL) return 0;
        if (t->from == t->to && t->duration == 0.0f) return 0;
    }
    return 1;
}

static void fsm_start(Fsm* fsm, int state) {
    fsm->current = state;
    fsm->dest = -1;
    fsm->active = -1;
    fsm->elapsed = 0.0f;
    memcpy(fsm->out, fsm->states[state].target, sizeof(fsm->out));
}

static const float* fsm_output(const Fsm* fsm) {
    return fsm->out;
}

static int fsm_current(const Fsm* fsm) { return fsm->current; }
static int fsm_blending(const Fsm* fsm) { return fsm->dest >= 0; }

/* begin a blend from whatever the machine outputs right now, which keeps
   the output continuous even when a transition is interrupted mid-flight.
   current is left untouched: it moves to the destination only when the
   blend reaches it, so source-state conditions stay live while blending.
   active records the edge whose blend is running so the scan can skip it. */
static void fsm_begin_blend(Fsm* fsm, int ti) {
    const FsmTrans* t = &fsm->trans[ti];
    memcpy(fsm->from_val, fsm->out, sizeof(fsm->from_val));
    fsm->elapsed = 0.0f;
    fsm->blend_dur = t->duration;
    fsm->blend_ease = t->ease;
    fsm->active = ti;
    if (t->duration == 0.0f) {
        /* settle at once and leave dest clear so zero-duration chains
           can keep firing within the same tick */
        memcpy(fsm->out, fsm->states[t->to].target, sizeof(fsm->out));
        fsm->dest = -1;
        fsm->active = -1;
    } else {
        fsm->dest = t->to;
    }
}

static void fsm_advance_blend(Fsm* fsm, float dt) {
    if (fsm->dest < 0 || fsm->blend_dur == 0.0f) return;
    fsm->elapsed += dt;
    float u = fsm->elapsed / fsm->blend_dur;
    if (u >= 1.0f) u = 1.0f;
    float w = ease_apply(fsm->blend_ease, u);
    const float* dst = fsm->states[fsm->dest].target;
    for (int k = 0; k < FSM_PARAM_DIM; k++)
        fsm->out[k] = fsm->from_val[k] + (dst[k] - fsm->from_val[k]) * w;
    if (u >= 1.0f) {
        fsm->current = fsm->dest;
        fsm->dest = -1;
        fsm->active = -1;
    }
}

/* one clock tick. The current blend advances first; on landing, current
   becomes the destination. Conditions are then evaluated once per tick, in
   insertion order, from the logical current state (which stays at the source
   while a blend is in flight): the first matching edge wins and that single
   edge is all the tick may fire. Taking a fresh condition snapshot each tick
   means two edges that are true together cannot ping-pong a blend within one
   tick, while a different edge becoming true on a later tick still interrupts
   the running blend and restarts it from the live output, so nothing jumps.
   The edge whose blend is running is skipped, so a held condition cannot
   restart it. Zero-duration transitions chain within the same tick, up to
   FSM_MAX_STEPS firings, so a transition cycle always terminates. */
static void fsm_update(Fsm* fsm, float dt, void* user) {
    fsm->last_steps = 0;
    fsm_advance_blend(fsm, dt);
    for (;;) {
        if (fsm->last_steps >= FSM_MAX_STEPS) break;       /* bound per tick */
        int fired = -1;
        for (int i = 0; i < fsm->ntrans; i++) {
            if (i == fsm->active) continue;                /* this blend is already running */
            const FsmTrans* t = &fsm->trans[i];
            if (t->from == fsm->current && t->cond(user)) { fired = i; break; }
        }
        if (fired < 0) break;
        const FsmTrans* t = &fsm->trans[fired];
        fsm->last_steps++;
        fsm_begin_blend(fsm, fired);
        if (t->duration == 0.0f) {
            fsm->current = t->to;                          /* settled, chain may continue */
        } else {
            break;                                         /* one positive-duration blend per tick */
        }
    }
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int near4(const float* a, const float* b, float eps) {
    for (int k = 0; k < FSM_PARAM_DIM; k++)
        if (fabsf(a[k] - b[k]) > eps) return 0;
    return 1;
}

/* ---- condition inputs -------------------------------------------------- */
/* Edge trigger: true only on the first update tick after the input changes
   to the wanted value. g_edge is armed by drive() and consumed at the end of
   the next fsm_update, so a command that stays held cannot retrigger an edge
   on later ticks and two competing edges cannot restart a blend across
   ticks. */
static int g_input;                /* current command value */
static int g_edge;                 /* an input change is waiting to be read */
static int cond_edge(void* user) {
    int want = *(int*)user;
    return g_edge && g_input == want;
}
static void drive(int value) {
    if (value != g_input) g_edge = 1;
    g_input = value;
}
/* one clock tick for the edge-driven test rigs: run the machine, then retire
   the pending edge so a held command is level, not a repeating trigger. */
static void tick(Fsm* fsm, float dt, void* user) {
    fsm_update(fsm, dt, user);
    g_edge = 0;
}
static int cond_always(void* user) { (void)user; return 1; }
static int cond_never(void* user) { (void)user; return 0; }

int main(void) {
    /* 1. basic transition: interpolation, easing and arrival */
    Fsm fsm;
    fsm_init(&fsm);
    float p_idle[FSM_PARAM_DIM] = {0, 0, 0, 0};
    float p_walk[FSM_PARAM_DIM] = {10, 20, 30, 40};
    int s_idle = fsm_add_state(&fsm, p_idle);
    int s_walk = fsm_add_state(&fsm, p_walk);
    int want_walk = 1;
    check("add_trans_ok", fsm_add_trans(&fsm, s_idle, s_walk, 1.0f, EASE_LINEAR, cond_edge) == 0);
    fsm_start(&fsm, s_idle);
    check("start_output", near4(fsm_output(&fsm), p_idle, 1e-6f));
    drive(want_walk);
    tick(&fsm, 0.0f, &want_walk);
    check("transition_fired", fsm_current(&fsm) == s_idle && fsm_blending(&fsm));
    check("blend_starts_continuous", near4(fsm_output(&fsm), p_idle, 1e-6f));
    check("steps_one", fsm.last_steps == 1);
    tick(&fsm, 0.5f, &want_walk);
    float expect_half[FSM_PARAM_DIM] = {5, 10, 15, 20};
    check("blend_midpoint", near4(fsm_output(&fsm), expect_half, 1e-4f));
    tick(&fsm, 0.5f, &want_walk);
    check("blend_arrives", fsm_current(&fsm) == s_walk && !fsm_blending(&fsm)
                           && near4(fsm_output(&fsm), p_walk, 1e-5f));
    /* held input: the edge is gone, so nothing refires */
    drive(want_walk);
    tick(&fsm, 0.1f, &want_walk);
    check("idle_no_steps", fsm.last_steps == 0 && !fsm_blending(&fsm));

    /* cubic ease stays inside [0,1] and hits its midpoint */
    Fsm ez;
    fsm_init(&ez);
    float z0[FSM_PARAM_DIM] = {0};
    float z1[FSM_PARAM_DIM] = {1, 1, 1, 1};
    int ez0 = fsm_add_state(&ez, z0);
    int ez1 = fsm_add_state(&ez, z1);
    fsm_add_trans(&ez, ez0, ez1, 1.0f, EASE_CUBIC_IN_OUT, cond_always);
    fsm_start(&ez, ez0);
    fsm_update(&ez, 0.0f, NULL);
    fsm_update(&ez, 0.5f, NULL);
    check("cubic_midpoint", fabsf(fsm_output(&ez)[0] - 0.5f) < 1e-4f);
    float prev = -1.0f;
    int mono = 1, bounded = 1;
    for (int i = 0; i <= 20; i++) {
        Fsm e2;
        fsm_init(&e2);
        int a = fsm_add_state(&e2, z0);
        int b = fsm_add_state(&e2, z1);
        fsm_add_trans(&e2, a, b, 1.0f, EASE_CUBIC_IN_OUT, cond_always);
        fsm_start(&e2, a);
        fsm_update(&e2, 0.0f, NULL);
        fsm_update(&e2, i / 20.0f, NULL);
        float v = fsm_output(&e2)[0];
        if (v < prev - 1e-6f) mono = 0;
        if (v < -1e-6f || v > 1.0f + 1e-6f) bounded = 0;
        prev = v;
    }
    check("cubic_monotonic", mono);
    check("cubic_bounded", bounded);

    /* 2. interrupting a blend mid-flight starts the new one from the
          current output: no parameter jump. Each command is a distinct
          edge out of the source state, so command 1 blends first and a
          later command 2 edge interrupts it. */
    Fsm mb;
    fsm_init(&mb);
    float m0[FSM_PARAM_DIM] = {0};
    float m1[FSM_PARAM_DIM] = {10};
    float m2[FSM_PARAM_DIM] = {50};
    int mb0 = fsm_add_state(&mb, m0);
    int mb1 = fsm_add_state(&mb, m1);
    int mb2 = fsm_add_state(&mb, m2);
    int in1 = 1, in2 = 2;
    fsm_add_trans(&mb, mb0, mb1, 1.0f, EASE_LINEAR, cond_edge);
    fsm_add_trans(&mb, mb0, mb2, 1.0f, EASE_LINEAR, cond_edge);   /* command 2 interrupts */
    fsm_start(&mb, mb0);
    g_input = 0;                                        /* fresh edge baseline */
    g_edge = 0;
    drive(in1);                                            /* edge: command 1 */
    tick(&mb, 0.0f, &in1);
    check("interrupt_first_fired", fsm_blending(&mb) && mb.last_steps == 1
                                   && near4(fsm_output(&mb), m0, 1e-6f));
    tick(&mb, 0.5f, &in1);
    float at_interrupt[FSM_PARAM_DIM];
    memcpy(at_interrupt, fsm_output(&mb), sizeof(at_interrupt));
    check("interrupt_at_half", fabsf(at_interrupt[0] - 5.0f) < 1e-4f);
    /* same level held: no new edge, the in-flight blend continues */
    drive(in1);
    tick(&mb, 0.0f, &in1);
    check("inflight_holds", mb.last_steps == 0 && fsm_blending(&mb)
                            && near4(fsm_output(&mb), at_interrupt, 1e-6f));
    drive(in2);                                            /* edge: command 2 */
    tick(&mb, 0.0f, &in2);
    check("interrupt_switches", fsm_blending(&mb) && mb.last_steps == 1);
    check("interrupt_continuous", near4(fsm_output(&mb), at_interrupt, 1e-6f));
    tick(&mb, 0.5f, &in2);
    check("interrupt_blend_half", fabsf(fsm_output(&mb)[0] - 27.5f) < 1e-3f);
    tick(&mb, 0.5f, &in2);
    check("interrupt_arrives", !fsm_blending(&mb) && fsm_current(&mb) == mb2
                               && fabsf(fsm_output(&mb)[0] - 50.0f) < 1e-4f);

    /* 3. fine-grained linear run: consecutive outputs never jump. */
    Fsm ct;
    fsm_init(&ct);
    float c0[FSM_PARAM_DIM] = {0};
    float c1[FSM_PARAM_DIM] = {10, 10, 10, 10};
    int ct0 = fsm_add_state(&ct, c0);
    int ct1 = fsm_add_state(&ct, c1);
    fsm_add_trans(&ct, ct0, ct1, 1.0f, EASE_LINEAR, cond_always);
    fsm_start(&ct, ct0);
    fsm_update(&ct, 0.0f, NULL);
    float prev_out[FSM_PARAM_DIM];
    memcpy(prev_out, fsm_output(&ct), sizeof(prev_out));
    int continuous = 1;
    float dt = 0.008f;
    for (int i = 0; i < 200; i++) {
        fsm_update(&ct, dt, NULL);
        const float* o = fsm_output(&ct);
        for (int k = 0; k < FSM_PARAM_DIM; k++) {
            float step = 10.0f * dt / 1.0f;
            if (fabsf(o[k] - prev_out[k]) > step + 1e-5f) continuous = 0;
        }
        memcpy(prev_out, o, sizeof(prev_out));
    }
    check("output_continuous", continuous);
    check("continuous_settles", !fsm_blending(&ct) && near4(fsm_output(&ct), c1, 1e-4f));

    /* 4. 1000-tick pseudo-random walk: no illegal state, finite in-range
          output, per-tick step budget respected. */
    Fsm r;
    fsm_init(&r);
    float s0[FSM_PARAM_DIM] = {0, 1, 2, 3};
    float s1[FSM_PARAM_DIM] = {4, 5, 6, 7};
    float s2[FSM_PARAM_DIM] = {8, 9, 10, 0};
    float s3[FSM_PARAM_DIM] = {3, 2, 1, 0};
    int r0 = fsm_add_state(&r, s0);
    int r1 = fsm_add_state(&r, s1);
    int r2 = fsm_add_state(&r, s2);
    int r3 = fsm_add_state(&r, s3);
    int v1 = 1, v2 = 2, v3 = 3, v0 = 0;
    fsm_add_trans(&r, r0, r1, 0.2f, EASE_LINEAR, cond_edge);
    fsm_add_trans(&r, r0, r2, 0.3f, EASE_QUAD_OUT, cond_edge);
    fsm_add_trans(&r, r1, r2, 0.1f, EASE_CUBIC_IN_OUT, cond_edge);
    fsm_add_trans(&r, r1, r0, 0.2f, EASE_QUAD_IN, cond_edge);
    fsm_add_trans(&r, r2, r3, 0.25f, EASE_LINEAR, cond_edge);
    fsm_add_trans(&r, r2, r0, 0.4f, EASE_STEP, cond_edge);
    fsm_add_trans(&r, r3, r0, 0.15f, EASE_LINEAR, cond_edge);
    fsm_add_trans(&r, r3, r1, 0.5f, EASE_QUAD_OUT, cond_edge);
    check("validate_ok", fsm_validate(&r));
    fsm_start(&r, r0);
    g_input = 0;
    g_edge = 0;
    int illegal = 0, nonfinite = 0, out_of_range = 0, cap_hit = 0;
    int visited[FSM_MAX_STATES] = {0};
    unsigned seed = 12345u;
    for (int i = 0; i < 1000; i++) {
        seed = seed * 1103515245u + 12345u;
        int next = (int)((seed >> 16) % 4);
        drive(next);
        void* arg = next == 0 ? &v0 : next == 1 ? &v1 : next == 2 ? &v2 : &v3;
        tick(&r, 0.05f, arg);
        int cur = fsm_current(&r);
        if (cur < 0 || cur >= r.nstates) illegal = 1;
        else visited[cur] = 1;
        if (r.dest < -1 || r.dest >= r.nstates) illegal = 1;
        if (r.active < -1 || r.active >= r.ntrans) illegal = 1;
        if (r.last_steps > FSM_MAX_STEPS) illegal = 1;
        if (r.last_steps == FSM_MAX_STEPS) cap_hit = 1;
        const float* o = fsm_output(&r);
        for (int k = 0; k < FSM_PARAM_DIM; k++) {
            if (!isfinite(o[k])) nonfinite = 1;
            if (o[k] < -1e-4f || o[k] > 10.0f + 1e-4f) out_of_range = 1;
        }
    }
    int all_visited = visited[r0] && visited[r1] && visited[r2] && visited[r3];
    check("thousand_no_illegal", !illegal);
    check("thousand_finite", !nonfinite);
    check("thousand_in_range", !out_of_range);
    check("thousand_all_visited", all_visited);
    check("thousand_cap_free", !cap_hit);

    /* 5. zero-duration transition cycle: bounded per tick, never hangs. */
    Fsm cyc;
    fsm_init(&cyc);
    float zc[FSM_PARAM_DIM] = {0};
    int c_a = fsm_add_state(&cyc, zc);
    int c_b = fsm_add_state(&cyc, zc);
    int c_c = fsm_add_state(&cyc, zc);
    fsm_add_trans(&cyc, c_a, c_b, 0.0f, EASE_LINEAR, cond_always);
    fsm_add_trans(&cyc, c_b, c_c, 0.0f, EASE_LINEAR, cond_always);
    fsm_add_trans(&cyc, c_c, c_a, 0.0f, EASE_LINEAR, cond_always);
    check("cycle_validates", fsm_validate(&cyc));
    fsm_start(&cyc, c_a);
    fsm_update(&cyc, 0.016f, NULL);
    check("cycle_step_bound", cyc.last_steps == FSM_MAX_STEPS);
    /* 32 steps around a 3-state cycle starting at A lands on C */
    check("cycle_settles_legal", cyc.current == c_c && !fsm_blending(&cyc));
    /* a tick is independently bounded: the machine still moves afterwards */
    int before = cyc.current;
    fsm_update(&cyc, 0.016f, NULL);
    check("cycle_resumes", cyc.last_steps == FSM_MAX_STEPS && cyc.current != before);

    /* zero-duration chain whose conditions eventually stop ends early */
    Fsm chain;
    fsm_init(&chain);
    int q0 = fsm_add_state(&chain, zc);
    int q1 = fsm_add_state(&chain, zc);
    int q2 = fsm_add_state(&chain, zc);
    fsm_add_trans(&chain, q0, q1, 0.0f, EASE_LINEAR, cond_always);
    fsm_add_trans(&chain, q1, q2, 0.0f, EASE_LINEAR, cond_always);
    fsm_add_trans(&chain, q2, q0, 0.0f, EASE_LINEAR, cond_never);
    fsm_start(&chain, q0);
    fsm_update(&chain, 0.016f, NULL);
    check("chain_stops", chain.last_steps == 2 && fsm_current(&chain) == q2);

    /* 6. construction-time rejection of illegal wiring */
    Fsm bad;
    fsm_init(&bad);
    int b0 = fsm_add_state(&bad, zc);
    (void)b0;
    check("reject_bad_from", fsm_add_trans(&bad, 9, 0, 1.0f, EASE_LINEAR, cond_always) == -1);
    check("reject_bad_to", fsm_add_trans(&bad, 0, 9, 1.0f, EASE_LINEAR, cond_always) == -1);
    check("reject_neg_dur", fsm_add_trans(&bad, 0, 0, -1.0f, EASE_LINEAR, cond_always) == -1);
    check("reject_null_cond", fsm_add_trans(&bad, 0, 0, 1.0f, EASE_LINEAR, NULL) == -1);
    check("reject_zero_self_loop", fsm_add_trans(&bad, 0, 0, 0.0f, EASE_LINEAR, cond_always) == -1);
    check("positive_self_loop_ok", fsm_add_trans(&bad, 0, 0, 1.0f, EASE_LINEAR, cond_always) == 0);
    check("bad_machine_validates", fsm_validate(&bad));
    Fsm empty;
    fsm_init(&empty);
    check("empty_machine_rejected", !fsm_validate(&empty));

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
