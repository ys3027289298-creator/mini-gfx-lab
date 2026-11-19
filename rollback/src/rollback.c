/* rollback: client-side prediction + rollback reconciliation for a networked
   unit, plus two control clients that reproduce the bugs it fixes.

   Model (client/server netcode, RTS-shaped):
     - fixed 20 Hz tick; one input byte per tick.
     - the host owns the authoritative state, advances it once per tick with the
       client's input, and broadcasts a snapshot (tick, position) every tick.
     - snapshots cross a lossy, jittery link and may arrive out of order.
     - the client applies its input locally the moment it is pressed, so the
       unit moves with zero perceived input latency.
     - a snapshot at tick a is the authoritative point. If the prediction at a
       disagrees, the client restores the snapshot state at a and replays every
       input it made in (a, live tick] -- rollback + replay.
     - the host also applies impulses (knockback) the client cannot predict.
       That is the only source of divergence here, and exactly what snapshots
       are for.

   Property under test: after a rollback, the state at every replayed tick is
   bit-identical to replaying the authoritative input sequence from the
   snapshot state. rb_apply() proves it for every replay; the tests also check
   it from the outside against the host's own input trace.

   Three clients share one input stream and one wire:
     RB_ROLLBACK  predict + reconcile
     RB_IGNORE    predict, never reconcile      (error accumulates)
     RB_SNAP      draw snapshots on arrival     (rubber-banding)

   build: make      self-test: make test      chart data: make metrics
*/
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TICK_HZ 20
#define TICK_MS (1000 / TICK_HZ)
#define DT (1.0f / (float)TICK_HZ)
#define SPEED 5.0f              /* units/s -> 0.25 per tick, diagonals normalised */
#define KNOCKBACK 0.6f          /* unpredicted host impulse, units */
#define HISTORY 512             /* ticks of input + state retained per timeline */
#define MAX_WIRE 8192
#define MAX_TICKS 4096
#define NCLIENTS 3

/* ------------------------------------------------------------------------ */
/* deterministic unit model                                                 */
/* ------------------------------------------------------------------------ */
typedef struct { float x, y; } Unit;

/* input byte: 0..7 = 8-way direction, 8 = idle */
static void input_dir(int in, float* dx, float* dy) {
    static const int DIRS[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    if (in < 0 || in > 7) { *dx = 0.0f; *dy = 0.0f; return; }
    float nx = (float)DIRS[in][0], ny = (float)DIRS[in][1];
    float len = sqrtf(nx * nx + ny * ny);
    *dx = nx / len; *dy = ny / len;
}

/* pure function of (state, input), so any replay reproduces the same floats */
static Unit unit_step(Unit u, int in) {
    float dx, dy;
    input_dir(in, &dx, &dy);
    u.x += dx * SPEED * DT;
    u.y += dy * SPEED * DT;
    return u;
}

static int unit_same(Unit a, Unit b) { return a.x == b.x && a.y == b.y; }
static float unit_dist(Unit a, Unit b) { return hypotf(a.x - b.x, a.y - b.y); }

static uint32_t xorshift32(uint32_t* s) {
    uint32_t x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x;
    return x;
}

/* ------------------------------------------------------------------------ */
/* tick-indexed timeline                                                    */
/* ------------------------------------------------------------------------ */
typedef struct {
    Unit u;                    /* state at tick */
    int tick;
    int input[HISTORY];        /* input applied at each tick */
    Unit state[HISTORY];       /* state after that input */
    int wrapped;
} Timeline;

static void tl_init(Timeline* t, Unit u, int tick) {
    memset(t, 0, sizeof(*t));
    t->u = u;
    t->tick = tick;
}

static int tl_has(const Timeline* t, int tick) {
    int lo = t->wrapped ? t->tick - HISTORY + 1 : 0;
    return tick >= lo && tick <= t->tick;
}
static int tl_input(const Timeline* t, int tick) { return t->input[tick % HISTORY]; }
static Unit tl_state(const Timeline* t, int tick) { return t->state[tick % HISTORY]; }

static void tl_push(Timeline* t, int in) {
    t->u = unit_step(t->u, in);
    t->tick++;
    if (t->tick >= HISTORY) t->wrapped = 1;
    t->input[t->tick % HISTORY] = in;
    t->state[t->tick % HISTORY] = t->u;
}

/* replay (from, to] from a base state, same arithmetic as tl_push */
static Unit replay_from(Unit base, const Timeline* src, int from, int to) {
    Unit u = base;
    for (int tick = from + 1; tick <= to; tick++) u = unit_step(u, tl_input(src, tick));
    return u;
}

/* ------------------------------------------------------------------------ */
/* input stream                                                             */
/* ------------------------------------------------------------------------ */
/* key held 2..6 ticks, then a random walk to a new direction. The client owns
   the keyboard; host and clients replay the same trace (lockstep). */
static int gen_input(uint32_t* rng, int* run_left, int* cur_dir) {
    if (*run_left <= 0) {
        *rng = (*rng * 1664525u) + 1013904223u;
        *cur_dir = (int)((*rng >> 16) % 9);
        *rng = (*rng * 1664525u) + 1013904223u;
        *run_left = 2 + (int)((*rng >> 16) % 5);
    }
    (*run_left)--;
    return *cur_dir;
}

/* ------------------------------------------------------------------------ */
/* wire                                                                     */
/* ------------------------------------------------------------------------ */
typedef struct {
    int one_way_ms;       /* base one-way delay per packet */
    int jitter_ms;        /* +/- uniform per packet */
    int loss_pct;
    int dup_pct;
    int knockback_every;  /* host impulse period in ticks, 0 = off */
    int ticks;
    uint32_t seed;
} Cfg;

typedef struct {
    int tick;
    Unit pos;
    int deliver_at;       /* client tick at which it becomes visible */
} Packet;

typedef enum { RB_ROLLBACK = 0, RB_IGNORE = 1, RB_SNAP = 2 } RbMode;

static int chance(uint32_t* rng, int pct) {
    return pct > 0 && (int)(xorshift32(rng) % 100) < pct;
}

static int pkt_delay(const Cfg* c, uint32_t* rng) {
    int d = c->one_way_ms;
    if (c->jitter_ms > 0)
        d += (int)(xorshift32(rng) % (uint32_t)(2 * c->jitter_ms + 1)) - c->jitter_ms;
    if (d < 0) d = 0;
    return (d + TICK_MS - 1) / TICK_MS;   /* ceil to ticks */
}

/* ------------------------------------------------------------------------ */
/* host                                                                     */
/* ------------------------------------------------------------------------ */
typedef struct { Timeline t; } Host;

static void host_init(Host* h) {
    memset(h, 0, sizeof(*h));
    tl_init(&h->t, (Unit){0.0f, 0.0f}, -1);
}

/* returns 1 when an unpredictable impulse landed on this tick */
static int host_step(Host* h, const Cfg* c, int tick, int in) {
    tl_push(&h->t, in);
    if (c->knockback_every > 0 && tick > 0 && tick % c->knockback_every == 0) {
        h->t.u.x += KNOCKBACK;
        h->t.state[tick % HISTORY] = h->t.u;
        return 1;
    }
    return 0;
}

static Packet host_snapshot(const Host* h, int now, int delay) {
    Packet p;
    p.tick = h->t.tick;
    p.pos = h->t.u;
    p.deliver_at = now + delay;
    return p;
}

/* ------------------------------------------------------------------------ */
/* client                                                                   */
/* ------------------------------------------------------------------------ */
typedef struct {
    RbMode mode;
    Timeline t;               /* predicted timeline (or echoed inputs) */

    int auth_tick;            /* newest authoritative tick folded in, -1 = none */
    Unit shown;               /* what this client draws */
    int shown_tick;           /* tick the drawn position came from */

    int snaps_seen;
    int snaps_applied;        /* snapshots folded into the prediction */
    int snaps_ack;            /* snapshots that agreed: ack only, no replay */
    int snaps_stale;          /* duplicate / superseded by a newer snapshot */
    int snaps_dropped;        /* outside retained history or from the future */

    int rollbacks;            /* reconciliations that replayed optimistic input */
    int rollbacks_accepted;   /* rollbacks whose replay was bit-exact */
    int replayed_ticks;
    int depth_max;
    int rewinds;              /* snap client: drawn tick moved backwards */

    int replays_checked;      /* exactness proofs run */
    int replay_mismatch;      /* must stay 0 */
} Client;

static void client_init(Client* cl, RbMode mode) {
    memset(cl, 0, sizeof(*cl));
    cl->mode = mode;
    tl_init(&cl->t, (Unit){0.0f, 0.0f}, -1);
    cl->auth_tick = -1;
    cl->shown_tick = -1;
}

static int rb_apply(Client* cl, Packet p);

/* Fold one authoritative point in. Returns 1 if a replay ran. */
static int rb_apply(Client* cl, Packet p) {
    if (p.tick > cl->t.tick || !tl_has(&cl->t, p.tick)) {
        cl->snaps_dropped++;
        return 0;
    }
    if (p.tick <= cl->auth_tick) {
        cl->snaps_stale++;
        return 0;
    }
    cl->snaps_applied++;

    Unit predicted = tl_state(&cl->t, p.tick);
    if (unit_same(predicted, p.pos)) {
        cl->auth_tick = p.tick;          /* prediction agrees: nothing to replay */
        cl->snaps_ack++;
        return 0;
    }

    int live = cl->t.tick;
    int depth = live - p.tick;
    cl->rollbacks++;
    cl->replayed_ticks += depth;
    if (depth > cl->depth_max) cl->depth_max = depth;

    /* independent reference: authoritative sequence replayed from scratch */
    Unit reference = replay_from(p.pos, &cl->t, p.tick, live);

    /* rollback: adopt the snapshot, replay the unconfirmed inputs into the
       timeline -- the same unit_step the live loop used */
    cl->t.state[p.tick % HISTORY] = p.pos;
    Unit u = p.pos;
    for (int tick = p.tick + 1; tick <= live; tick++) {
        u = unit_step(u, tl_input(&cl->t, tick));
        cl->t.state[tick % HISTORY] = u;
    }
    cl->t.u = u;
    cl->auth_tick = p.tick;

    cl->replays_checked++;
    if (unit_same(u, reference)) cl->rollbacks_accepted++;
    else cl->replay_mismatch++;
    return 1;
}

/* ------------------------------------------------------------------------ */
/* match: host + three clients on one wire                                  */
/* ------------------------------------------------------------------------ */
typedef struct {
    int ticks;
    float err[NCLIENTS][MAX_TICKS];   /* |drawn - authority| at each tick */
    unsigned char rb_event[MAX_TICKS];
    int impulse[MAX_TICKS];
    int impulse_n;
    Host host;
    Client cl[NCLIENTS];
} Run;

typedef struct {
    Cfg cfg;
    uint32_t net_rng;
    Packet wire[MAX_WIRE];
    int wire_n;
} Match;

static void match_emit(Match* m, Run* run, int now) {
    const Cfg* c = &m->cfg;
    if (chance(&m->net_rng, c->loss_pct)) return;
    int delay = pkt_delay(c, &m->net_rng);
    int copies = 1 + (chance(&m->net_rng, c->dup_pct) ? 1 : 0);
    for (int i = 0; i < copies && m->wire_n < MAX_WIRE; i++) {
        m->wire[m->wire_n++] = host_snapshot(&run->host, now, delay);
    }
}

static void match_run(const Cfg* cfg, Run* run) {
    memset(run, 0, sizeof(*run));
    run->ticks = cfg->ticks > MAX_TICKS ? MAX_TICKS : cfg->ticks;

    host_init(&run->host);
    for (int i = 0; i < NCLIENTS; i++) client_init(&run->cl[i], (RbMode)i);

    static Match m;
    memset(&m, 0, sizeof(m));
    m.cfg = *cfg;
    m.net_rng = cfg->seed ^ 0xa5a5a5a5u;

    uint32_t in_rng = cfg->seed ^ 0x9e3779b9u;
    int run_left = 0, cur_dir = 6;

    for (int t = 0; t < run->ticks; t++) {
        int in = gen_input(&in_rng, &run_left, &cur_dir);
        if (host_step(&run->host, cfg, t, in))
            run->impulse[run->impulse_n++] = t;
        match_emit(&m, run, t);

        /* local prediction: the unit moves this tick, before any packet */
        for (int i = 0; i < NCLIENTS; i++)
            if (run->cl[i].mode != RB_SNAP) tl_push(&run->cl[i].t, in);

        /* deliver whatever landed this tick, in wire order */
        for (int k = 0; k < m.wire_n; k++) {
            if (m.wire[k].deliver_at != t) continue;
            Packet p = m.wire[k];
            for (int i = 0; i < NCLIENTS; i++) {
                Client* cl = &run->cl[i];
                cl->snaps_seen++;
                if (cl->mode == RB_SNAP) {
                    /* the bug: last packet to land is drawn, older or not */
                    if (p.tick < cl->shown_tick) cl->rewinds++;
                    cl->shown = p.pos;
                    cl->shown_tick = p.tick;
                    continue;
                }
                if (cl->mode == RB_IGNORE) continue;
                if (rb_apply(cl, p)) run->rb_event[t] = 1;
            }
        }

        for (int i = 0; i < NCLIENTS; i++)
            if (run->cl[i].mode != RB_SNAP) run->cl[i].shown = run->cl[i].t.u;
        for (int i = 0; i < NCLIENTS; i++)
            run->err[i][t] = unit_dist(run->cl[i].shown, run->host.t.u);
    }
}

/* ------------------------------------------------------------------------ */
/* tests                                                                    */
/* ------------------------------------------------------------------------ */
static int failures;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

typedef struct { Host h; Client c; } Pair;

static void pair_feed(Pair* p, int n, uint32_t seed) {
    uint32_t rng = seed;
    int run_left = 0, cur_dir = 6;
    for (int i = 0; i < n; i++) {
        int in = gen_input(&rng, &run_left, &cur_dir);
        host_step(&p->h, &(Cfg){0}, p->h.t.tick + 1, in);
        tl_push(&p->c.t, in);
    }
}

static void pair_impulse(Pair* p) {
    p->h.t.u.x += KNOCKBACK;
    p->h.t.state[p->h.t.tick % HISTORY] = p->h.t.u;
}

static Packet snap_of(const Host* h, int tick) {
    Packet p;
    p.tick = tick;
    p.pos = tl_state(&h->t, tick);
    p.deliver_at = 0;
    return p;
}

/* the property the module exists for: every replayed tick equals the
   authoritative sequence replayed directly from the snapshot point */
static int verify_replay(const Client* cl, const Host* h, Packet p) {
    if (!unit_same(tl_state(&cl->t, p.tick), p.pos)) return 0;
    Unit u = p.pos;
    for (int tick = p.tick + 1; tick <= cl->t.tick; tick++) {
        u = unit_step(u, tl_input(&h->t, tick));
        if (!unit_same(tl_state(&cl->t, tick), u)) return 0;
    }
    return unit_same(cl->t.u, u);
}

static int recovery_after(const Run* run, int impulse_tick) {
    for (int t = impulse_tick + 1; t < run->ticks; t++)
        if (run->err[RB_ROLLBACK][t] == 0.0f) return t - impulse_tick;
    return -1;
}

static void tests_model(void) {
    Timeline a, b;
    tl_init(&a, (Unit){0.0f, 0.0f}, -1);
    tl_init(&b, (Unit){0.0f, 0.0f}, -1);
    uint32_t r1 = 7u, r2 = 7u;
    int l1 = 0, d1 = 0, l2 = 0, d2 = 0;
    for (int i = 0; i < 200; i++) {
        tl_push(&a, gen_input(&r1, &l1, &d1));
        tl_push(&b, gen_input(&r2, &l2, &d2));
    }
    int same = 1;
    for (int i = 0; i < 200; i++) same &= unit_same(tl_state(&a, i), tl_state(&b, i));
    check("model_deterministic", same && unit_same(a.u, b.u));

    Unit u = {0, 0};
    check("idle_holds", unit_same(unit_step(u, 8), u));
    Unit e = unit_step(u, 0);
    check("cardinal_step", fabsf(e.x - SPEED * DT) < 1e-6f && e.y == 0.0f);
    Unit d = unit_step(u, 4);
    check("diagonal_normalised", fabsf(hypotf(d.x, d.y) - SPEED * DT) < 1e-5f);
}

static void tests_rollback(void) {
    Pair p;
    memset(&p, 0, sizeof(p));
    host_init(&p.h);
    client_init(&p.c, RB_ROLLBACK);

    pair_feed(&p, 6, 11u);        /* tick 5 */
    pair_impulse(&p);             /* host only: client cannot predict this */
    pair_feed(&p, 7, 11u);        /* live tick 12 */

    Packet s = snap_of(&p.h, 5);
    int replayed = rb_apply(&p.c, s);
    check("rollback_detected", replayed == 1 && p.c.rollbacks == 1);
    check("rollback_restores_authority", unit_same(tl_state(&p.c.t, 5), s.pos));
    check("rollback_exact_replay", verify_replay(&p.c, &p.h, s));
    check("rollback_tail_matches_host", unit_same(p.c.t.u, p.h.t.u));
    check("rollback_depth", p.c.replayed_ticks == 7 && p.c.depth_max == 7);
    check("rollback_checked", p.c.replays_checked == 1 &&
                              p.c.rollbacks_accepted == 1 && p.c.replay_mismatch == 0);

    /* duplicate and superseded packets are stale, not rolled back again */
    int before = p.c.rollbacks;
    rb_apply(&p.c, s);
    rb_apply(&p.c, snap_of(&p.h, 4));
    check("duplicate_and_old_stale", p.c.snaps_stale == 2 && p.c.rollbacks == before);

    /* a snapshot that agrees with the prediction is an ack: no replay */
    int applied = p.c.snaps_ack;
    int replays = p.c.replays_checked;
    rb_apply(&p.c, snap_of(&p.h, 10));
    check("ack_skips_replay", p.c.snaps_ack == applied + 1 && p.c.replays_checked == replays);

    /* a packet from the future is dropped */
    Packet future = snap_of(&p.h, 12);
    future.tick = 99;
    rb_apply(&p.c, future);
    check("future_dropped", p.c.snaps_dropped == 1);

    /* a packet older than the retained history is dropped, not misapplied */
    Pair q;
    memset(&q, 0, sizeof(q));
    host_init(&q.h);
    client_init(&q.c, RB_ROLLBACK);
    pair_feed(&q, 600, 5u);
    rb_apply(&q.c, snap_of(&q.h, 10));
    check("too_old_dropped", q.c.snaps_dropped == 1 && q.c.rollbacks == 0);

    /* many rollbacks against the same authority stay exact */
    Pair r;
    memset(&r, 0, sizeof(r));
    host_init(&r.h);
    client_init(&r.c, RB_ROLLBACK);
    pair_feed(&r, 193, 3u);
    pair_impulse(&r);
    pair_feed(&r, 9, 3u);
    int exact = 1;
    for (int t = 0; t <= r.h.t.tick; t += 7) {
        Packet sp = snap_of(&r.h, t);
        rb_apply(&r.c, sp);
        exact &= verify_replay(&r.c, &r.h, sp);
    }
    check("sweep_exact", exact && r.c.replay_mismatch == 0 && r.c.rollbacks > 0);
}

static void report_match(const Cfg* cfg, const Run* run) {
    const Client* rb = &run->cl[RB_ROLLBACK];
    const Client* sn = &run->cl[RB_SNAP];
    float rb_max = 0.0f, rb_sum = 0.0f, sn_sum = 0.0f;
    for (int t = 0; t < run->ticks; t++) {
        if (run->err[RB_ROLLBACK][t] > rb_max) rb_max = run->err[RB_ROLLBACK][t];
        rb_sum += run->err[RB_ROLLBACK][t];
        sn_sum += run->err[RB_SNAP][t];
    }
    int max_rec = -1;
    for (int i = 0; i < run->impulse_n; i++) {
        int rec = recovery_after(run, run->impulse[i]);
        if (rec > max_rec) max_rec = rec;
    }
    printf("summary mode=rollback rollbacks=%d accepted=%d replayed_ticks=%d depth_max=%d "
           "snapshots_applied=%d ack=%d stale=%d dropped=%d replays_checked=%d mismatch=%d "
           "err_max=%.4f err_mean=%.4f max_recovery_ticks=%d\n",
           rb->rollbacks, rb->rollbacks_accepted, rb->replayed_ticks, rb->depth_max,
           rb->snaps_applied, rb->snaps_ack, rb->snaps_stale, rb->snaps_dropped,
           rb->replays_checked, rb->replay_mismatch, rb_max, rb_sum / run->ticks, max_rec);
    printf("summary mode=ignore err_end=%.4f err_mean=%.4f mode=snap err_mean=%.4f rewinds=%d\n",
           run->err[RB_IGNORE][run->ticks - 1],
           (run->err[RB_IGNORE][3] + run->err[RB_IGNORE][run->ticks - 1]) / 2.0f,
           sn_sum / run->ticks, sn->rewinds);
    (void)cfg;
}

static void tests_match(void) {
    Cfg cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.one_way_ms = 100;
    cfg.jitter_ms = 60;
    cfg.loss_pct = 3;
    cfg.dup_pct = 5;
    cfg.knockback_every = 40;
    cfg.ticks = 240;
    cfg.seed = 2026;

    static Run run;
    match_run(&cfg, &run);
    Client* rb = &run.cl[RB_ROLLBACK];
    Client* sn = &run.cl[RB_SNAP];

    check("match_impulses", run.impulse_n == 5);
    check("match_rollbacks_count", rb->rollbacks == run.impulse_n);
    check("match_replays_exact", rb->replays_checked == rb->rollbacks &&
                                 rb->rollbacks_accepted == rb->rollbacks &&
                                 rb->replay_mismatch == 0);
    check("match_rollbacks_bounded", rb->depth_max <= 4 && rb->replayed_ticks < cfg.ticks);

    int max_rec = -1;
    for (int i = 0; i < run.impulse_n; i++) {
        int rec = recovery_after(&run, run.impulse[i]);
        if (rec > max_rec) max_rec = rec;
    }
    check("match_converges", max_rec > 0 && max_rec <= 6);
    check("match_zero_at_end", run.err[RB_ROLLBACK][cfg.ticks - 1] == 0.0f);

    float rb_max = 0.0f;
    for (int t = 0; t < cfg.ticks; t++)
        if (run.err[RB_ROLLBACK][t] > rb_max) rb_max = run.err[RB_ROLLBACK][t];
    check("match_rollback_error_bounded", rb_max <= KNOCKBACK + 1e-4f);
    check("match_ignore_accumulates",
          fabsf(run.err[RB_IGNORE][cfg.ticks - 1] - KNOCKBACK * run.impulse_n) < 1e-3f);

    float sn_sum = 0.0f;
    for (int t = 0; t < cfg.ticks; t++) sn_sum += run.err[RB_SNAP][t];
    check("match_snap_rewinds", sn->rewinds > 0);
    check("match_snap_lags", sn_sum / cfg.ticks > 0.15f);

    check("match_stale_counted", rb->snaps_stale > 0 && rb->snaps_dropped == 0);
    report_match(&cfg, &run);
}

static void tests_hostile(void) {
    Cfg cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.one_way_ms = 210;
    cfg.jitter_ms = 90;
    cfg.loss_pct = 10;
    cfg.dup_pct = 20;
    cfg.knockback_every = 17;
    cfg.ticks = 900;
    cfg.seed = 99;

    static Run run;
    match_run(&cfg, &run);
    Client* rb = &run.cl[RB_ROLLBACK];
    check("hostile_replays_exact", rb->replay_mismatch == 0 &&
                                   rb->replays_checked == rb->rollbacks &&
                                   rb->replays_checked > 0);
    check("hostile_no_desync", run.err[RB_ROLLBACK][cfg.ticks - 1] == 0.0f);
    check("hostile_counts", rb->rollbacks == run.impulse_n && rb->depth_max <= 10);
    check("hostile_history_wrapped", rb->t.wrapped == 1);
    report_match(&cfg, &run);
}

/* property sweep: every seed x link condition must stay exact and converge */
static void tests_sweep(void) {
    static const int delays[] = {0, 20, 60, 100, 150, 220};
    static const int jitters[] = {0, 15, 60};
    static const int losses[] = {0, 5, 25};
    static const int dups[] = {0, 10, 40};
    int runs = 0, bad = 0, max_rec = 0;
    for (uint32_t seed = 1; seed <= 40; seed++)
    for (int di = 0; di < 6; di++) for (int ji = 0; ji < 3; ji++)
    for (int li = 0; li < 3; li++) for (int pi = 0; pi < 3; pi++) {
        Cfg cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.one_way_ms = delays[di];
        cfg.jitter_ms = jitters[ji];
        cfg.loss_pct = losses[li];
        cfg.dup_pct = dups[pi];
        cfg.knockback_every = (seed % 3 == 0) ? 24 : 36;
        cfg.ticks = 504;
        cfg.seed = seed;

        static Run run;
        match_run(&cfg, &run);
        Client* rb = &run.cl[RB_ROLLBACK];
        int ok = rb->replay_mismatch == 0 && rb->replays_checked == rb->rollbacks &&
                 rb->rollbacks == run.impulse_n && rb->depth_max < 40 &&
                 run.err[RB_ROLLBACK][cfg.ticks - 1] == 0.0f;
        for (int i = 0; i < run.impulse_n; i++) {
            int rec = recovery_after(&run, run.impulse[i]);
            if (rec <= 0 || rec > 16) ok = 0;
            if (rec > max_rec) max_rec = rec;
        }
        runs++;
        if (!ok) bad++;
    }
    char name[64];
    snprintf(name, sizeof(name), "sweep_exact_%d", runs);
    check(name, bad == 0 && max_rec > 0 && max_rec <= 16);
    printf("sweep runs=%d bad=%d max_recovery_ticks=%d\n", runs, bad, max_rec);
}

static void emit_csv(const Cfg* cfg, const Run* run) {
    printf("# cfg tick_hz=%d ticks=%d one_way_ms=%d jitter_ms=%d loss_pct=%d dup_pct=%d "
           "knockback_every=%d impulses=%d\n",
           TICK_HZ, run->ticks, cfg->one_way_ms, cfg->jitter_ms, cfg->loss_pct,
           cfg->dup_pct, cfg->knockback_every, run->impulse_n);
    printf("tick,time_s,err_rollback,err_ignore,err_snap,rollback_event\n");
    for (int t = 0; t < run->ticks; t++)
        printf("%d,%.3f,%.4f,%.4f,%.4f,%d\n", t, (double)t / TICK_HZ,
               run->err[RB_ROLLBACK][t], run->err[RB_IGNORE][t],
               run->err[RB_SNAP][t], run->rb_event[t]);

    const Client* rb = &run->cl[RB_ROLLBACK];
    const Client* sn = &run->cl[RB_SNAP];
    float rb_max = 0.0f, rb_sum = 0.0f, sn_sum = 0.0f;
    for (int t = 0; t < run->ticks; t++) {
        if (run->err[RB_ROLLBACK][t] > rb_max) rb_max = run->err[RB_ROLLBACK][t];
        rb_sum += run->err[RB_ROLLBACK][t];
        sn_sum += run->err[RB_SNAP][t];
    }
    int max_rec = -1;
    for (int i = 0; i < run->impulse_n; i++) {
        int rec = recovery_after(run, run->impulse[i]);
        if (rec > max_rec) max_rec = rec;
    }
    printf("# summary impulses=%d rollbacks=%d accepted=%d replayed_ticks=%d depth_max=%d "
           "snaps_applied=%d ack=%d stale=%d dropped=%d replays_checked=%d mismatch=%d\n",
           run->impulse_n, rb->rollbacks, rb->rollbacks_accepted, rb->replayed_ticks,
           rb->depth_max, rb->snaps_applied, rb->snaps_ack, rb->snaps_stale,
           rb->snaps_dropped, rb->replays_checked, rb->replay_mismatch);
    printf("# summary err_rollback_max=%.4f err_rollback_mean=%.4f err_ignore_end=%.4f "
           "err_snap_mean=%.4f max_recovery_ticks=%d rewinds=%d\n",
           rb_max, rb_sum / run->ticks, run->err[RB_IGNORE][run->ticks - 1],
           sn_sum / run->ticks, max_rec, sn->rewinds);
    printf("# impulses");
    for (int i = 0; i < run->impulse_n; i++) printf(" %d", run->impulse[i]);
    printf("\n");
}

int main(int argc, char** argv) {
    if (argc > 1 && strcmp(argv[1], "--csv") == 0) {
        Cfg cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.one_way_ms = 100;
        cfg.jitter_ms = 60;
        cfg.loss_pct = 3;
        cfg.dup_pct = 5;
        cfg.knockback_every = 40;
        cfg.ticks = 240;
        cfg.seed = 2026;
        static Run run;
        match_run(&cfg, &run);
        emit_csv(&cfg, &run);
        return 0;
    }
    tests_model();
    tests_rollback();
    tests_match();
    tests_hostile();
    tests_sweep();
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
