
/* skillsched: fixed-capacity ability request scheduler.

   - Cooldown and multi-resource gating on every cast attempt.
   - FIFO-preserving request queue; each tick scans every live request once
     (O(n)) and compacts dead slots in a single O(n) pass.
   - Resources are integers. The exact deducted cost is snapshotted per cast
     and restored verbatim on interrupt, so rollback error is always zero.
   - One active cast at a time (global cast lock). Deterministic tick order:
     regen -> cooldown decrement -> active cast settle -> queue scan. */
#include <stdio.h>
#include <string.h>
#include <time.h>

#define SKILL_MAX 16
#define RES_KINDS 2          /* 0 = mana, 1 = energy */
#define QUEUE_CAP 128
#define LOG_CAP 512

typedef struct {
    int cooldown;                       /* frames, 0 = ready immediately */
    int cast_time;                      /* frames the cast lock is held */
    int cost[RES_KINDS];
} SkillDef;

typedef struct {
    int amount[RES_KINDS];
    int regen[RES_KINDS];              /* gained per frame */
    int cap[RES_KINDS];
} Pool;

typedef enum { REQ_WAITING = 0, REQ_CAST } ReqState;

typedef struct {
    int seq;
    int skill;
    int state;
} Request;

enum { EV_CAST = 0, EV_DONE, EV_INTERRUPT };

typedef struct {
    int kind;
    int skill;
    int seq;
    int frame;
} Event;

typedef struct {
    SkillDef defs[SKILL_MAX];
    int nskills;
    int cd[SKILL_MAX];                 /* remaining cooldown per skill */
    Pool pool;

    Request q[QUEUE_CAP];
    int qn;

    Event log[LOG_CAP];
    int logn;

    int frame;
    int active_seq;                    /* -1 when no cast is in flight */
    int active_skill;
    int active_left;
    int active_cost[RES_KINDS];        /* snapshot for exact rollback */

    int next_seq;
    long scans;                        /* request slots examined */
    long moves;                        /* slots copied during compaction */
} Scheduler;

static void sched_init(Scheduler* s, const SkillDef* defs, int n, Pool pool) {
    memset(s, 0, sizeof(*s));
    s->nskills = n > SKILL_MAX ? SKILL_MAX : n;
    for (int i = 0; i < s->nskills; i++) {
        s->defs[i] = defs[i];
        if (s->defs[i].cast_time < 1) s->defs[i].cast_time = 1;
    }
    s->pool = pool;
    s->active_seq = -1;
    s->active_skill = -1;
}

static void sched_push(Scheduler* s, int kind, int skill, int seq) {
    if (s->logn < LOG_CAP)
        s->log[s->logn++] = (Event){ kind, skill, seq, s->frame };
}

/* Enqueue a cast request. Returns the sequence id, or -1 when the skill id
   is invalid or the queue is full. */
static int sched_request(Scheduler* s, int skill) {
    if (skill < 0 || skill >= s->nskills) return -1;
    if (s->qn >= QUEUE_CAP) return -1;
    Request* rq = &s->q[s->qn++];
    rq->seq = s->next_seq++;
    rq->skill = skill;
    rq->state = REQ_WAITING;
    return rq->seq;
}

static int sched_can_cast(const Scheduler* s, int skill) {
    if (s->active_seq >= 0) return 0;              /* cast lock held */
    if (s->cd[skill] != 0) return 0;               /* cooling down */
    for (int r = 0; r < RES_KINDS; r++) {
        if (s->pool.amount[r] < s->defs[skill].cost[r]) return 0;
    }
    return 1;
}

static void sched_begin(Scheduler* s, const Request* rq) {
    int sk = rq->skill;
    for (int r = 0; r < RES_KINDS; r++) {
        s->pool.amount[r] -= s->defs[sk].cost[r];
        s->active_cost[r] = s->defs[sk].cost[r];   /* rollback snapshot */
    }
    s->cd[sk] = s->defs[sk].cooldown;
    s->active_seq = rq->seq;
    s->active_skill = sk;
    s->active_left = s->defs[sk].cast_time;
    sched_push(s, EV_CAST, sk, rq->seq);
}

static void sched_tick(Scheduler* s) {
    s->frame++;

    for (int r = 0; r < RES_KINDS; r++) {
        int v = s->pool.amount[r] + s->pool.regen[r];
        if (v > s->pool.cap[r]) v = s->pool.cap[r];
        s->pool.amount[r] = v;
    }
    for (int i = 0; i < s->nskills; i++)
        if (s->cd[i] > 0) s->cd[i]--;

    if (s->active_seq >= 0) {
        s->active_left--;
        if (s->active_left == 0) {
            sched_push(s, EV_DONE, s->active_skill, s->active_seq);
            s->active_seq = -1;
            s->active_skill = -1;
            memset(s->active_cost, 0, sizeof(s->active_cost));
        }
    }

    /* Single pass over the live queue: every slot is examined exactly once,
       so the scan cost is O(qn). Requests that fire are marked dead; the
       rest keep their relative submission order. */
    for (int i = 0; i < s->qn; i++) {
        s->scans++;
        Request* rq = &s->q[i];
        if (rq->state == REQ_WAITING && sched_can_cast(s, rq->skill)) {
            sched_begin(s, rq);
            rq->state = REQ_CAST;
        }
    }

    /* One compaction pass, also O(qn). */
    int alive = 0;
    for (int i = 0; i < s->qn; i++) {
        if (s->q[i].state == REQ_WAITING) {
            if (alive != i) s->q[alive] = s->q[i];
            s->moves++;
            alive++;
        }
    }
    s->qn = alive;
}

/* Interrupt the in-flight cast and restore every deducted resource.
   Refunds bypass the regen cap: they restore resources, they do not
   generate them, so rollback is exact. Cooldown started at cast time
   is retained. Returns -1 when nothing is casting. */
static int sched_interrupt(Scheduler* s) {
    if (s->active_seq < 0) return -1;
    for (int r = 0; r < RES_KINDS; r++)
        s->pool.amount[r] += s->active_cost[r];
    sched_push(s, EV_INTERRUPT, s->active_skill, s->active_seq);
    s->active_seq = -1;
    s->active_skill = -1;
    s->active_left = 0;
    memset(s->active_cost, 0, sizeof(s->active_cost));
    return 0;
}

/* ------------------------------------------------------------------ */
/* Frame-by-frame reference simulator: independent implementation of  */
/* the same rules, used as a differential oracle.                     */
/* ------------------------------------------------------------------ */

typedef struct {
    int nskills;
    int cd[SKILL_MAX];
    int amt[RES_KINDS];
    int qsk[QUEUE_CAP], qseq[QUEUE_CAP], qn;
    int active_seq, active_sk, active_left;
    int acost[RES_KINDS];
    int frame;
    Event log[LOG_CAP];
    int logn;
} Ref;

static void ref_init(Ref* r, int nskills, Pool p) {
    memset(r, 0, sizeof(*r));
    r->nskills = nskills;
    for (int k = 0; k < RES_KINDS; k++) r->amt[k] = p.amount[k];
    r->active_seq = -1;
    r->active_sk = -1;
    r->frame = 0;
}

static void ref_push(Ref* r, int kind, int sk, int seq) {
    if (r->logn < LOG_CAP)
        r->log[r->logn++] = (Event){ kind, sk, seq, r->frame };
}

static int ref_request(Ref* r, int nskills, int sk, int seq) {
    if (sk < 0 || sk >= nskills) return -1;
    if (r->qn >= QUEUE_CAP) return -1;
    r->qsk[r->qn] = sk;
    r->qseq[r->qn] = seq;
    r->qn++;
    return seq;
}

static void ref_tick(Ref* r, const SkillDef* defs, int nskills, const Pool* cfg) {
    r->frame++;
    for (int k = 0; k < RES_KINDS; k++) {
        int v = r->amt[k] + cfg->regen[k];
        if (v > cfg->cap[k]) v = cfg->cap[k];
        r->amt[k] = v;
    }
    for (int i = 0; i < nskills; i++)
        if (r->cd[i] > 0) r->cd[i]--;

    if (r->active_seq >= 0) {
        r->active_left--;
        if (r->active_left == 0) {
            ref_push(r, EV_DONE, r->active_sk, r->active_seq);
            r->active_seq = -1;
            r->active_sk = -1;
            memset(r->acost, 0, sizeof(r->acost));
        }
    }

    int dead[QUEUE_CAP];
    memset(dead, 0, sizeof(dead));
    for (int i = 0; i < r->qn; i++) {
        int sk = r->qsk[i];
        int ok = r->active_seq < 0 && r->cd[sk] == 0;
        for (int k = 0; ok && k < RES_KINDS; k++)
            if (r->amt[k] < defs[sk].cost[k]) ok = 0;
        if (ok) {
            for (int k = 0; k < RES_KINDS; k++) {
                r->amt[k] -= defs[sk].cost[k];
                r->acost[k] = defs[sk].cost[k];
            }
            r->cd[sk] = defs[sk].cooldown;
            r->active_seq = r->qseq[i];
            r->active_sk = sk;
            r->active_left = defs[sk].cast_time < 1 ? 1 : defs[sk].cast_time;
            ref_push(r, EV_CAST, sk, r->qseq[i]);
            dead[i] = 1;
        }
    }
    int alive = 0;
    for (int i = 0; i < r->qn; i++) {
        if (!dead[i]) {
            r->qsk[alive] = r->qsk[i];
            r->qseq[alive] = r->qseq[i];
            alive++;
        }
    }
    r->qn = alive;
}

static int ref_interrupt(Ref* r) {
    if (r->active_seq < 0) return -1;
    for (int k = 0; k < RES_KINDS; k++) r->amt[k] += r->acost[k];
    ref_push(r, EV_INTERRUPT, r->active_sk, r->active_seq);
    r->active_seq = -1;
    r->active_sk = -1;
    r->active_left = 0;
    memset(r->acost, 0, sizeof(r->acost));
    return 0;
}

/* Replay-only verifier: given the scheduler's event log, re-derive every
   state transition from scratch and reject any cast that could not legally
   fire. Shares no code with the scheduler. */
static int verify_log(const Event* ev, int n, const SkillDef* defs, int nskills,
                      Pool start, int last_frame, const int* final_amt) {
    int cd[SKILL_MAX] = { 0 };
    int amt[RES_KINDS];
    for (int k = 0; k < RES_KINDS; k++) amt[k] = start.amount[k];
    int lock_left = 0, lock_sk = -1, lock_cast_at = -1;
    int idx = 0;

    for (int f = 1; f <= last_frame; f++) {
        for (int k = 0; k < RES_KINDS; k++) {
            int v = amt[k] + start.regen[k];
            if (v > start.cap[k]) v = start.cap[k];
            amt[k] = v;
        }
        for (int i = 0; i < nskills; i++)
            if (cd[i] > 0) cd[i]--;
        if (lock_left > 0) lock_left--;   /* unlock only via the event below */

        while (idx < n && ev[idx].frame == f) {
            const Event* e = &ev[idx++];
            if (e->skill < 0 || e->skill >= nskills) return 0;  /* OOB cast */
            if (e->kind == EV_CAST) {
                if (lock_sk != -1) return 0;                     /* lock busy */
                if (cd[e->skill] != 0) return 0;                 /* cooldown */
                for (int k = 0; k < RES_KINDS; k++)
                    if (amt[k] < defs[e->skill].cost[k]) return 0; /* broke */
                for (int k = 0; k < RES_KINDS; k++)
                    amt[k] -= defs[e->skill].cost[k];
                cd[e->skill] = defs[e->skill].cooldown;
                lock_left = defs[e->skill].cast_time;
                lock_sk = e->skill;
                lock_cast_at = f;
            } else if (e->kind == EV_INTERRUPT) {
                if (lock_sk != e->skill) return 0;
                for (int k = 0; k < RES_KINDS; k++)
                    amt[k] += defs[e->skill].cost[k];
                lock_left = 0;
                lock_sk = -1;
                lock_cast_at = -1;
            } else { /* EV_DONE: must match the cast's declared duration */
                if (lock_sk != e->skill) return 0;
                if (f - lock_cast_at != defs[e->skill].cast_time) return 0;
                if (lock_left != 0) return 0;
                lock_sk = -1;
                lock_cast_at = -1;
            }
            for (int k = 0; k < RES_KINDS; k++)
                if (amt[k] < 0) return 0;
        }
    }
    if (idx != n) return 0;
    for (int k = 0; k < RES_KINDS; k++)
        if (amt[k] != final_amt[k]) return 0;
    return 1;
}

/* ------------------------------ tests ------------------------------ */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static unsigned lcg_state;
static unsigned lcg(void) {
    lcg_state = lcg_state * 1103515245u + 12345u;
    return lcg_state;
}

/* One full differential scenario: 100 mixed requests against the
   independent reference, with burst submissions and random interrupts.
   Returns a bitmask: 1 state, 2 sequence, 4 legality, 8 enqueue. */
static int diff_scenario(unsigned seed, int interrupt_every) {
    SkillDef defs[4] = {
        { 3, 2, { 10, 0 } },
        { 5, 3, { 0, 20 } },
        { 8, 4, { 15, 15 } },
        { 2, 1, { 5, 5 } },
    };
    Pool start = { { 100, 80 }, { 3, 2 }, { 120, 100 } };
    Scheduler s;
    Ref r;
    sched_init(&s, defs, 4, start);
    ref_init(&r, 4, start);

    enum { NREQ = 100, NFRAME = 200 };
    int sf[NREQ], ss[NREQ];
    lcg_state = seed;
    for (int i = 0; i < NREQ; i++) {
        sf[i] = (int)(lcg() % 120);
        ss[i] = (int)(lcg() % 4);
        if (i % 10 == 0 && i > 0) sf[i] = sf[i - 1]; /* bursts */
    }

    int rc = 15, enq_ok = 1, frame_match = 1;
    for (int f = 0; f < NFRAME; f++) {
        for (int i = 0; i < NREQ; i++) {
            if (sf[i] != f) continue;
            int a = sched_request(&s, ss[i]);
            int b = ref_request(&r, 4, ss[i], a);
            if (a < 0 || b != a) enq_ok = 0;
        }
        sched_tick(&s);
        ref_tick(&r, defs, 4, &start);
        if (interrupt_every > 0 && f % interrupt_every == interrupt_every / 2) {
            if (sched_interrupt(&s) != ref_interrupt(&r)) frame_match = 0;
        }
        for (int k = 0; k < RES_KINDS; k++)
            if (s.pool.amount[k] != r.amt[k]) frame_match = 0;
        for (int k = 0; k < 4; k++)
            if (s.cd[k] != r.cd[k]) frame_match = 0;
        if (s.active_seq != r.active_seq) frame_match = 0;
        if (s.qn != r.qn) frame_match = 0;
        for (int i = 0; i < s.qn; i++)
            if (s.q[i].seq != r.qseq[i] || s.q[i].skill != r.qsk[i])
                frame_match = 0;
    }
    if (!enq_ok) rc &= ~8;
    if (!frame_match) rc &= ~1;

    int logs_equal = s.logn == r.logn;
    if (logs_equal)
        for (int i = 0; i < s.logn; i++)
            if (s.log[i].kind != r.log[i].kind ||
                s.log[i].skill != r.log[i].skill ||
                s.log[i].seq != r.log[i].seq ||
                s.log[i].frame != r.log[i].frame)
                logs_equal = 0;
    if (!logs_equal) rc &= ~2;

    if (!verify_log(s.log, s.logn, defs, 4, start, NFRAME, s.pool.amount))
        rc &= ~4;
    return rc;
}

int main(void) {
    /* Hand-computed cooldown gating: casts land on frames 1 and 5. */
    {
        SkillDef d[1] = { { 4, 1, { 10, 0 } } };
        Pool p = { { 100, 0 }, { 0, 0 }, { 200, 100 } };
        Scheduler s;
        sched_init(&s, d, 1, p);
        check("cd_enqueue_a", sched_request(&s, 0) >= 0);
        check("cd_enqueue_b", sched_request(&s, 0) >= 0);
        for (int f = 0; f < 6; f++) sched_tick(&s);
        check("cd_two_casts", s.logn == 4);  /* cast f1, done f2, cast f5 */
        check("cd_first_frame", s.log[0].frame == 1 && s.log[0].kind == EV_CAST);
        check("cd_second_frame", s.log[2].frame == 5 && s.log[2].kind == EV_CAST);
    }

    /* Resource gating with regen: 5 + 2/frame, cost 10 -> fires frame 3. */
    {
        SkillDef d[1] = { { 0, 1, { 10, 0 } } };
        Pool p = { { 5, 0 }, { 2, 0 }, { 200, 100 } };
        Scheduler s;
        sched_init(&s, d, 1, p);
        sched_request(&s, 0);
        for (int f = 0; f < 4; f++) sched_tick(&s);
        check("res_wait_then_fire", s.logn == 2 && s.log[0].frame == 3);
        check("res_exact_deduct", s.pool.amount[0] == 3); /* 13 - 10 */
    }

    /* A missing second resource blocks the cast even when mana is ample. */
    {
        SkillDef d[1] = { { 0, 1, { 5, 10 } } };
        Pool p = { { 100, 0 }, { 0, 3 }, { 200, 100 } };
        Scheduler s;
        sched_init(&s, d, 1, p);
        sched_request(&s, 0);
        for (int f = 0; f < 4; f++) sched_tick(&s);
        check("multi_res_blocks", s.logn == 1 && s.log[0].frame == 4);
        check("multi_res_leftover", s.pool.amount[1] == 2); /* 12 - 10 */
    }

    /* Exact rollback, hand arithmetic, no regen: 100 - 30 + 30 == 100. */
    {
        SkillDef d[1] = { { 6, 5, { 30, 0 } } };
        Pool p = { { 100, 50 }, { 0, 0 }, { 200, 100 } };
        Scheduler s;
        sched_init(&s, d, 1, p);
        sched_request(&s, 0);
        sched_tick(&s);                          /* frame 1: cast, mana 70 */
        check("rb_deducted", s.pool.amount[0] == 70 && s.active_seq >= 0);
        sched_tick(&s);                          /* frame 2: still casting */
        check("rb_interrupt", sched_interrupt(&s) == 0);
        check("rb_mana_zero_error", s.pool.amount[0] == 100);
        check("rb_energy_untouched", s.pool.amount[1] == 50);
        check("rb_cooldown_kept", s.cd[0] == 5); /* 6 set at f1, ticked f2 */
        check("rb_double_interrupt", sched_interrupt(&s) == -1);
    }

    /* Rollback with regen arriving mid-cast: refund is not cap-clamped. */
    {
        SkillDef d[1] = { { 0, 2, { 30, 10 } } };
        Pool p = { { 100, 50 }, { 2, 1 }, { 200, 100 } };
        Scheduler s;
        sched_init(&s, d, 1, p);
        sched_request(&s, 0);
        sched_tick(&s);   /* f1: mana 102-30=72, energy 51-10=41 */
        sched_tick(&s);   /* f2: mana 74, energy 42 */
        sched_interrupt(&s);
        check("rb_regen_mana", s.pool.amount[0] == 104);
        check("rb_regen_energy", s.pool.amount[1] == 52);
    }

    /* Completed casts keep their cost; interrupting idle state is a no-op. */
    {
        SkillDef d[1] = { { 0, 2, { 30, 0 } } };
        Pool p = { { 100, 0 }, { 0, 0 }, { 200, 100 } };
        Scheduler s;
        sched_init(&s, d, 1, p);
        check("idle_interrupt", sched_interrupt(&s) == -1);
        sched_request(&s, 0);
        sched_tick(&s);                 /* f1 cast */
        sched_tick(&s);                 /* f2 */
        sched_tick(&s);                 /* f3 done */
        check("done_logged", s.log[s.logn - 1].kind == EV_DONE);
        check("done_cost_kept", s.pool.amount[0] == 70);
        check("interrupt_after_done", sched_interrupt(&s) == -1);
        check("pool_still_70", s.pool.amount[0] == 70);
    }

    /* Invalid ids are rejected outright and can never reach the log. */
    {
        SkillDef d[2] = { { 1, 1, { 0, 0 } }, { 1, 1, { 0, 0 } } };
        Pool p = { { 0, 0 }, { 0, 0 }, { 100, 100 } };
        Scheduler s;
        sched_init(&s, d, 2, p);
        check("bad_id_neg", sched_request(&s, -1) == -1);
        check("bad_id_high", sched_request(&s, 2) == -1);
        check("bad_id_wild", sched_request(&s, 99) == -1);
        for (int f = 0; f < 3; f++) sched_tick(&s);
        check("bad_id_no_cast", s.logn == 0 && s.qn == 0);
    }

    /* Queue saturation: 100 requests fit, the 129th is refused. */
    {
        SkillDef d[1] = { { 0, 1, { 999, 999 } } };
        Pool p = { { 0, 0 }, { 0, 0 }, { 10000, 10000 } };
        Scheduler s;
        sched_init(&s, d, 1, p);
        int all_fit = 1;
        for (int i = 0; i < 100; i++)
            if (sched_request(&s, 0) < 0) all_fit = 0;
        check("hundred_enqueued", all_fit && s.qn == 100);
        int extra = 0;
        for (int i = 100; i < QUEUE_CAP; i++)
            if (sched_request(&s, 0) < 0) { extra = -1; break; }
        check("queue_full_rejects", extra == 0 && sched_request(&s, 0) == -1);
    }

    /* Differential tests: 100 mixed requests per scenario vs the independent
       reference, frame by frame, across several seeds and interrupt rates.
       Bitmask 15 = state + sequence + legality + enqueue all matched. */
    {
        unsigned seeds[] = { 0x1234abcdu, 0x9e3779b9u, 0xdeadbeefu,
                             0x0badf00du, 7u };
        int rates[] = { 7, 3, 2, 5, 0 };   /* 0 = no interrupts */
        int all_match = 1;
        for (size_t i = 0; i < sizeof(seeds) / sizeof(seeds[0]); i++) {
            int rc = diff_scenario(seeds[i], rates[i]);
            printf("diff_seed=%x rate=%d rc=%d\n", seeds[i], rates[i], rc);
            if (rc != 15) all_match = 0;
        }
        check("diff_all_enqueued", all_match);
        check("diff_frame_state", all_match);
        check("diff_cast_sequence", all_match);
        check("diff_pending_order", all_match);
        check("no_illegal_cast", all_match);
    }

    /* Linear cost: a blocked queue keeps n live requests; each tick must
       examine exactly n slots and copy at most n slots. */
    {
        SkillDef d[1] = { { 0, 1, { 999, 0 } } };
        Pool p = { { 0, 0 }, { 0, 0 }, { 10000, 10000 } };
        int sizes[] = { 1, 10, 25, 50, 100 };
        int linear = 1;
        clock_t t0 = clock();
        for (size_t z = 0; z < sizeof(sizes) / sizeof(sizes[0]); z++) {
            int n = sizes[z];
            Scheduler s;
            sched_init(&s, d, 1, p);
            for (int i = 0; i < n; i++)
                if (sched_request(&s, 0) < 0) linear = 0;
            long scans0 = s.scans, moves0 = s.moves;
            enum { K = 50 };
            for (int f = 0; f < K; f++) sched_tick(&s);
            long dscan = s.scans - scans0;
            long dmove = s.moves - moves0;
            if (dscan != (long)K * n) linear = 0;
            if (dmove > (long)K * n) linear = 0;
            if (s.qn != n) linear = 0;
            printf("linear_n=%d scans/tick=%ld moves/tick=%ld\n",
                   n, dscan / K, dmove / K);
        }
        clock_t t1 = clock();
        check("scan_linear_in_queue", linear);
        printf("linear_bench_ms=%d\n",
               (int)((t1 - t0) * 1000 / CLOCKS_PER_SEC));
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
