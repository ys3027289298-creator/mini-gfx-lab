
/* inputbuf: frame-driven input buffer and command queue for combo inputs.
   - buffer window: a command stays consumable through frame (issued + window);
     past that frame it expires and is counted as dropped input
   - same-frame presses merge by priority (higher wins; ties keep the latest)
   - fixed-capacity pending queue: overflow evicts the lowest-priority command,
     otherwise the incoming press is rejected; both are counted
   - buffered commands pass through the same try_execute() handler as immediate
     presses, so a buffer hit yields exactly the action an immediate press would
   One tick == one logic frame; all timing is integer frames so two runs with
   the same script and window produce byte-identical results. */
#include <stdio.h>
#include <string.h>
#include <limits.h>

#define MAX_PENDING 4
#define MAX_LOG 64

typedef enum {
    ACT_NONE = 0, ACT_LIGHT, ACT_HEAVY, ACT_SKILL, ACT_DASH, ACT_GUARD, ACT_COUNT
} Action;

typedef struct { const char* name; int priority; int lock; } ActionDef;

static const ActionDef ACTS[ACT_COUNT] = {
    [ACT_NONE]  = {"none",  0, 0},
    [ACT_GUARD] = {"guard", 1, 2},
    [ACT_LIGHT] = {"light", 2, 3},
    [ACT_DASH]  = {"dash",  3, 4},
    [ACT_SKILL] = {"skill", 4, 8},
    [ACT_HEAVY] = {"heavy", 5, 6},
};

typedef struct {
    Action act;
    int priority;
    int issued;     /* frame when the press happened */
    int expires;    /* last frame on which it may be consumed */
    int seq;        /* global press order, for stable tie-breaking */
} Cmd;

typedef struct {
    Cmd pending[MAX_PENDING];
    int count;
    Cmd staged;             /* merged command for the frame in progress */
    int has_staged;
    int window;             /* buffer window, in frames */
    int frame;
    int lock_until;         /* character is busy while frame < lock_until */
    int seq_next;
    /* drop accounting */
    int submitted, executed;
    int expired_drop, merged_drop, evicted_drop, full_drop;
    /* observed character action sequence */
    Action log[MAX_LOG];
    int exec_frame[MAX_LOG];
    int log_buffered[MAX_LOG]; /* 1 = consumed from the pending queue */
    int log_count;
} InputBuffer;

static void ib_init(InputBuffer* ib, int window) {
    memset(ib, 0, sizeof(*ib));
    ib->window = window;
}

/* Only commands enqueued afterwards pick up the new window; commands already
   pending keep the expiry they were queued with. */
static void ib_set_window(InputBuffer* ib, int window) { ib->window = window; }

static int ib_busy(const InputBuffer* ib) { return ib->frame < ib->lock_until; }

static int ib_dropped(const InputBuffer* ib) {
    return ib->expired_drop + ib->merged_drop + ib->evicted_drop + ib->full_drop;
}

/* The single action handler shared by immediate presses and buffer hits. */
static int try_execute(InputBuffer* ib, const Cmd* c, int buffered) {
    if (ib_busy(ib) || ib->log_count >= MAX_LOG) return 0;
    ib->lock_until = ib->frame + ACTS[c->act].lock;
    ib->log[ib->log_count] = c->act;
    ib->exec_frame[ib->log_count] = ib->frame;
    ib->log_buffered[ib->log_count] = buffered;
    ib->log_count++;
    ib->executed++;
    return 1;
}

/* Fold one press into the current frame's staged command. */
static void stage(InputBuffer* ib, Action act) {
    ib->submitted++;
    Cmd c;
    c.act = act;
    c.priority = ACTS[act].priority;
    c.issued = ib->frame;
    c.expires = ib->frame + ib->window;
    c.seq = ib->seq_next++;
    if (!ib->has_staged) { ib->staged = c; ib->has_staged = 1; return; }
    /* same-frame merge: higher priority wins, ties keep the latest press */
    ib->merged_drop++;
    if (c.priority >= ib->staged.priority) ib->staged = c;
}

static void enqueue(InputBuffer* ib, const Cmd* c) {
    if (ib->count < MAX_PENDING) { ib->pending[ib->count++] = *c; return; }
    int worst = 0;
    for (int i = 1; i < ib->count; i++) {
        const Cmd* p = &ib->pending[i];
        const Cmd* w = &ib->pending[worst];
        if (p->priority < w->priority ||
            (p->priority == w->priority && p->seq > w->seq))
            worst = i;
    }
    if (c->priority > ib->pending[worst].priority) {
        ib->evicted_drop++;
        ib->pending[worst] = *c;
    } else {
        ib->full_drop++;
    }
}

static void sweep_expired(InputBuffer* ib) {
    int w = 0;
    for (int i = 0; i < ib->count; i++) {
        if (ib->frame > ib->pending[i].expires) { ib->expired_drop++; continue; }
        ib->pending[w++] = ib->pending[i];
    }
    ib->count = w;
}

/* Highest priority first; ties go to the earliest press (stable FIFO). */
static void consume_pending(InputBuffer* ib) {
    if (ib->count == 0 || ib_busy(ib)) return;
    int best = 0;
    for (int i = 1; i < ib->count; i++) {
        const Cmd* a = &ib->pending[i];
        const Cmd* b = &ib->pending[best];
        if (a->priority > b->priority ||
            (a->priority == b->priority &&
             (a->issued < b->issued ||
              (a->issued == b->issued && a->seq < b->seq))))
            best = i;
    }
    Cmd c = ib->pending[best];
    ib->pending[best] = ib->pending[--ib->count];
    try_execute(ib, &c, 1);
}

/* Advance one logic frame, applying every press observed during that frame. */
static void ib_step(InputBuffer* ib, int frame, const Action* presses, int n) {
    ib->frame = frame;
    sweep_expired(ib);
    for (int i = 0; i < n; i++) stage(ib, presses[i]);
    if (ib->has_staged) {
        if (!try_execute(ib, &ib->staged, 0)) enqueue(ib, &ib->staged);
        ib->has_staged = 0;
    }
    consume_pending(ib);
}

static void ib_print_stats(const InputBuffer* ib, const char* tag) {
    printf("%s: submitted=%d executed=%d expired=%d merged=%d evicted=%d "
           "full=%d dropped=%d\n",
           tag, ib->submitted, ib->executed, ib->expired_drop,
           ib->merged_drop, ib->evicted_drop, ib->full_drop, ib_dropped(ib));
}

/* ---- scripted scenario -------------------------------------------------- */

typedef struct { int frame; Action presses[4]; int n; } InputEvent;

static const InputEvent SCRIPT[] = {
    {0,  {ACT_LIGHT}, 1},
    {1,  {ACT_HEAVY, ACT_LIGHT}, 2},
    {2,  {ACT_SKILL}, 1},
    {4,  {ACT_DASH, ACT_GUARD}, 2},
    {5,  {ACT_LIGHT}, 1},
    {6,  {ACT_HEAVY, ACT_SKILL}, 2},
    {8,  {ACT_GUARD, ACT_LIGHT}, 2},
    {9,  {ACT_DASH}, 1},
    {10, {ACT_SKILL, ACT_HEAVY}, 2},
};
#define SCRIPT_EVENTS (int)(sizeof(SCRIPT) / sizeof(SCRIPT[0]))
#define LAST_FRAME 20   /* tail frames let buffered commands expire or fire */

static void run_script(int window, InputBuffer* out) {
    ib_init(out, window);
    int ev = 0;
    for (int f = 0; f <= LAST_FRAME; f++) {
        const Action* p = NULL; int n = 0;
        if (ev < SCRIPT_EVENTS && SCRIPT[ev].frame == f) {
            p = SCRIPT[ev].presses; n = SCRIPT[ev].n; ev++;
        }
        ib_step(out, f, p, n);
    }
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int accounting_ok(const InputBuffer* ib) {
    return ib->submitted == ib->executed + ib_dropped(ib);
}

int main(void) {
    /* 1. immediate press while idle executes on the same frame */
    InputBuffer a; ib_init(&a, 0);
    Action p1[] = {ACT_LIGHT};
    ib_step(&a, 0, p1, 1);
    check("immediate_same_frame", a.log_count == 1 && a.exec_frame[0] == 0 &&
          a.log[0] == ACT_LIGHT && a.lock_until == ACTS[ACT_LIGHT].lock);

    /* 2. window 0 == pure immediate model: a fast follow-up press is dropped */
    InputBuffer b; ib_init(&b, 0);
    Action b0[] = {ACT_LIGHT};
    ib_step(&b, 0, b0, 1);
    Action b1[] = {ACT_HEAVY};
    ib_step(&b, 1, b1, 1);
    ib_step(&b, 2, NULL, 0);
    check("window0_drops_fast_input", b.executed == 1 && b.expired_drop == 1 &&
          b.log_count == 1 && accounting_ok(&b));

    /* 3. buffer hit: heavy pressed during light's lock fires when idle opens,
          and an immediate oracle given the same command at that frame produces
          the identical action and character state. */
    InputBuffer c; ib_init(&c, 3);
    Action c0[] = {ACT_LIGHT};
    ib_step(&c, 0, c0, 1);
    Action c1[] = {ACT_HEAVY};
    ib_step(&c, 1, c1, 1);
    ib_step(&c, 2, NULL, 0);
    ib_step(&c, 3, NULL, 0);
    check("buffer_hit_delayed", c.log_count == 2 && c.log[1] == ACT_HEAVY &&
          c.exec_frame[1] == 3 && c.log_buffered[1] == 1 && c.expired_drop == 0);
    ib_print_stats(&c, "buffer_hit");
    InputBuffer oracle; ib_init(&oracle, 0);
    Action oh[] = {ACT_HEAVY};
    ib_step(&oracle, 3, oh, 1);
    check("buffer_hit_matches_immediate",
          oracle.log[0] == c.log[1] && oracle.exec_frame[0] == c.exec_frame[1] &&
          oracle.lock_until == c.lock_until);

    /* 4. expiry boundary: window W means consumable through issued+W,
          dropped on the next frame, and the drop is counted. */
    InputBuffer d; ib_init(&d, 2);
    Action d0[] = {ACT_SKILL};   /* locks frames 0..7, idle again at 8 */
    ib_step(&d, 0, d0, 1);
    Action d1[] = {ACT_HEAVY};   /* issued 1, expires 3 */
    ib_step(&d, 1, d1, 1);
    ib_step(&d, 2, NULL, 0);
    ib_step(&d, 3, NULL, 0);
    check("expiry_inclusive_last_frame", d.expired_drop == 0 && d.count == 1);
    ib_step(&d, 4, NULL, 0);
    check("expiry_dropped_and_counted", d.expired_drop == 1 && d.count == 0 &&
          d.executed == 1 && accounting_ok(&d));

    /* 5. same-frame merge by priority; ties keep the latest press */
    InputBuffer e; ib_init(&e, 0);
    Action e0[] = {ACT_LIGHT, ACT_HEAVY, ACT_SKILL};
    ib_step(&e, 0, e0, 3);
    check("merge_priority_wins", e.log[0] == ACT_HEAVY && e.merged_drop == 2);
    InputBuffer e2; ib_init(&e2, 0);
    Action e2f[] = {ACT_DASH, ACT_DASH};
    ib_step(&e2, 0, e2f, 2);
    check("merge_tie_latest", e2.log_count == 1 && e2.log[0] == ACT_DASH &&
          e2.merged_drop == 1);

    /* 6. queue overflow: higher priority evicts the worst entry, an equal or
          weaker press is rejected outright; both are counted separately. */
    InputBuffer f; ib_init(&f, 16);
    Action f0[] = {ACT_SKILL};
    ib_step(&f, 0, f0, 1);                 /* locks through frame 8 */
    Action fg[] = {ACT_GUARD};
    Action fl[] = {ACT_LIGHT};
    Action fd[] = {ACT_DASH};
    ib_step(&f, 1, fg, 1);
    ib_step(&f, 2, fl, 1);
    ib_step(&f, 3, fd, 1);
    ib_step(&f, 4, fg, 1);                 /* 4 pending, queue full */
    Action fh[] = {ACT_HEAVY};
    ib_step(&f, 5, fh, 1);                 /* evicts a guard */
    Action fg2[] = {ACT_GUARD};
    ib_step(&f, 6, fg2, 1);                /* too weak, rejected */
    check("overflow_evict_and_reject", f.count == MAX_PENDING &&
          f.evicted_drop == 1 && f.full_drop == 1);
    /* drain: heavy -> dash -> light execute in priority order, the surviving
       guard expires; afterwards the accounting invariant closes exactly. */
    for (int fr = 7; fr <= 22; fr++) ib_step(&f, fr, NULL, 0);
    check("overflow_drain_order", f.count == 0 &&
          f.log[1] == ACT_HEAVY && f.log[2] == ACT_DASH && f.log[3] == ACT_LIGHT &&
          f.expired_drop == 1);
    check("overflow_accounting", accounting_ok(&f));

    /* 7. changing the window only affects commands enqueued afterwards */
    InputBuffer g; ib_init(&g, 4);
    Action g0[] = {ACT_SKILL};             /* locks through frame 8 */
    ib_step(&g, 0, g0, 1);
    Action g1[] = {ACT_HEAVY};             /* expires 5 under old window */
    ib_step(&g, 1, g1, 1);
    ib_set_window(&g, 1);
    Action g2[] = {ACT_LIGHT};             /* expires 3 under new window */
    ib_step(&g, 2, g2, 1);
    ib_step(&g, 3, NULL, 0);
    check("window_change_keeps_old_expiry", g.expired_drop == 0);
    ib_step(&g, 4, NULL, 0);
    check("window_change_new_expiry", g.expired_drop == 1); /* light gone */
    ib_step(&g, 6, NULL, 0);
    check("window_change_old_expiry", g.expired_drop == 2); /* heavy gone */
    Action g8[] = {ACT_GUARD};
    ib_step(&g, 8, g8, 1);                 /* idle: executes immediately */
    check("window_change_executed", g.executed == 2 &&
          g.exec_frame[1] == 8 && accounting_ok(&g));

    /* 8. determinism: identical script and window replay identically */
    InputBuffer r1, r2;
    run_script(4, &r1);
    run_script(4, &r2);
    check("deterministic_replay",
          memcmp(r1.log, r2.log, sizeof(r1.log)) == 0 &&
          memcmp(r1.exec_frame, r2.exec_frame, sizeof(r1.exec_frame)) == 0 &&
          memcmp(r1.log_buffered, r2.log_buffered, sizeof(r1.log_buffered)) == 0 &&
          r1.executed == r2.executed && ib_dropped(&r1) == ib_dropped(&r2));

    /* 9. order preservation: equal-priority actions execute in press order */
    int order_ok = 1;
    for (int i = 1; i < r1.log_count; i++) {
        if (ACTS[r1.log[i]].priority == ACTS[r1.log[i - 1]].priority &&
            r1.exec_frame[i] < r1.exec_frame[i - 1])
            order_ok = 0;
    }
    check("equal_priority_fifo_order", order_ok);

    /* 10. window sweep: same script under different windows, comparable.
       Window 0 is the immediate model; wider windows can only recover inputs,
       never lose more. */
    printf("\nwindow comparison (same script, %d submitted):\n", r1.submitted);
    printf("%-7s %-9s %-9s %-7s %-8s %-8s\n",
           "window", "executed", "expired", "merged", "evicted", "dropped");
    const int windows[] = {0, 1, 2, 4, 8};
    int prev_exec = -1, prev_drop = INT_MAX;
    int submitted_const = 1, monotonic = 1;
    InputBuffer imm; run_script(0, &imm);
    for (int k = 0; k < (int)(sizeof(windows) / sizeof(windows[0])); k++) {
        InputBuffer r; run_script(windows[k], &r);
        printf("%-7d %-9d %-9d %-7d %-8d %-8d\n", windows[k], r.executed,
               r.expired_drop, r.merged_drop, r.evicted_drop, ib_dropped(&r));
        check("accounting_invariant", accounting_ok(&r));
        if (r.submitted != r1.submitted) submitted_const = 0;
        if (r.executed < prev_exec || ib_dropped(&r) > prev_drop) monotonic = 0;
        prev_exec = r.executed; prev_drop = ib_dropped(&r);
        if (windows[k] == 0)
            check("window0_is_immediate_model",
                  r.executed == imm.executed && r.expired_drop == imm.expired_drop &&
                  memcmp(r.log, imm.log, sizeof(r.log)) == 0);
    }
    check("submitted_constant_across_windows", submitted_const);
    check("wider_window_never_worse", monotonic);

    printf("\nall_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
