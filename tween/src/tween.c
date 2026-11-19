
/* tween: keyframe timeline with easing, looping and clamping. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_KEYS 32
#define MAX_EVENTS 16

typedef enum { EASE_LINEAR = 0, EASE_QUAD_IN, EASE_QUAD_OUT, EASE_CUBIC_IN_OUT, EASE_STEP } Ease;

typedef struct { float t; float v; Ease ease; } Key;
typedef struct { float t; int id; } Event;

typedef struct {
    Key keys[MAX_KEYS];
    int count;
    Event events[MAX_EVENTS];
    int event_count;
    float duration;
    int loop;
} Timeline;

static void tl_init(Timeline* tl, float duration, int loop) {
    memset(tl, 0, sizeof(*tl));
    tl->duration = duration;
    tl->loop = loop;
}

static int tl_add(Timeline* tl, float t, float v, Ease e) {
    if (tl->count >= MAX_KEYS) return -1;
    if (t < 0.0f || (tl->duration > 0.0f && t > tl->duration)) return -1;
    if (tl->count > 0 && t < tl->keys[tl->count - 1].t) return -1;   /* must be sorted */
    Key* k = &tl->keys[tl->count++];
    k->t = t; k->v = v; k->ease = e;
    return 0;
}

static int tl_add_event(Timeline* tl, float t, int id) {
    if (tl->event_count >= MAX_EVENTS) return -1;
    if (t < 0.0f || (tl->duration > 0.0f && t > tl->duration)) return -1;
    if (tl->event_count > 0 && t < tl->events[tl->event_count - 1].t) return -1;   /* must be sorted */
    Event* e = &tl->events[tl->event_count++];
    e->t = t; e->id = id;
    return 0;
}

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

static float tl_eval(const Timeline* tl, float time) {
    if (tl->count == 0) return 0.0f;
    if (tl->duration > 0.0f) {
        if (tl->loop) {
            time = fmodf(time, tl->duration);
            if (time < 0.0f) time += tl->duration;
        } else {
            if (time < 0.0f) time = 0.0f;
            if (time > tl->duration) time = tl->duration;
        }
    }
    if (time <= tl->keys[0].t) return tl->keys[0].v;
    for (int i = 1; i < tl->count; i++) {
        const Key* a = &tl->keys[i - 1];
        const Key* b = &tl->keys[i];
        if (time <= b->t) {
            float span = b->t - a->t;
            float u = span > 0.0f ? (time - a->t) / span : 1.0f;
            float w = ease_apply(a->ease, u);
            return a->v + (b->v - a->v) * w;
        }
    }
    return tl->keys[tl->count - 1].v;
}

/* ---- time-scaled player: pause / speed / seek with event backfill ----
 *
 * A Player drives a playhead over a Timeline at `speed` x realtime:
 *   speed == 0  paused (playhead frozen, no events fire)
 *   speed  > 0  forward; events fire on the scaled timeline time
 *   speed  < 0  reverse: the playhead runs backward, crossed events fire
 *               in reverse chronological order; a looping timeline wraps
 *               to its end, a non-looping one clamps at 0 and stays there.
 *
 * Events fire when the playhead arrives at or passes their time, never on
 * departure: forward sweeps fire (old, new], backward sweeps fire
 * [new, old). Each event therefore fires exactly once per pass over its
 * timestamp regardless of speed, and pl_seek backfills every event on the
 * path it jumps across. pl_init counts as an arrival at t=0.
 */
typedef struct {
    const Timeline* tl;
    float time;                 /* playhead position, timeline seconds */
    float speed;                /* 1 = realtime, 0 = paused, <0 = reverse */
    void (*on_event)(int id, void* user);
    void* user;
} Player;

static void fire_forward(Player* p, float lo, float hi) {   /* (lo, hi], ascending */
    for (int i = 0; i < p->tl->event_count; i++) {
        const Event* e = &p->tl->events[i];
        if (e->t > lo && e->t <= hi) p->on_event(e->id, p->user);
    }
}

static void fire_backward(Player* p, float lo, float hi) {  /* [lo, hi), descending */
    for (int i = p->tl->event_count - 1; i >= 0; i--) {
        const Event* e = &p->tl->events[i];
        if (e->t >= lo && e->t < hi) p->on_event(e->id, p->user);
    }
}

static void fire_at(Player* p, float t) {                   /* boundary arrival */
    for (int i = 0; i < p->tl->event_count; i++)
        if (p->tl->events[i].t == t) p->on_event(p->tl->events[i].id, p->user);
}

static void pl_noop(int id, void* user) { (void)id; (void)user; }

static void pl_init(Player* p, const Timeline* tl, void (*cb)(int, void*), void* user) {
    p->tl = tl;
    p->time = 0.0f;
    p->speed = 1.0f;
    p->on_event = cb ? cb : pl_noop;
    p->user = user;
    fire_at(p, 0.0f);
}

static void pl_set_speed(Player* p, float speed) { p->speed = speed; }

static void pl_seek(Player* p, float t) {
    float dur = p->tl->duration;
    if (dur > 0.0f) {
        if (t < 0.0f) t = 0.0f;
        if (t > dur) t = dur;
    }
    if (t > p->time) fire_forward(p, p->time, t);
    else if (t < p->time) fire_backward(p, t, p->time);
    p->time = t;
}

static void pl_advance_forward(Player* p, float delta) {
    float dur = p->tl->duration;
    float t = p->time + delta;
    if (p->tl->loop && dur > 0.0f) {
        while (t > dur) {
            fire_forward(p, p->time, dur);
            fire_at(p, 0.0f);           /* arrival at start of next cycle */
            t -= dur;
            p->time = 0.0f;
        }
    } else if (dur > 0.0f && t > dur) {
        t = dur;                        /* clamp at end and stop */
    }
    fire_forward(p, p->time, t);
    p->time = t;
}

static void pl_advance_backward(Player* p, float delta) {
    float dur = p->tl->duration;
    float t = p->time - delta;
    if (p->tl->loop && dur > 0.0f) {
        while (t < 0.0f) {
            fire_backward(p, 0.0f, p->time);
            fire_at(p, dur);            /* arrival at end of previous cycle */
            t += dur;
            p->time = dur;
        }
    } else if (t < 0.0f) {
        t = 0.0f;                       /* clamp at start and stop */
    }
    fire_backward(p, t, p->time);
    p->time = t;
}

static void pl_update(Player* p, float dt) {
    if (dt < 0.0f) dt = 0.0f;           /* negative dt is ignored */
    float delta = dt * p->speed;
    if (delta > 0.0f) pl_advance_forward(p, delta);
    else if (delta < 0.0f) pl_advance_backward(p, -delta);
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int evlog[64];
static int evn;
static void log_cb(int id, void* user) { (void)user; if (evn < 64) evlog[evn++] = id; }

int main(void) {
    Timeline tl;
    tl_init(&tl, 1.0f, 0);
    check("add_first", tl_add(&tl, 0.0f, 0.0f, EASE_LINEAR) == 0);
    check("add_second", tl_add(&tl, 1.0f, 10.0f, EASE_LINEAR) == 0);
    check("clamp_before", fabsf(tl_eval(&tl, -1.0f) - 0.0f) < 1e-6f);
    check("clamp_after", fabsf(tl_eval(&tl, 2.0f) - 10.0f) < 1e-6f);
    check("mid_linear", fabsf(tl_eval(&tl, 0.5f) - 5.0f) < 1e-5f);

    Timeline q;
    tl_init(&q, 1.0f, 0);
    tl_add(&q, 0.0f, 0.0f, EASE_QUAD_IN);
    tl_add(&q, 1.0f, 1.0f, EASE_LINEAR);
    check("quad_in_quarter", fabsf(tl_eval(&q, 0.5f) - 0.25f) < 1e-5f);

    Timeline c;
    tl_init(&c, 1.0f, 0);
    tl_add(&c, 0.0f, 0.0f, EASE_CUBIC_IN_OUT);
    tl_add(&c, 1.0f, 1.0f, EASE_LINEAR);
    check("cubic_mid", fabsf(tl_eval(&c, 0.5f) - 0.5f) < 1e-4f);
    check("cubic_monotonic", tl_eval(&c, 0.4f) < tl_eval(&c, 0.6f));

    Timeline s;
    tl_init(&s, 1.0f, 0);
    tl_add(&s, 0.0f, 0.0f, EASE_STEP);
    tl_add(&s, 1.0f, 1.0f, EASE_LINEAR);
    check("step_holds", fabsf(tl_eval(&s, 0.9f) - 0.0f) < 1e-6f);
    check("step_end", fabsf(tl_eval(&s, 1.0f) - 1.0f) < 1e-6f);

    Timeline l;
    tl_init(&l, 1.0f, 1);
    tl_add(&l, 0.0f, 0.0f, EASE_LINEAR);
    tl_add(&l, 1.0f, 10.0f, EASE_LINEAR);
    check("loop_wrap", fabsf(tl_eval(&l, 1.5f) - 5.0f) < 1e-4f);
    check("loop_negative", fabsf(tl_eval(&l, -0.5f) - 5.0f) < 1e-4f);

    Timeline bad;
    tl_init(&bad, 1.0f, 0);
    check("reject_unsorted", (tl_add(&bad, 0.5f, 1.0f, EASE_LINEAR), tl_add(&bad, 0.2f, 2.0f, EASE_LINEAR)) == -1);
    check("reject_out_of_range", tl_add(&bad, 2.0f, 3.0f, EASE_LINEAR) == -1);

    /* ---- player: pause / speed / seek ---- */
    Timeline cut;
    tl_init(&cut, 2.0f, 0);
    tl_add(&cut, 0.0f, 0.0f, EASE_LINEAR);
    tl_add(&cut, 2.0f, 1.0f, EASE_LINEAR);
    tl_add_event(&cut, 0.5f, 1);
    tl_add_event(&cut, 1.0f, 2);
    tl_add_event(&cut, 1.5f, 3);

    Player p;

    evn = 0; pl_init(&p, &cut, log_cb, NULL);
    pl_set_speed(&p, 0.0f);
    pl_update(&p, 1.0f);
    check("pause_holds_time", fabsf(p.time) < 1e-6f);
    check("pause_fires_nothing", evn == 0);

    evn = 0; pl_init(&p, &cut, log_cb, NULL);
    pl_set_speed(&p, 2.0f);
    pl_update(&p, 0.5f);                    /* scaled time reaches t=1.0 */
    check("speed_2x_event_on_scaled_time", evn == 2 && evlog[0] == 1 && evlog[1] == 2);

    evn = 0; pl_init(&p, &cut, log_cb, NULL);
    pl_set_speed(&p, 3.7f);
    for (int i = 0; i < 40 && p.time < cut.duration; i++) pl_update(&p, 0.13f);
    check("scaled_same_count", evn == 3);
    check("scaled_same_order", evlog[0] == 1 && evlog[1] == 2 && evlog[2] == 3);

    evn = 0; pl_init(&p, &cut, log_cb, NULL);
    pl_seek(&p, 2.0f);
    check("seek_backfills", evn == 3 && evlog[0] == 1 && evlog[1] == 2 && evlog[2] == 3);
    evn = 0;
    pl_seek(&p, 0.75f);
    check("seek_backward_reverse_order", evn == 2 && evlog[0] == 3 && evlog[1] == 2);

    evn = 0; pl_init(&p, &cut, log_cb, NULL);
    pl_seek(&p, 2.0f); evn = 0;
    pl_set_speed(&p, -1.0f);
    pl_update(&p, 0.6f);                    /* time 2.0 -> 1.4, crosses t=1.5 */
    check("reverse_fires_crossed", evn == 1 && evlog[0] == 3);
    pl_update(&p, 5.0f);                    /* clamps at 0, fires 2 then 1 */
    check("reverse_order_and_clamp", evn == 3 && evlog[1] == 2 && evlog[2] == 1 && fabsf(p.time) < 1e-6f);
    pl_update(&p, 1.0f);
    check("reverse_clamp_stays", evn == 3 && fabsf(p.time) < 1e-6f);

    evn = 0; pl_init(&p, &cut, log_cb, NULL);
    pl_update(&p, 10.0f);                   /* one huge step, non-looping */
    check("big_dt_skips_none", evn == 3 && fabsf(p.time - 2.0f) < 1e-6f);
    pl_update(&p, -1.0f);                   /* negative dt ignored */
    check("negative_dt_ignored", fabsf(p.time - 2.0f) < 1e-6f && evn == 3);

    Timeline lp;
    tl_init(&lp, 1.0f, 1);
    tl_add(&lp, 0.0f, 0.0f, EASE_LINEAR);
    tl_add(&lp, 1.0f, 1.0f, EASE_LINEAR);
    tl_add_event(&lp, 0.0f, 7);
    tl_add_event(&lp, 0.5f, 8);

    evn = 0; pl_init(&p, &lp, log_cb, NULL);
    check("init_fires_zero_event", evn == 1 && evlog[0] == 7);
    pl_update(&p, 1.2f);                    /* wraps once */
    check("loop_wrap_each_once", evn == 3 && evlog[1] == 8 && evlog[2] == 7);
    pl_update(&p, 1.0f);                    /* next cycle, still once each */
    check("loop_next_cycle", evn == 5 && evlog[3] == 8 && evlog[4] == 7);
    pl_set_speed(&p, -1.0f);
    evn = 0;
    pl_update(&p, 0.8f);                    /* 0.2 -> backward wraps to 0.4 */
    check("loop_reverse_wrap", evn == 2 && evlog[0] == 7 && evlog[1] == 8 && fabsf(p.time - 0.4f) < 1e-5f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
