
/* tween: keyframe timeline with easing, looping and clamping. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_KEYS 32
#define MAX_EVENTS 32

typedef enum { EASE_LINEAR = 0, EASE_QUAD_IN, EASE_QUAD_OUT, EASE_CUBIC_IN_OUT, EASE_STEP } Ease;

typedef struct { float t; float v; Ease ease; } Key;

typedef struct { float t; int id; } TlEvent;

typedef struct {
    Key keys[MAX_KEYS];
    int count;
    TlEvent events[MAX_EVENTS];
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
    TlEvent* e = &tl->events[tl->event_count++];
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

/* Cutscene player: forward / reverse playback with optional horizontal mirror.
 * The cursor is a timeline position and dir only sets the travel direction,
 * so reverse passes through every intermediate state of the forward play,
 * frame-by-frame. Events fire once each per
 * playthrough, in reverse order when playing backwards. The cursor clamps at
 * both ends and stops there -- reverse never loops back to the end. */
typedef struct {
    const Timeline* tl;
    float time;          /* cursor = position on the timeline, [0, duration] */
    int dir;             /* +1 forward (0 -> D), -1 reverse (D -> 0) */
    int mirror;          /* nonzero: reflect values around mirror_center */
    float mirror_center;
    int done;            /* reached the end of the current direction */
    int ev;              /* next event index to fire (forward: ascending, reverse: descending) */
} Player;

static void player_init(Player* p, const Timeline* tl, int dir, int mirror, float mirror_center,
                        int* fired, int max_fired, int* fired_count) {
    p->tl = tl;
    p->dir = dir >= 0 ? 1 : -1;
    p->mirror = mirror;
    p->mirror_center = mirror_center;
    p->done = 0;
    int n = 0;
    if (p->dir > 0) {
        p->time = 0.0f;
        p->ev = 0;
        /* events sitting exactly at the start fire immediately */
        while (p->ev < tl->event_count && tl->events[p->ev].t <= 0.0f) {
            if (fired && n < max_fired) fired[n] = tl->events[p->ev].id;
            n++;
            p->ev++;
        }
    } else {
        p->time = tl->duration;
        p->ev = tl->event_count - 1;
        while (p->ev >= 0 && tl->events[p->ev].t >= tl->duration) {
            if (fired && n < max_fired) fired[n] = tl->events[p->ev].id;
            n++;
            p->ev--;
        }
    }
    if (fired_count) *fired_count = n;
}

/* Current value: the cursor IS the timeline position, so forward and reverse
 * passing the same position yield the same state (frame-by-frame match).
 * Mirror reflects the sampled value around the center line. */
static float player_value(const Player* p) {
    Timeline tmp = *p->tl;  /* cutscenes clamp, never loop */
    tmp.loop = 0;
    float v = tl_eval(&tmp, p->time);
    return p->mirror ? 2.0f * p->mirror_center - v : v;
}

/* Advance by dt seconds of wall time. Fired event ids are appended to fired[]
 * (in trigger order); returns how many fired this step. */
static int player_step(Player* p, float dt, int* fired, int max_fired) {
    const Timeline* tl = p->tl;
    int n = 0;
    if (p->done || dt <= 0.0f) return 0;
    p->time += dt * (float)p->dir;
    if (p->time <= 0.0f) { p->time = 0.0f; if (p->dir < 0) p->done = 1; }
    if (p->time >= tl->duration) { p->time = tl->duration; if (p->dir > 0) p->done = 1; }
    if (p->dir > 0) {
        while (p->ev < tl->event_count && tl->events[p->ev].t <= p->time) {
            if (fired && n < max_fired) fired[n] = tl->events[p->ev].id;
            n++;
            p->ev++;
        }
    } else {
        while (p->ev >= 0 && tl->events[p->ev].t >= p->time) {
            if (fired && n < max_fired) fired[n] = tl->events[p->ev].id;
            n++;
            p->ev--;
        }
    }
    return n;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

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

    /* cutscene player: reverse + mirror */
    Timeline cs;
    tl_init(&cs, 1.0f, 0);
    tl_add(&cs, 0.0f, 0.0f, EASE_LINEAR);
    tl_add(&cs, 1.0f, 10.0f, EASE_LINEAR);
    tl_add_event(&cs, 0.0f, 1);
    tl_add_event(&cs, 0.4f, 2);
    tl_add_event(&cs, 0.8f, 3);
    tl_add_event(&cs, 1.0f, 4);
    check("reject_unsorted_event", tl_add_event(&cs, 0.5f, 9) == -1);

    /* forward: events fire in order, one full playthrough fires each once */
    Player pf;
    int fired[8], nf = 0, n;
    player_init(&pf, &cs, 1, 0, 0.0f, fired, 8, &nf);
    n = player_step(&pf, 0.5f, fired + nf, 8 - nf); nf += n;
    n = player_step(&pf, 0.5f, fired + nf, 8 - nf); nf += n;
    check("fwd_done", pf.done == 1);
    check("fwd_event_count", nf == 4);
    check("fwd_event_order", fired[0] == 1 && fired[1] == 2 && fired[2] == 3 && fired[3] == 4);

    /* reverse: same events, reversed order, same count */
    Player pr;
    int rfired[8], rn = 0;
    player_init(&pr, &cs, -1, 0, 0.0f, rfired, 8, &rn);
    n = player_step(&pr, 0.5f, rfired + rn, 8 - rn); rn += n;
    n = player_step(&pr, 0.5f, rfired + rn, 8 - rn); rn += n;
    check("rev_done", pr.done == 1);
    check("rev_event_count_same", rn == nf);
    check("rev_event_order", rfired[0] == 4 && rfired[1] == 3 && rfired[2] == 2 && rfired[3] == 1);

    /* reverse stops at the start: no wrap, no refire, stays done */
    n = player_step(&pr, 0.5f, rfired, 8);
    check("rev_stops_at_start", pr.done == 1 && pr.time == 0.0f && n == 0);
    check("rev_value_holds_start", fabsf(player_value(&pr) - 0.0f) < 1e-6f);

    /* frame-by-frame correspondence: same timeline position -> same state */
    int states_match = 1;
    for (int i = 0; i <= 10; i++) {
        float wall = 0.1f * (float)i;
        Player a, b;
        player_init(&a, &cs, 1, 0, 0.0f, 0, 0, 0);
        player_init(&b, &cs, -1, 0, 0.0f, 0, 0, 0);
        player_step(&a, wall, 0, 0);            /* forward at wall time t   */
        player_step(&b, 1.0f - wall, 0, 0);     /* reverse at wall time D-t */
        if (fabsf(player_value(&a) - player_value(&b)) > 1e-4f) states_match = 0;
    }
    check("rev_matches_forward_frames", states_match);

    /* mirror: displacement symmetric around the center line */
    Player pm;
    player_init(&pm, &cs, 1, 1, 5.0f, 0, 0, 0);
    player_step(&pm, 0.3f, 0, 0);
    check("mirror_symmetric", fabsf(player_value(&pm) - (2.0f * 5.0f - 3.0f)) < 1e-4f);

    /* mirror + reverse compose: mirrored reverse still matches mirrored forward */
    Player mr;
    player_init(&mr, &cs, -1, 1, 5.0f, 0, 0, 0);
    player_step(&mr, 0.7f, 0, 0);               /* lands on forward t = 0.3 */
    check("mirror_reverse_compose", fabsf(player_value(&mr) - (10.0f - 3.0f)) < 1e-4f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
