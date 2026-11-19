
/* tween: keyframe timeline with easing, looping and clamping. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_KEYS 32

typedef enum { EASE_LINEAR = 0, EASE_QUAD_IN, EASE_QUAD_OUT, EASE_CUBIC_IN_OUT, EASE_STEP } Ease;

typedef struct { float t; float v; Ease ease; } Key;

typedef struct {
    Key keys[MAX_KEYS];
    int count;
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

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
