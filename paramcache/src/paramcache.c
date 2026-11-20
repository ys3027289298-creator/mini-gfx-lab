
/* paramcache: shader-parameter cache for batched draw calls.
   - per-key direct-indexed lookup (O(1), dedup by parameter key)
   - dirty flags: flush only pushes values that actually changed
   - cached state is reused across batches instead of resetting everything
   - shadow audit compares logical state against backend state to catch
     a missed dirty mark, and a pixel diff against the naive full-reset
     renderer proves the cached path is bit-identical. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define FB_W 24
#define FB_H 24
#define MAIN_BATCHES 48
#define MAIN_NPARAMS 32
#define LINEAR_BATCHES 64

typedef enum { P_FLOAT = 0, P_VEC4 = 1, P_INT = 2 } ParamType;

typedef struct {
    int type;
    union {
        float f;
        float v4[4];
        int i;
    } u;
} ParamValue;

static ParamValue pv_float(float x) {
    ParamValue v; memset(&v, 0, sizeof(v)); v.type = P_FLOAT; v.u.f = x; return v;
}
static ParamValue pv_vec4(const float c[4]) {
    ParamValue v; memset(&v, 0, sizeof(v)); v.type = P_VEC4;
    memcpy(v.u.v4, c, sizeof(v.u.v4)); return v;
}
static ParamValue pv_int(int x) {
    ParamValue v; memset(&v, 0, sizeof(v)); v.type = P_INT; v.u.i = x; return v;
}

static int pv_equal(const ParamValue* a, const ParamValue* b) {
    if (a->type != b->type) return 0;
    if (a->type == P_FLOAT) return memcmp(&a->u.f, &b->u.f, sizeof(float)) == 0;
    if (a->type == P_INT) return a->u.i == b->u.i;
    return memcmp(a->u.v4, b->u.v4, sizeof(a->u.v4)) == 0;
}

/* ---------------- mock GPU backend: uniform slots + framebuffer ---------------- */

typedef struct {
    int nparams;
    ParamValue* slot;       /* values actually held by the "hardware" */
    unsigned char* present;
    long set_calls;
    uint32_t fb[FB_H][FB_W];
} Backend;

static void backend_init(Backend* b, int nparams) {
    memset(b, 0, sizeof(*b));
    b->nparams = nparams;
    b->slot = calloc((size_t)nparams, sizeof(ParamValue));
    b->present = calloc((size_t)nparams, 1);
}
static void backend_free(Backend* b) {
    free(b->slot); free(b->present);
    b->slot = NULL; b->present = NULL;
}
static void backend_set(Backend* b, int key, ParamValue v) {
    b->slot[key] = v;
    b->present[key] = 1;
    b->set_calls++;
}

/* ---------------- parameter cache ---------------- */

typedef struct {
    int nparams;
    ParamValue* desired;    /* logical state staged by the scene script */
    ParamValue* flushed;    /* last value actually pushed to the backend */
    unsigned char* known;   /* desired[key] is valid */
    unsigned char* pushed;  /* flushed[key] is valid in the backend */
    unsigned char* dirty;
    long scan_ops;          /* keys inspected while flushing */
    long dedup_hits;        /* cache_set calls that changed nothing */
} ParamCache;

static int cache_init(ParamCache* c, int nparams) {
    memset(c, 0, sizeof(*c));
    c->nparams = nparams;
    c->desired = calloc((size_t)nparams, sizeof(ParamValue));
    c->flushed = calloc((size_t)nparams, sizeof(ParamValue));
    c->known = calloc((size_t)nparams, 1);
    c->pushed = calloc((size_t)nparams, 1);
    c->dirty = calloc((size_t)nparams, 1);
    return c->desired && c->flushed && c->known && c->pushed && c->dirty ? 0 : -1;
}
static void cache_free(ParamCache* c) {
    free(c->desired); free(c->flushed);
    free(c->known); free(c->pushed); free(c->dirty);
    memset(c, 0, sizeof(*c));
}

/* Stage a value. Direct-indexed O(1); a write that needs no backend call
   is deduplicated: either it re-stages the pending desired value whose
   change is already marked, or it matches the value the backend already
   holds (revert / missed-dirty repair). If desired holds the value but it
   is neither marked dirty nor present in the backend, a missed dirty mark
   has occurred and re-staging must raise the flag. */
static void cache_set(ParamCache* c, int key, ParamValue v) {
    if (key < 0 || key >= c->nparams) return;
    if (c->known[key] && pv_equal(&c->desired[key], &v)) {
        int in_flight = c->dirty[key] ||
                        (c->pushed[key] && pv_equal(&c->flushed[key], &v));
        if (in_flight) { c->dedup_hits++; return; }
        /* missed dirty mark: fall through and raise the flag */
    } else if (c->pushed[key] && pv_equal(&c->flushed[key], &v)) {
        c->dedup_hits++;
        c->desired[key] = v;
        c->known[key] = 1;
        c->dirty[key] = 0;
        return;
    }
    c->desired[key] = v;
    c->known[key] = 1;
    c->dirty[key] = 1;
}

/* Push dirty keys to the backend. One linear scan over the key space;
   the value re-check makes spurious dirty marks cost no backend call. */
static void cache_flush(ParamCache* c, Backend* b) {
    for (int k = 0; k < c->nparams; k++) {
        c->scan_ops++;
        if (!c->dirty[k]) continue;
        c->dirty[k] = 0;
        if (c->pushed[k] && pv_equal(&c->desired[k], &c->flushed[k])) continue;
        backend_set(b, k, c->desired[k]);
        c->flushed[k] = c->desired[k];
        c->pushed[k] = 1;
    }
}

/* Shadow audit: every pushed key must match between logical state and
   backend reality. A missed dirty mark leaves a divergence here. */
static int cache_audit(const ParamCache* c, const Backend* b) {
    int bad = 0;
    for (int k = 0; k < c->nparams; k++) {
        if (!c->pushed[k]) continue;
        if (!b->present[k] || !pv_equal(&c->desired[k], &b->slot[k])) bad++;
    }
    return bad;
}

/* Fault injection: change logical state without raising the dirty flag. */
static void inject_missed_dirty(ParamCache* c, int key, ParamValue v) {
    c->desired[key] = v;
    c->known[key] = 1;
    /* dirty[key] deliberately left clear */
}

/* ---------------- deterministic scene script ---------------- */

static uint32_t lcg32(uint32_t* s) {
    *s = *s * 1664525u + 1013904223u;
    return *s;
}

static ParamType type_of_key(int key) {
    if (key == 0) return P_VEC4;       /* tint */
    if (key == 1) return P_FLOAT;      /* opacity */
    if (key == 2) return P_INT;        /* tone */
    return (ParamType)(key % 3);
}

static ParamValue scene_value(int key, int batch) {
    uint32_t s = (uint32_t)(key * 7919 + batch * 104729 + 1);
    switch (type_of_key(key)) {
        case P_FLOAT: return pv_float((int)(lcg32(&s) % 2000) / 1000.0f - 1.0f);
        case P_VEC4: {
            float c[4];
            for (int i = 0; i < 4; i++) c[i] = (lcg32(&s) % 1000) / 1000.0f;
            return pv_vec4(c);
        }
        default: return pv_int((int)(lcg32(&s) % 64) - 32);
    }
}

/* ~25% of keys change per batch; batch 0 sets everything. */
static int key_changes(int key, int batch) {
    if (batch == 0) return 1;
    uint32_t s = (uint32_t)(key * 131 + batch * 17 + 7);
    return lcg32(&s) % 4 == 0;
}

/* Scene model: a parameter keeps its value across batches until a change
   event fires. Returns the value the scene currently holds for the key. */
static ParamValue effective_value(int key, int batch) {
    for (int t = batch; t >= 0; t--)
        if (key_changes(key, t)) return scene_value(key, t);
    return scene_value(key, 0);   /* unreachable: batch 0 changes every key */
}

/* ---------------- software shading (identical for both paths) ---------------- */

static uint32_t value_bits(const ParamValue* v) {
    uint32_t bits = (uint32_t)v->type;
    if (v->type == P_FLOAT) {
        uint32_t b; memcpy(&b, &v->u.f, sizeof(b));
        bits ^= b * 2654435761u;
    } else if (v->type == P_INT) {
        bits ^= (uint32_t)v->u.i * 2246822519u;
    } else {
        for (int i = 0; i < 4; i++) {
            uint32_t b; memcpy(&b, &v->u.v4[i], sizeof(b));
            bits ^= b + 0x9e3779b9u + (bits << 6) + (bits >> 2);
        }
    }
    return bits;
}

static unsigned clampb(long x) {
    if (x < 0) return 0;
    if (x > 255) return 255;
    return (unsigned)x;
}

/* Every backend slot feeds the pixel checksum, so any stale parameter
   (including a missed dirty mark) changes the output. */
static void draw_frame(Backend* b, int batch) {
    for (int y = 0; y < FB_H; y++) {
        for (int x = 0; x < FB_W; x++) {
            uint32_t s = (uint32_t)(batch * 100003u + x * 389u + y * 17u + 1u);
            uint32_t acc = lcg32(&s);
            for (int k = 0; k < b->nparams; k++) {
                if (!b->present[k]) continue;
                acc ^= value_bits(&b->slot[k]);
                acc = lcg32(&acc);
            }
            const ParamValue* tint = &b->slot[0];
            const ParamValue* opv = &b->slot[1];
            const ParamValue* tone = &b->slot[2];
            float op = opv->u.f;
            if (op < 0.0f) op = 0.0f; else if (op > 1.0f) op = 1.0f;
            long jitter = (long)(acc % 19) - 9;
            unsigned r = clampb(lroundf(tint->u.v4[0] * 255.0f * op) + jitter + tone->u.i);
            unsigned g = clampb(lroundf(tint->u.v4[1] * 255.0f * op) + jitter + tone->u.i);
            unsigned bl = clampb(lroundf(tint->u.v4[2] * 255.0f * op) + jitter);
            unsigned a = clampb(lroundf(tint->u.v4[3] * 255.0f));
            b->fb[y][x] = (r << 24) | (g << 16) | (bl << 8) | a;
        }
    }
}

/* ---------------- test harness ---------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* Naive reference: reset every parameter on every batch even though most
   values are unchanged. The set call count is batches * nparams. */
static long run_naive(Backend* b, int nparams, int batches) {
    for (int t = 0; t < batches; t++) {
        for (int k = 0; k < nparams; k++) backend_set(b, k, effective_value(k, t));
        draw_frame(b, t);
    }
    return b->set_calls;
}

/* Cached path: stage only changed keys, flush dirty ones, reuse the rest. */
static long run_cached(ParamCache* c, Backend* b, int nparams, int batches,
                       int* audit_errors, int* change_events,
                       int* batch3_backend_sets) {
    int events = 0;
    *audit_errors = 0;
    if (batch3_backend_sets) *batch3_backend_sets = -1;
    for (int t = 0; t < batches; t++) {
        for (int k = 0; k < nparams; k++) {
            if (!key_changes(k, t)) continue;
            events++;
            ParamValue v = scene_value(k, t);
            cache_set(c, k, v);
            /* batch 3, key 1 (a key that changes here): exercise
               within-batch dedup and A->B->A churn */
            if (t == 3 && k == 1) {
                ParamValue old = effective_value(1, 2);
                cache_set(c, 1, v);            /* same value: dedup */
                cache_set(c, 1, v);            /* same value: dedup */
                cache_set(c, 1, old);          /* back to old value */
                cache_set(c, 1, v);            /* and forward again */
            }
        }
        long before = b->set_calls;
        cache_flush(c, b);
        if (t == 3 && batch3_backend_sets)
            *batch3_backend_sets = (int)(b->set_calls - before);
        *audit_errors += cache_audit(c, b);
        draw_frame(b, t);
    }
    if (change_events) *change_events = events;
    return b->set_calls;
}

static void test_pixel_identical(void) {
    Backend ref; backend_init(&ref, MAIN_NPARAMS);
    Backend hot; backend_init(&hot, MAIN_NPARAMS);
    ParamCache c; cache_init(&c, MAIN_NPARAMS);

    long naive_calls = run_naive(&ref, MAIN_NPARAMS, MAIN_BATCHES);
    int audit_errors = 0, events = 0, batch3_sets = 0;
    long cached_calls = run_cached(&c, &hot, MAIN_NPARAMS, MAIN_BATCHES,
                                   &audit_errors, &events, &batch3_sets);

    int changed_batch3 = 0;
    for (int k = 0; k < MAIN_NPARAMS; k++)
        if (key_changes(k, 3)) changed_batch3++;

    check("audit_clean", audit_errors == 0);
    check("pixels_identical",
          memcmp(ref.fb, hot.fb, sizeof(ref.fb)) == 0);
    check("calls_match_change_events", cached_calls == events);
    check("batch3_coalesced", batch3_sets == changed_batch3);
    check("dedup_recorded", c.dedup_hits >= 2);

    printf("batches=%d nparams=%d\n", MAIN_BATCHES, MAIN_NPARAMS);
    printf("naive_set_calls=%ld\n", naive_calls);
    printf("cached_set_calls=%ld\n", cached_calls);
    printf("calls_saved=%ld\n", naive_calls - cached_calls);
    printf("calls_reduction_pct=%.1f\n",
           100.0 * (double)(naive_calls - cached_calls) / (double)naive_calls);
    printf("dedup_hits=%ld scan_ops=%ld scan_per_batch=%.1f\n",
           c.dedup_hits, c.scan_ops, (double)c.scan_ops / MAIN_BATCHES);

    cache_free(&c);
    backend_free(&ref);
    backend_free(&hot);
}

static void test_missed_dirty_detection(void) {
    enum { N = 8 };
    ParamCache c; cache_init(&c, N);
    Backend buggy; backend_init(&buggy, N);
    Backend good; backend_init(&good, N);

    /* batch 0 baseline: every key staged through the normal path */
    for (int k = 0; k < N; k++) cache_set(&c, k, effective_value(k, 0));
    cache_flush(&c, &buggy);
    check("audit_clean_before_fault", cache_audit(&c, &buggy) == 0);

    /* batch 1: keys 3 and 7 are the ones that change; key 7 goes through
       the normal path, key 3 gets a missed dirty mark */
    cache_set(&c, 7, effective_value(7, 1));
    inject_missed_dirty(&c, 3, effective_value(3, 1));
    cache_flush(&c, &buggy);

    int diverged = cache_audit(&c, &buggy);
    check("missed_dirty_audit_detected", diverged == 1);

    /* pixel-level detection: a correct backend holding the full logical
       state of batch 1 renders a different frame */
    for (int k = 0; k < N; k++) backend_set(&good, k, effective_value(k, 1));
    draw_frame(&buggy, 1);
    draw_frame(&good, 1);
    check("missed_dirty_pixel_detected",
          memcmp(buggy.fb, good.fb, sizeof(buggy.fb)) != 0);

    /* proper staging repairs the logical/backend divergence */
    cache_set(&c, 3, effective_value(3, 1));
    cache_flush(&c, &buggy);
    check("audit_clean_after_repair", cache_audit(&c, &buggy) == 0);

    /* after repair the repaired backend matches a freshly rebuilt one */
    Backend fixed; backend_init(&fixed, N);
    for (int k = 0; k < N; k++) backend_set(&fixed, k, effective_value(k, 1));
    draw_frame(&buggy, 2);
    draw_frame(&fixed, 2);
    check("pixels_match_after_repair",
          memcmp(buggy.fb, fixed.fb, sizeof(buggy.fb)) == 0);

    cache_free(&c);
    backend_free(&buggy); backend_free(&good); backend_free(&fixed);
}

static void test_linear_scaling(void) {
    const int sizes[] = {16, 64, 256, 1024};
    long prev_calls = 0;
    double prev_ratio = 0.0;
    int linear_ok = 1;
    printf("%8s %12s %12s %14s %14s\n",
           "nparams", "scan_ops", "set_calls", "ops_per_param", "calls_per_param");
    for (size_t i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
        int n = sizes[i];
        ParamCache c; cache_init(&c, n);
        Backend b; backend_init(&b, n);
        int events = 0, audit_errors = 0;
        run_cached(&c, &b, n, LINEAR_BATCHES, &audit_errors, &events, NULL);
        if (audit_errors != 0) linear_ok = 0;
        if (c.scan_ops != (long)LINEAR_BATCHES * n) linear_ok = 0;
        double ops_per = (double)c.scan_ops / n;
        double calls_per = (double)b.set_calls / n;
        printf("%8d %12ld %12ld %14.2f %14.2f\n",
               n, c.scan_ops, b.set_calls, ops_per, calls_per);
        if (i > 0) {
            /* quadrupling the parameter kinds must ~quadruple the work:
               per-kind work stays constant while totals track n linearly */
            double drift = calls_per - prev_ratio;
            if (drift < 0) drift = -drift;
            if (drift > 2.0) linear_ok = 0;
            if (b.set_calls < 3 * prev_calls || b.set_calls > 5 * prev_calls)
                linear_ok = 0;
        }
        prev_calls = b.set_calls;
        prev_ratio = calls_per;
        cache_free(&c);
        backend_free(&b);
    }
    check("linear_in_param_kinds", linear_ok);
}

int main(void) {
    test_pixel_identical();
    test_missed_dirty_detection();
    test_linear_scaling();
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
