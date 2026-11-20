
/* ringbuf: single-producer/single-consumer lock-free ring buffer for crossing
   the audio/main thread boundary.

   - power-of-two capacity, byte-granular element size
   - monotonic 64-bit head/tail indices (wrap only on ring position, never on
     the counters themselves), so fill = head - tail needs no modular reasoning
   - head is written only by the producer, tail only by the consumer; the two
     live on separate cache lines to avoid false sharing
   - data is published with release and observed with acquire; no locks, no
     copies of the shared indices other than the one acquire load per op
   - underrun (short read) and overrun (rejected write) are counted per call
     and per sample on the thread that owns the side

   A mutex-locked reference implementation with byte-identical semantics is
   included below and driven through the same schedules in the tests so that
   sample-for-sample equivalence can be checked. */
#define _GNU_SOURCE
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================== lock-free SPSC ring buffer ===================== */

typedef struct {
    unsigned char* buf;     /* cap * elem_size, 64-byte aligned */
    uint32_t cap;           /* power of two, element count */
    uint32_t mask;          /* cap - 1 */
    uint32_t elem_size;

    /* producer-owned cache line: only the writer updates head + overrun */
    _Alignas(64) _Atomic uint64_t head;
    _Atomic uint64_t overrun_events;    /* write calls rejecting >= 1 sample */
    _Atomic uint64_t overrun_samples;   /* total samples rejected */

    /* consumer-owned cache line: only the reader updates tail + underrun */
    _Alignas(64) _Atomic uint64_t tail;
    _Atomic uint64_t underrun_events;   /* read calls short by >= 1 sample */
    _Atomic uint64_t underrun_samples;  /* total samples missing */
} RingBuf;

static int rb_init(RingBuf* rb, uint32_t elem_size, uint32_t capacity) {
    if (rb == NULL || elem_size == 0 || capacity == 0) return -1;
    uint32_t cap = 1;
    while (cap < capacity) {
        if (cap & 0x80000000u) return -1;   /* rounding overflow */
        cap <<= 1;
    }
    if ((size_t)cap * (size_t)elem_size / (size_t)elem_size != (size_t)cap)
        return -1;                          /* byte size overflow */
    void* mem = NULL;
    if (posix_memalign(&mem, 64, (size_t)cap * (size_t)elem_size) != 0)
        return -1;
    rb->buf = (unsigned char*)mem;
    rb->cap = cap;
    rb->mask = cap - 1;
    rb->elem_size = elem_size;
    atomic_init(&rb->head, 0);
    atomic_init(&rb->tail, 0);
    atomic_init(&rb->overrun_events, 0);
    atomic_init(&rb->overrun_samples, 0);
    atomic_init(&rb->underrun_events, 0);
    atomic_init(&rb->underrun_samples, 0);
    return 0;
}

static void rb_destroy(RingBuf* rb) {
    free(rb->buf);
    rb->buf = NULL;
}

static void ring_copy_in(unsigned char* buf, uint32_t cap, uint32_t mask,
                         uint32_t esize, uint64_t pos,
                         const void* src, uint32_t n) {
    uint32_t i = (uint32_t)(pos & mask);
    uint32_t first = cap - i;               /* slots until end of ring */
    if (first > n) first = n;
    memcpy(buf + (size_t)i * esize, src, (size_t)first * esize);
    if (n > first)
        memcpy(buf, (const unsigned char*)src + (size_t)first * esize,
               (size_t)(n - first) * esize);
}

static void ring_copy_out(unsigned char* buf, uint32_t cap, uint32_t mask,
                          uint32_t esize, uint64_t pos,
                          void* dst, uint32_t n) {
    uint32_t i = (uint32_t)(pos & mask);
    uint32_t first = cap - i;
    if (first > n) first = n;
    memcpy(dst, buf + (size_t)i * esize, (size_t)first * esize);
    if (n > first)
        memcpy((unsigned char*)dst + (size_t)first * esize, buf,
               (size_t)(n - first) * esize);
}

/* Enqueue up to n elements. Returns the number accepted; samples that did not
   fit are rejected (never overwrite unread data) and counted as one overrun
   event plus the rejected sample count. */
static uint32_t rb_write(RingBuf* rb, const void* src, uint32_t n) {
    if (n == 0) return 0;
    uint64_t tail = atomic_load_explicit(&rb->tail, memory_order_acquire);
    uint64_t head = atomic_load_explicit(&rb->head, memory_order_relaxed);
    uint32_t fill = (uint32_t)(head - tail);
    uint32_t room = rb->cap - fill;
    uint32_t take = (n < room) ? n : room;
    if (take < n) {
        atomic_fetch_add_explicit(&rb->overrun_events, 1, memory_order_relaxed);
        atomic_fetch_add_explicit(&rb->overrun_samples, (uint64_t)(n - take),
                                  memory_order_relaxed);
    }
    if (take > 0) {
        ring_copy_in(rb->buf, rb->cap, rb->mask, rb->elem_size, head, src, take);
        atomic_store_explicit(&rb->head, head + take, memory_order_release);
    }
    return take;
}

/* Dequeue up to n elements. Returns the number delivered; a short read is
   counted as one underrun event plus the missing sample count, and the dst
   slots for missing samples are left untouched. */
static uint32_t rb_read(RingBuf* rb, void* dst, uint32_t n) {
    if (n == 0) return 0;
    uint64_t head = atomic_load_explicit(&rb->head, memory_order_acquire);
    uint64_t tail = atomic_load_explicit(&rb->tail, memory_order_relaxed);
    uint32_t avail = (uint32_t)(head - tail);
    uint32_t take = (n < avail) ? n : avail;
    if (take < n) {
        atomic_fetch_add_explicit(&rb->underrun_events, 1, memory_order_relaxed);
        atomic_fetch_add_explicit(&rb->underrun_samples, (uint64_t)(n - take),
                                  memory_order_relaxed);
    }
    if (take > 0) {
        ring_copy_out(rb->buf, rb->cap, rb->mask, rb->elem_size, tail, dst, take);
        atomic_store_explicit(&rb->tail, tail + take, memory_order_release);
    }
    return take;
}

static uint32_t rb_fill(const RingBuf* rb) {
    uint64_t head = atomic_load_explicit(&rb->head, memory_order_acquire);
    uint64_t tail = atomic_load_explicit(&rb->tail, memory_order_acquire);
    return (uint32_t)(head - tail);
}

static uint64_t rb_ctr_load(const _Atomic uint64_t* p) {
    return atomic_load_explicit((_Atomic uint64_t*)p, memory_order_relaxed);
}

/* ===================== mutex-locked reference ===================== */

/* Same call semantics and the same counter rules, guarded by one mutex. Used
   as the oracle for sample-exact comparison under identical schedules. */
typedef struct {
    pthread_mutex_t mu;
    unsigned char* buf;
    uint32_t cap, mask, elem_size;
    uint64_t head, tail;
    uint64_t overrun_events, overrun_samples;
    uint64_t underrun_events, underrun_samples;
} LockBuf;

static int lb_init(LockBuf* lb, uint32_t elem_size, uint32_t capacity) {
    if (lb == NULL || elem_size == 0 || capacity == 0) return -1;
    uint32_t cap = 1;
    while (cap < capacity) {
        if (cap & 0x80000000u) return -1;
        cap <<= 1;
    }
    if (pthread_mutex_init(&lb->mu, NULL) != 0) return -1;
    lb->buf = (unsigned char*)malloc((size_t)cap * (size_t)elem_size);
    if (lb->buf == NULL) {
        pthread_mutex_destroy(&lb->mu);
        return -1;
    }
    lb->cap = cap;
    lb->mask = cap - 1;
    lb->elem_size = elem_size;
    lb->head = lb->tail = 0;
    lb->overrun_events = lb->overrun_samples = 0;
    lb->underrun_events = lb->underrun_samples = 0;
    return 0;
}

static void lb_destroy(LockBuf* lb) {
    pthread_mutex_destroy(&lb->mu);
    free(lb->buf);
    lb->buf = NULL;
}

static uint32_t lb_write(LockBuf* lb, const void* src, uint32_t n) {
    if (n == 0) return 0;
    pthread_mutex_lock(&lb->mu);
    uint32_t fill = (uint32_t)(lb->head - lb->tail);
    uint32_t room = lb->cap - fill;
    uint32_t take = (n < room) ? n : room;
    if (take < n) {
        lb->overrun_events++;
        lb->overrun_samples += n - take;
    }
    if (take > 0)
        ring_copy_in(lb->buf, lb->cap, lb->mask, lb->elem_size,
                     lb->head, src, take);
    lb->head += take;
    pthread_mutex_unlock(&lb->mu);
    return take;
}

static uint32_t lb_read(LockBuf* lb, void* dst, uint32_t n) {
    if (n == 0) return 0;
    pthread_mutex_lock(&lb->mu);
    uint32_t avail = (uint32_t)(lb->head - lb->tail);
    uint32_t take = (n < avail) ? n : avail;
    if (take < n) {
        lb->underrun_events++;
        lb->underrun_samples += n - take;
    }
    if (take > 0)
        ring_copy_out(lb->buf, lb->cap, lb->mask, lb->elem_size,
                      lb->tail, dst, take);
    lb->tail += take;
    pthread_mutex_unlock(&lb->mu);
    return take;
}

static uint32_t lb_fill(LockBuf* lb) {
    pthread_mutex_lock(&lb->mu);
    uint32_t f = (uint32_t)(lb->head - lb->tail);
    pthread_mutex_unlock(&lb->mu);
    return f;
}

/* ============================ test scaffolding ============================ */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static uint64_t rng_state = 0x123456789abcdef0ULL;
static uint32_t rng_next(void) {
    rng_state = rng_state * 6364136223846793005ULL + 1442695040888963407ULL;
    return (uint32_t)(rng_state >> 33);
}

/* deterministic per-offer value: both producers in the concurrent test derive
   the exact same stream without sharing a generated array */
static uint32_t value_at(uint64_t k) {
    uint64_t z = k + 0x9e3779b97f4a7c15ULL;
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return (uint32_t)(z ^ (z >> 31));
}

/* ------------------------- basic single-thread tests ------------------------- */

static void test_basic(void) {
    RingBuf rb;
    check("init_rounds_pow2", rb_init(&rb, sizeof(int32_t), 3) == 0 && rb.cap == 4);
    check("init_rejects_bad_args",
          rb_init(&rb, 0, 8) == -1 && rb_init(&rb, 4, 0) == -1);

    int32_t in[8], out[8];
    for (int i = 0; i < 8; i++) in[i] = 100 + i;

    check("empty_read_underrun", rb_read(&rb, out, 2) == 0);
    check("underrun_event_counted",
          rb_ctr_load(&rb.underrun_events) == 1 &&
          rb_ctr_load(&rb.underrun_samples) == 2);

    check("write_four_of_four", rb_write(&rb, in, 4) == 4);
    check("fill_full", rb_fill(&rb) == 4);
    check("full_write_rejected", rb_write(&rb, in + 4, 2) == 0);
    check("overrun_counted",
          rb_ctr_load(&rb.overrun_events) == 1 &&
          rb_ctr_load(&rb.overrun_samples) == 2);

    check("read_four", rb_read(&rb, out, 4) == 4);
    int order_ok = 1;
    for (int i = 0; i < 4; i++) order_ok &= (out[i] == 100 + i);
    check("fifo_order", order_ok);
    check("drained", rb_fill(&rb) == 0);

    /* zero-length calls must not touch the counters */
    check("zero_ops_are_free",
          rb_write(&rb, in, 0) == 0 && rb_read(&rb, out, 0) == 0 &&
          rb_ctr_load(&rb.overrun_events) == 1 &&
          rb_ctr_load(&rb.underrun_events) == 1);
    rb_destroy(&rb);
}

static void test_wraparound(void) {
    RingBuf rb;
    rb_init(&rb, sizeof(int32_t), 4);
    int32_t in[16], out[16];
    uint64_t offer = 0, delivered = 0;
    int ok = 1;
    /* many cycles force the split-memcpy path at every ring position */
    for (int cycle = 0; cycle < 5000; cycle++) {
        uint32_t wn = 1 + rng_next() % 6;           /* up to 6 into cap 4 */
        for (uint32_t j = 0; j < wn; j++) in[j] = (int32_t)value_at(offer + j);
        uint32_t accepted = rb_write(&rb, in, wn);
        offer += accepted;      /* rejected samples are dropped, not retried */
        uint32_t rn = 1 + rng_next() % 6;
        uint32_t rd = rb_read(&rb, out, rn);
        /* global rank of the j-th delivered sample = delivered-before + j */
        uint64_t base = delivered;
        for (uint32_t j = 0; j < rd; j++) {
            if (out[j] != (int32_t)value_at(base + j)) ok = 0;
            delivered++;
        }
    }
    /* drain whatever remains */
    uint32_t rd;
    while ((rd = rb_read(&rb, out, 16)) > 0) {
        uint64_t base = delivered;
        for (uint32_t j = 0; j < rd; j++) {
            if (out[j] != (int32_t)value_at(base + j)) ok = 0;
            delivered++;
        }
    }
    /* every accepted sample was delivered, in order */
    check("wrap_sequence_exact", ok && delivered == offer);
    check("wrap_ends_empty", rb_fill(&rb) == 0);
    rb_destroy(&rb);
}

static void test_float_elements(void) {
    RingBuf rb;
    rb_init(&rb, sizeof(float), 8);
    float fin[6] = { 0.25f, -0.5f, 1.0f, 0.125f, 0.75f, -0.0625f };
    float fout[6] = { 0 };
    check("float_write", rb_write(&rb, fin, 6) == 6);
    check("float_read", rb_read(&rb, fout, 6) == 6);
    int ok = 1;
    for (int i = 0; i < 6; i++) ok &= (fout[i] == fin[i]);
    check("float_bits_preserved", ok);
    rb_destroy(&rb);
}

/* ------------- single-thread fuzz: lock-free vs locked oracle ------------- */

static void test_fuzz_equivalence(void) {
    RingBuf rb;
    LockBuf lb;
    rb_init(&rb, sizeof(int32_t), 8);
    lb_init(&lb, sizeof(int32_t), 8);

    int32_t src[32], dst_rb[32], dst_lb[32];
    uint64_t offer = 0;
    int ok = 1;
    for (int step = 0; step < 200000; step++) {
        if ((step & 1) == 0) {
            uint32_t n = rng_next() % 21;                 /* 0..20 into cap 8 */
            for (uint32_t j = 0; j < n; j++)
                src[j] = (int32_t)value_at(offer + j);
            offer += n;
            uint32_t a1 = rb_write(&rb, src, n);
            uint32_t a2 = lb_write(&lb, src, n);
            ok &= (a1 == a2);
        } else {
            uint32_t n = rng_next() % 21;
            memset(dst_rb, 0x5a, sizeof(dst_rb));
            memset(dst_lb, 0x5a, sizeof(dst_lb));
            uint32_t g1 = rb_read(&rb, dst_rb, n);
            uint32_t g2 = lb_read(&lb, dst_lb, n);
            ok &= (g1 == g2);
            ok &= (memcmp(dst_rb, dst_lb, (size_t)g1 * sizeof(int32_t)) == 0);
        }
        ok &= (rb_ctr_load(&rb.overrun_events) == lb.overrun_events);
        ok &= (rb_ctr_load(&rb.overrun_samples) == lb.overrun_samples);
        ok &= (rb_ctr_load(&rb.underrun_events) == lb.underrun_events);
        ok &= (rb_ctr_load(&rb.underrun_samples) == lb.underrun_samples);
        ok &= (rb_fill(&rb) == lb_fill(&lb));
    }
    check("fuzz_counters_and_samples_match", ok);

    /* drain both and compare the tails */
    int32_t a[32], b[32];
    uint32_t ga, gb;
    do {
        ga = rb_read(&rb, a, 32);
        gb = lb_read(&lb, b, 32);
        ok &= (ga == gb && memcmp(a, b, (size_t)ga * sizeof(int32_t)) == 0);
    } while (ga > 0);
    check("fuzz_drain_identical", ok && rb_fill(&rb) == 0 && lb_fill(&lb) == 0);
    rb_destroy(&rb);
    lb_destroy(&lb);
}

/* ---------- concurrent equivalence: both impls on the same schedule ---------- */

#define BAR_PHASES 100000

typedef struct { uint16_t wn, rn; } Phase;

typedef struct {
    RingBuf* rb;
    LockBuf* lb;
    pthread_barrier_t* bar;
    const Phase* schedule;
    uint32_t phases;
    uint32_t* accept;       /* per-phase accepted write count */
    uint32_t* got;          /* per-phase delivered read count */
    int32_t* out;           /* all delivered samples in order */
    uint64_t out_n;
    int use_lockfree;       /* 1 = RingBuf, 0 = LockBuf */
    atomic_int* err;
} BarArg;

static void* bar_producer(void* p) {
    BarArg* a = (BarArg*)p;
    int32_t tmp[24];
    uint64_t offer = 0;
    for (uint32_t s = 0; s < a->phases; s++) {
        uint32_t n = a->schedule[s].wn;
        for (uint32_t j = 0; j < n; j++)
            tmp[j] = (int32_t)value_at(offer + j);
        offer += n;
        pthread_barrier_wait(a->bar);            /* B1: all four ready */
        uint32_t got = a->use_lockfree
            ? rb_write(a->rb, tmp, n)
            : lb_write(a->lb, tmp, n);
        a->accept[s] = got;
        pthread_barrier_wait(a->bar);            /* B2: both writes done */
        pthread_barrier_wait(a->bar);            /* B3: reads done, next phase */
    }
    return NULL;
}

static void* bar_consumer(void* p) {
    BarArg* a = (BarArg*)p;
    int32_t tmp[24];
    for (uint32_t s = 0; s < a->phases; s++) {
        pthread_barrier_wait(a->bar);            /* B1: ready */
        pthread_barrier_wait(a->bar);            /* B2: waits for both writes */
        uint32_t n = a->schedule[s].rn;
        uint32_t got = a->use_lockfree
            ? rb_read(a->rb, tmp, n)
            : lb_read(a->lb, tmp, n);
        a->got[s] = got;
        memcpy(a->out + a->out_n, tmp, (size_t)got * sizeof(int32_t));
        a->out_n += got;
        pthread_barrier_wait(a->bar);            /* B3: reads done */
    }
    /* drain the remainder after the producer has exited */
    uint32_t got;
    do {
        got = a->use_lockfree ? rb_read(a->rb, tmp, 24)
                              : lb_read(a->lb, tmp, 24);
        memcpy(a->out + a->out_n, tmp, (size_t)got * sizeof(int32_t));
        a->out_n += got;
    } while (got > 0);
    return NULL;
}

static void test_concurrent_equivalence(void) {
    Phase* schedule = (Phase*)malloc(sizeof(Phase) * BAR_PHASES);
    uint32_t* acc_rb = (uint32_t*)malloc(sizeof(uint32_t) * BAR_PHASES);
    uint32_t* acc_lb = (uint32_t*)malloc(sizeof(uint32_t) * BAR_PHASES);
    uint32_t* got_rb = (uint32_t*)malloc(sizeof(uint32_t) * BAR_PHASES);
    uint32_t* got_lb = (uint32_t*)malloc(sizeof(uint32_t) * BAR_PHASES);
    size_t out_cap = (size_t)BAR_PHASES * 24;
    int32_t* out_rb = (int32_t*)malloc(out_cap * sizeof(int32_t));
    int32_t* out_lb = (int32_t*)malloc(out_cap * sizeof(int32_t));
    for (uint32_t s = 0; s < BAR_PHASES; s++) {
        schedule[s].wn = (uint16_t)(1 + rng_next() % 12);
        schedule[s].rn = (uint16_t)(1 + rng_next() % 12);
    }

    RingBuf rb;
    LockBuf lb;
    rb_init(&rb, sizeof(int32_t), 8);            /* tiny ring: frequent edges */
    lb_init(&lb, sizeof(int32_t), 8);
    pthread_barrier_t bar;
    pthread_barrier_init(&bar, NULL, 4);
    atomic_int err_lf = 0, err_lk = 0;

    BarArg args[4] = {
        { &rb, &lb, &bar, schedule, BAR_PHASES, acc_rb, got_rb, out_rb, 0, 1, &err_lf },
        { &rb, &lb, &bar, schedule, BAR_PHASES, acc_rb, got_rb, out_rb, 0, 1, &err_lf },
        { &rb, &lb, &bar, schedule, BAR_PHASES, acc_lb, got_lb, out_lb, 0, 0, &err_lk },
        { &rb, &lb, &bar, schedule, BAR_PHASES, acc_lb, got_lb, out_lb, 0, 0, &err_lk },
    };
    pthread_t th[4];
    pthread_create(&th[0], NULL, bar_producer, &args[0]);
    pthread_create(&th[1], NULL, bar_consumer, &args[1]);
    pthread_create(&th[2], NULL, bar_producer, &args[2]);
    pthread_create(&th[3], NULL, bar_consumer, &args[3]);
    for (int i = 0; i < 4; i++) pthread_join(th[i], NULL);
    pthread_barrier_destroy(&bar);

    int ok = (atomic_load(&err_lf) == 0 && atomic_load(&err_lk) == 0);
    for (uint32_t s = 0; s < BAR_PHASES; s++) {
        ok &= (acc_rb[s] == acc_lb[s]);
        ok &= (got_rb[s] == got_lb[s]);
    }
    ok &= (args[1].out_n == args[3].out_n);
    ok &= (memcmp(out_rb, out_lb,
                  (size_t)args[1].out_n * sizeof(int32_t)) == 0);
    ok &= (rb_ctr_load(&rb.overrun_events) == lb.overrun_events);
    ok &= (rb_ctr_load(&rb.overrun_samples) == lb.overrun_samples);
    ok &= (rb_ctr_load(&rb.underrun_events) == lb.underrun_events);
    ok &= (rb_ctr_load(&rb.underrun_samples) == lb.underrun_samples);
    ok &= (rb_fill(&rb) == 0 && lb_fill(&lb) == 0);
    check("concurrent_sample_exact_vs_locked", ok);

    free(schedule); free(acc_rb); free(acc_lb); free(got_rb); free(got_lb);
    free(out_rb); free(out_lb);
    rb_destroy(&rb);
    lb_destroy(&lb);
}

/* ----------------- free-running SPSC stress with invariants ----------------- */

#if defined(__SANITIZE_THREAD__)
#define STRESS_TARGET 2000000u
#elif defined(__SANITIZE_ADDRESS__)
#define STRESS_TARGET 5000000u
#else
#define STRESS_TARGET 30000000u
#endif

typedef struct {
    RingBuf* rb;
    uint64_t target;
    uint32_t max_chunk;
    int slow;            /* thread takes occasional nanosleeps */
    int paced;           /* producer waits for room (guarantees no overrun) */
    /* shared control, allocated outside so struct copies stay in sync */
    _Atomic int* stop;
    atomic_int* err;
    /* thread-local accounting; the ring's own underrun/overrun counters are
       the authoritative statistics, these are cross-checks on the other side */
    uint64_t rng;
    uint64_t drop_events, drop_samples;
    uint64_t consumed;
    uint64_t final_expect, final_gaps;   /* set by consumer before return */
} Stress;

static uint32_t stress_next(Stress* st) {
    st->rng = st->rng * 6364136223846793005ULL + 1442695040888963407ULL;
    return (uint32_t)(st->rng >> 33);
}

static void tiny_sleep(Stress* st) {
    if (st->slow && (stress_next(st) & 7) == 0) {
        struct timespec ts = { 0, 1000 + (long)(stress_next(st) % 5000) };
        nanosleep(&ts, NULL);
    } else if ((stress_next(st) & 15) == 0) {
        sched_yield();
    }
}

static void* stress_producer(void* p) {
    Stress* st = (Stress*)p;
    uint64_t seq = 0;
    int32_t tmp[300];
    while (seq < st->target) {
        uint32_t want = 1 + stress_next(st) % st->max_chunk;
        if (want > st->target - seq) want = (uint32_t)(st->target - seq);
        if (st->paced) {
            while ((uint64_t)rb_fill(st->rb) + want > st->rb->cap)
                sched_yield();
        }
        for (uint32_t j = 0; j < want; j++)
            tmp[j] = (int32_t)(seq + j);
        uint32_t got = rb_write(st->rb, tmp, want);
        if (got < want) {
            st->drop_events++;
            st->drop_samples += want - got;
        }
        seq += want;
        tiny_sleep(st);
    }
    atomic_store_explicit(st->stop, 1, memory_order_release);
    return NULL;
}

/* Verify one delivered batch against the expected stream. Producer overruns
   create legitimate forward gaps (those samples were rejected at write time);
   only duplicates and reorders are real faults. Returns the number of gaps
   observed and advances *expect past the whole batch. */
static uint64_t verify_batch(Stress* st, const int32_t* tmp, uint32_t got,
                             uint64_t* expect) {
    uint64_t new_gaps = 0;
    for (uint32_t j = 0; j < got; j++) {
        uint64_t v = (uint64_t)tmp[j];
        if (v < *expect) {
            atomic_store(st->err, 1);           /* duplicate / reorder */
        } else if (v > *expect) {
            new_gaps += v - *expect;           /* producer dropped these */
            *expect = v;
        }
        (*expect)++;
        st->consumed++;
    }
    return new_gaps;
}

static void* stress_consumer(void* p) {
    Stress* st = (Stress*)p;
    int32_t tmp[300];
    uint64_t expect = 0;          /* next value required for an exact stream */
    uint64_t gaps = 0;            /* samples skipped due to producer overruns */
    for (;;) {
        if (atomic_load_explicit(st->stop, memory_order_acquire)) {
            /* producer is finishing: keep draining until it has stopped AND
               the ring is empty, so an in-flight final write is neither
               counted as underrun nor left behind */
            uint32_t got;
            for (;;) {
                got = rb_read(st->rb, tmp, 300);
                if (got > 0) {
                    gaps += verify_batch(st, tmp, got, &expect);
                } else if (atomic_load_explicit(st->stop,
                                                 memory_order_acquire)) {
                    break;
                } else {
                    sched_yield();
                }
            }
            break;
        }
        uint32_t want = 1 + stress_next(st) % st->max_chunk;
        uint32_t got = rb_read(st->rb, tmp, want);
        gaps += verify_batch(st, tmp, got, &expect);
        /* The authoritative gap-vs-overrun reconciliation happens after join,
           when both threads are quiescent; comparing here would race with the
           producer's counter publish. */
        tiny_sleep(st);
    }
    /* expect = consumed + observed gaps; it may stop short of target only by
       tail samples the producer rejected (never observable as a value) */
    if (expect > st->target) atomic_store(st->err, 1);
    st->final_expect = expect;
    st->final_gaps = gaps;
    return NULL;
}

static void run_stress(const char* name, uint32_t cap, uint32_t max_chunk,
                       int p_slow, int c_slow, int paced, uint64_t target) {
    RingBuf rb;
    rb_init(&rb, sizeof(int32_t), cap);
    _Atomic int stop;
    atomic_int err;
    atomic_init(&stop, 0);
    atomic_init(&err, 0);

    Stress prod = { &rb, target, max_chunk, p_slow, paced,
                    &stop, &err, 0x1111111111111111ULL,
                    0, 0, 0, 0, 0 };
    Stress cons = { &rb, target, max_chunk, c_slow, 0,
                    &stop, &err, 0x2222222222222222ULL,
                    0, 0, 0, 0, 0 };

    pthread_t tp, tc;
    pthread_create(&tp, NULL, stress_producer, &prod);
    pthread_create(&tc, NULL, stress_consumer, &cons);
    pthread_join(tp, NULL);
    pthread_join(tc, NULL);

    int ok = atomic_load(&err) == 0;
    uint64_t oe = rb_ctr_load(&rb.overrun_events);
    uint64_t os = rb_ctr_load(&rb.overrun_samples);
    /* quiescent reconciliation, all four must balance exactly:
       consumed + rejected == produced; observed gaps + tail rejects ==
       rejected; expected stream advanced only over produced indices */
    int c_balance = cons.consumed + os == target;
    int c_gaps = cons.final_gaps + (target - cons.final_expect) == os;
    int c_oe = prod.drop_events == oe;
    int c_os = prod.drop_samples == os;
    int c_empty = rb_fill(&rb) == 0;
    int c_paced = !paced || oe == 0;
    if (!(ok && c_balance && c_gaps && c_oe && c_os && c_empty && c_paced))
        printf("  diag: err=%d balance=%d gaps=%d oe=%d os=%d empty=%d paced=%d\n"
               "        consumed=%llu gaps=%llu tail=%llu overrun=%llu\n",
               ok, c_balance, c_gaps, c_oe, c_os, c_empty, c_paced,
               (unsigned long long)cons.consumed,
               (unsigned long long)cons.final_gaps,
               (unsigned long long)(target - cons.final_expect),
               (unsigned long long)os);
    ok &= c_balance && c_gaps && c_oe && c_os && c_empty && c_paced;
    printf("%s: target=%llu delivered=%llu overrun_ev=%llu overrun_smp=%llu "
           "underrun_ev=%llu underrun_smp=%llu\n",
           name, (unsigned long long)target,
           (unsigned long long)cons.consumed,
           (unsigned long long)rb_ctr_load(&rb.overrun_events),
           (unsigned long long)rb_ctr_load(&rb.overrun_samples),
           (unsigned long long)rb_ctr_load(&rb.underrun_events),
           (unsigned long long)rb_ctr_load(&rb.underrun_samples));
    check(name, ok);
    rb_destroy(&rb);
}

static void test_stress(void) {
    run_stress("stress_overrun_heavy",      2,   8, 0, 1, 0, STRESS_TARGET);
    run_stress("stress_underrun_heavy",    16,   8, 1, 0, 0, STRESS_TARGET);
    run_stress("stress_mixed",             64,  32, 1, 1, 0, STRESS_TARGET);
    run_stress("stress_tiny_ring_big_io",   2, 256, 0, 0, 0, STRESS_TARGET / 4);
    run_stress("stress_paced_no_drop",  1u<<18, 64, 0, 0, 1, STRESS_TARGET / 2);
}

/* ---------- underrun vs buffer size: simulated audio callback clock ---------- */

/* The consumer (audio thread) demands exactly PERIOD samples every tick. The
   producer (main thread) normally supplies PERIOD per tick, but stalls for
   1..max_stall ticks with probability stall_prob; on resume it bursts the
   owed samples in one write. The same stall sequence is replayed for every
   capacity so the columns differ only in buffer size. */
#define PERIOD 256u
#define TICKS 100000u

static double rng_double(void) {
    return (double)(rng_next() >> 8) / (double)(1u << 24);
}

static void sweep(const char* label, double stall_prob, int max_stall) {
    printf("\n[%s] stall_prob=%.3f max_stall=%d ticks, period=%u samples\n",
           label, stall_prob, max_stall, PERIOD);
    printf("%8s %12s %14s %16s %14s %16s\n",
           "cap", "cap/period", "under_events", "under_samples",
           "over_events", "over_samples");
    uint64_t seed_save = rng_state;
    for (uint32_t cap = 64; cap <= 8192; cap <<= 1) {
        rng_state = seed_save;                 /* identical stall timeline */
        RingBuf rb;
        rb_init(&rb, sizeof(int32_t), cap);
        int stall = 0, saved = 0;
        uint32_t burst = 0;
        for (uint32_t t = 0; t < TICKS; t++) {
            uint32_t produce = PERIOD;
            if (stall > 0) {
                stall--;
                produce = 0;
                if (stall == 0) burst = (uint32_t)saved * PERIOD;
            } else if (rng_double() < stall_prob) {
                saved = 1 + (int)(rng_next() % (uint32_t)max_stall);
                stall = saved;
                produce = 0;
            } else {
                produce = PERIOD + burst;      /* catch-up burst */
                burst = 0;
            }
            if (produce > 0) {
                int32_t tmp[PERIOD * 5];
                rb_write(&rb, tmp, produce);   /* values irrelevant here */
            }
            int32_t dst[PERIOD];
            rb_read(&rb, dst, PERIOD);         /* audio callback */
        }
        printf("%8u %12.2f %14llu %16llu %14llu %16llu\n",
               rb.cap, (double)rb.cap / PERIOD,
               (unsigned long long)rb_ctr_load(&rb.underrun_events),
               (unsigned long long)rb_ctr_load(&rb.underrun_samples),
               (unsigned long long)rb_ctr_load(&rb.overrun_events),
               (unsigned long long)rb_ctr_load(&rb.overrun_samples));
        rb_destroy(&rb);
    }
}

int main(void) {
    /* lock-free only earns its name when the platform has native atomics */
    _Atomic uint64_t probe;
    check("uint64_atomics_lockfree", atomic_is_lock_free(&probe));

    test_basic();
    test_wraparound();
    test_float_elements();
    test_fuzz_equivalence();
    test_concurrent_equivalence();
    test_stress();

    rng_state = 0xabcdef0123456789ULL;
    sweep("light jitter", 0.01, 2);
    rng_state = 0xabcdef0123456789ULL;
    sweep("heavy jitter", 0.03, 4);

    printf("\nall_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
