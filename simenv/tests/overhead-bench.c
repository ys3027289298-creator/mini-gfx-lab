
/* overhead-bench: interface overhead measured against TOTAL frame time.
   Two variants of the identical particle workload are linked:
     bench_frames_direct()  - direct inlinable calls
     bench_frames_iface()   - SimEnv vtable (one indirect call per access)
   The denominator is the whole frame: 4096-slot pool scan, trig, spawn and
   integration. Direct and interface runs are interleaved in pairs so CPU
   frequency and cache state bias both equally; the headline number is the
   median paired delta. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "simenv/simenv.h"

double bench_frames_direct(void);
double bench_frames_iface(void);

#define RUNS 15
#define MICRO_ITERS 50000000

static double now_sec(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + (double)t.tv_nsec / 1e9;
}

static int cmp_double(const void* a, const void* b) {
    double x = *(const double*)a, y = *(const double*)b;
    return (x > y) - (x < y);
}

int main(void) {
    double pair_pct[RUNS];
    double t_direct[RUNS], t_iface[RUNS];
    volatile double sink = 0.0;

    (void)bench_frames_direct();      /* warmup: fault pages, ramp the core */
    (void)bench_frames_iface();

    for (int r = 0; r < RUNS; r++) {
        double t0, t1;
        /* alternate which variant starts the pair */
        if (r & 1) {
            t0 = now_sec(); sink += bench_frames_iface();  t1 = now_sec();
            t_iface[r] = t1 - t0;
            t0 = now_sec(); sink += bench_frames_direct(); t1 = now_sec();
            t_direct[r] = t1 - t0;
        } else {
            t0 = now_sec(); sink += bench_frames_direct(); t1 = now_sec();
            t_direct[r] = t1 - t0;
            t0 = now_sec(); sink += bench_frames_iface();  t1 = now_sec();
            t_iface[r] = t1 - t0;
        }
        pair_pct[r] = 100.0 * (t_iface[r] - t_direct[r]) / t_direct[r];
    }

    qsort(pair_pct, RUNS, sizeof(double), cmp_double);
    qsort(t_direct, RUNS, sizeof(double), cmp_double);
    qsort(t_iface, RUNS, sizeof(double), cmp_double);

    double median_pct = pair_pct[RUNS / 2];
    double worst_pct = pair_pct[RUNS - 1];
    double best_pct = pair_pct[0];

    printf("frame_workload=50000 frames x (spawn 50 + update 4096-slot pool), %d paired runs\n", RUNS);
    printf("direct_median_s=%.4f  interface_median_s=%.4f\n",
           t_direct[RUNS / 2], t_iface[RUNS / 2]);
    printf("paired_overhead_pct median=%.3f best=%.3f worst=%.3f\n",
           median_pct, best_pct, worst_pct);
    /* This timing comparison is diagnostic only: on a contended host (or an
       instrumented build) scheduling noise dwarfs the sub-microsecond delta.
       The enforceable 2% budget lives in dispatch-micro's analytical bound.
       Report status, never fail the suite on timing noise. */
    printf("diagnostic median_under_2pct=%d (hard budget: see dispatch-micro)\n",
           median_pct < 2.0);
    printf("(negative values mean the vtable build landed inside run-to-run noise)\n");

    /* absolute per-call dispatch cost, for the record: one indirect call with
       no surrounding frame work. Both accessors stay static-inline; this is
       just the vtable jump + RNG body under the interface. */
    static const double one_dt[1] = { 0.016 };
    SimEnv micro = { sim_clock_scripted(one_dt, 1), sim_rng_pcg32(1) };
    uint32_t acc = 0;
    double m0 = now_sec();
    for (long i = 0; i < MICRO_ITERS; i++) acc += sim_env_u32(&micro);
    double m1 = now_sec();
    for (long i = 0; i < MICRO_ITERS; i++) sink += sim_env_dt(&micro);
    double m2 = now_sec();
    printf("absolute_per_call_ns rng=%.3f clock=%.3f (acc=%u)\n",
           (m1 - m0) * 1e9 / MICRO_ITERS,
           (m2 - m1) * 1e9 / MICRO_ITERS, acc);

    sim_clock_free(micro.clock);
    sim_rng_free(micro.rng);
    (void)sink;
    return 0;   /* diagnostic only; dispatch-micro enforces the budget */
}
