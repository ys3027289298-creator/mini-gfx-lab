
/* dispatch-micro: isolates the pure indirect-call increment and applies it
   to the actual call topology. Both scalar paths call the SAME noinline PCG
   function; only the call mode (indirect vs direct) differs, so the delta is
   dispatch cost alone. The particle module's bulk path pays one dispatch per
   spawn (fill_f32, 3 internal direct steps) plus one clock tick per frame.
   The frame denominator is measured from the real workload. */
#define _POSIX_C_SOURCE 199309L
#include "simenv/simenv.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

double bench_frames_iface(void);   /* 50000 real particle frames */
#define MEASURED_FRAMES 50000

#define ITERS 100000000
#define RUNS  9
#define SPAWNS_PER_FRAME  50      /* spawn budget */
#define TICKS_PER_FRAME    1
#define DRAWS_PER_SPAWN    3      /* angle, speed, life */

typedef struct { uint64_t state, inc; } PcgState;

static __attribute__((noinline)) uint32_t pcg_body(PcgState* p) {
    uint64_t old = p->state;
    p->state = old * 6364136223846793005ull + p->inc;
    uint32_t xors = (uint32_t)(((old >> 18u) ^ old) >> 27u);
    uint32_t rot = (uint32_t)(old >> 59u);
    return (xors >> rot) | (xors << ((-(int32_t)rot) & 31));
}

/* vtable scalar entry, object mirrors Pcg32 layout (header then state) */
static uint32_t vt_pcg(Rng* self) {
    return pcg_body((PcgState*)((char*)self + sizeof(Rng)));
}

/* vtable bulk entry: one indirect call, DRAWS_PER_SPAWN direct steps inside */
static void vt_fill3(Rng* self, float* out, int n) {
    PcgState* p = (PcgState*)((char*)self + sizeof(Rng));
    for (int i = 0; i < n; i++)
        out[i] = (float)(pcg_body(p) >> 8) * (1.0f / 16777216.0f);
}

typedef struct { Rng vtable; PcgState state; } VtPcg;

/* direct-call version of the same bulk work */
static __attribute__((noinline)) void direct_fill3(PcgState* p, float* out, int n) {
    for (int i = 0; i < n; i++)
        out[i] = (float)(pcg_body(p) >> 8) * (1.0f / 16777216.0f);
}

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
    PcgState dp = { 0x123456789abcdef0ull, 0x853c49e6748fea9bull };
    VtPcg vp = {
        { vt_pcg, 0, 0, vt_fill3 },
        { 0x123456789abcdef0ull, 0x853c49e6748fea9bull }
    };
    Rng* vt = &vp.vtable;
    uint32_t sink = 0;
    float buf[DRAWS_PER_SPAWN];
    double td[RUNS], ti[RUNS], tbd[RUNS], tbi[RUNS];

    for (int r = 0; r < RUNS; r++) {
        double t0, t1;
        /* ---- scalar dispatch delta ---- */
        if (r & 1) {
            t0 = now_sec();
            for (long i = 0; i < ITERS; i++) sink += vt->next_u32(vt);
            t1 = now_sec(); ti[r] = t1 - t0;
            t0 = now_sec();
            for (long i = 0; i < ITERS; i++) sink += pcg_body(&dp);
            t1 = now_sec(); td[r] = t1 - t0;
        } else {
            t0 = now_sec();
            for (long i = 0; i < ITERS; i++) sink += pcg_body(&dp);
            t1 = now_sec(); td[r] = t1 - t0;
            t0 = now_sec();
            for (long i = 0; i < ITERS; i++) sink += vt->next_u32(vt);
            t1 = now_sec(); ti[r] = t1 - t0;
        }
        /* ---- bulk dispatch delta (the topology the module actually uses) ---- */
        long batches = ITERS / DRAWS_PER_SPAWN;
        if (r & 1) {
            t0 = now_sec();
            for (long i = 0; i < batches; i++) vt->fill_f32(vt, buf, DRAWS_PER_SPAWN);
            t1 = now_sec(); tbi[r] = t1 - t0;
            t0 = now_sec();
            for (long i = 0; i < batches; i++) direct_fill3(&dp, buf, DRAWS_PER_SPAWN);
            t1 = now_sec(); tbd[r] = t1 - t0;
        } else {
            t0 = now_sec();
            for (long i = 0; i < batches; i++) direct_fill3(&dp, buf, DRAWS_PER_SPAWN);
            t1 = now_sec(); tbd[r] = t1 - t0;
            t0 = now_sec();
            for (long i = 0; i < batches; i++) vt->fill_f32(vt, buf, DRAWS_PER_SPAWN);
            t1 = now_sec(); tbi[r] = t1 - t0;
        }
        sink += (uint32_t)(buf[0] * 1e6f);
    }
    qsort(td, RUNS, sizeof(double), cmp_double);
    qsort(ti, RUNS, sizeof(double), cmp_double);
    qsort(tbd, RUNS, sizeof(double), cmp_double);
    qsort(tbi, RUNS, sizeof(double), cmp_double);

    double scalar_dispatch_ns =
        (ti[RUNS / 2] - td[RUNS / 2]) * 1e9 / ITERS;
    if (scalar_dispatch_ns < 0) scalar_dispatch_ns = 0.0;
    long batches = ITERS / DRAWS_PER_SPAWN;
    double bulk_dispatch_ns =
        (tbi[RUNS / 2] - tbd[RUNS / 2]) * 1e9 / batches;
    if (bulk_dispatch_ns < 0) bulk_dispatch_ns = 0.0;

    /* measured end-to-end frame time from the interface build */
    double ft0 = now_sec();
    (void)bench_frames_iface();
    double frame_est_ns = (now_sec() - ft0) * 1e9 / MEASURED_FRAMES;

    /* frame topology: SPAWNS_PER_FRAME bulk dispatches + 1 clock tick,
       clock tick priced at the same scalar indirect-call delta */
    double per_frame_dispatch =
        bulk_dispatch_ns * SPAWNS_PER_FRAME +
        scalar_dispatch_ns * TICKS_PER_FRAME;
    double upper_pct = 100.0 * per_frame_dispatch / frame_est_ns;

    printf("scalar_dispatch_ns=%.3f  bulk_dispatch_ns_per_batch=%.3f\n",
           scalar_dispatch_ns, bulk_dispatch_ns);
    printf("dispatches_per_frame=%d bulk + %d tick, dispatch_ns=%.3f\n",
           SPAWNS_PER_FRAME, TICKS_PER_FRAME, per_frame_dispatch);
    printf("frame_time_ns~=%.0f  analytical_upper_bound_pct=%.3f\n",
           frame_est_ns, upper_pct);
    printf("budget=2.000%%  pass=%d  (sink=%u)\n", upper_pct < 2.0, sink);
    return upper_pct < 2.0 ? 0 : 1;
}
