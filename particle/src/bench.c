
/* bench.c — runtime allocation audit + per-frame timing for particle-pool.
   The whole allocator family (malloc/calloc/realloc/aligned_alloc/posix_memalign/
   valloc/memalign/pvalloc/free) is intercepted with ld --wrap, so any heap
   traffic during the measured frame loop increments the counters. */
#define _POSIX_C_SOURCE 199309L
#define PARTICLE_NO_MAIN
#include "particle.c"   /* reuse the pool implementation as a translation unit */

#include <time.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <sys/resource.h>

void *__real_malloc(size_t);
void *__real_calloc(size_t, size_t);
void *__real_realloc(void *, size_t);
void *__real_aligned_alloc(size_t, size_t);
void *__real_valloc(size_t);
void *__real_memalign(size_t, size_t);
void *__real_pvalloc(size_t);
int   __real_posix_memalign(void **, size_t, size_t);
void  __real_free(void *);

/* volatile: without it, gcc at -O2 caches the counters in registers across
   the wrapped calls and reports stale (zero) deltas. */
static volatile long n_alloc, n_realloc, n_free, bytes_alloc;

#define COUNT_ALLOC(p, n) do { n_alloc++; bytes_alloc += (long)(n); return __real_##p; } while (0)

void *__wrap_malloc(size_t s) { COUNT_ALLOC(malloc(s), s); }
void *__wrap_calloc(size_t a, size_t b) { COUNT_ALLOC(calloc(a, b), a * b); }
void *__wrap_realloc(void *p, size_t s) { n_alloc++; n_realloc++; bytes_alloc += (long)s; return __real_realloc(p, s); }
void *__wrap_aligned_alloc(size_t a, size_t s) { COUNT_ALLOC(aligned_alloc(a, s), s); }
void *__wrap_valloc(size_t s) { COUNT_ALLOC(valloc(s), s); }
void *__wrap_memalign(size_t a, size_t s) { COUNT_ALLOC(memalign(a, s), s); }
void *__wrap_pvalloc(size_t s) { COUNT_ALLOC(pvalloc(s), s); }
int   __wrap_posix_memalign(void **m, size_t a, size_t s) { n_alloc++; bytes_alloc += (long)s; return __real_posix_memalign(m, a, s); }
void  __wrap_free(void *p) { n_free++; __real_free(p); }

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

/* Static storage: the harness itself must not heap-allocate during timing. */
static World world;
static double frame_ms[2048];

static double max_of(const double *a, int n) {
    double m = a[0];
    for (int i = 1; i < n; i++) if (a[i] > m) m = a[i];
    return m;
}

int main(void) {
    const int frames_a = 600;                  /* realistic churn workload */
    const int frames_b = 300;                  /* worst case: full-pool spawn every frame */
    const int frames = frames_a + frames_b;
    const int full_spawn = MAX_PARTICLES;       /* frame 0 fills the pool once */
    const int churn_budget = 2000;              /* later frames: eviction path */

    /* warm up stdio so any lazy allocator plumbing happens before the snapshot */
    printf("particle bench: capacity=%d emitters<= %d\n", MAX_PARTICLES, MAX_EMITTERS);

    world_init(&world);
    emitter_add(&world, 1, 2500, 9);
    emitter_add(&world, 2, 2000, 5);
    emitter_add(&world, 3, 1500, 2);

    long a0 = n_alloc, r0 = n_realloc, f0 = n_free, b0 = bytes_alloc;

    struct rusage ru0, ru1;
    getrusage(RUSAGE_SELF, &ru0);

    double t_run_start = now_ms();
    for (int f = 0; f < frames_a; f++) {
        double t0 = now_ms();
        spawn_frame(&world, f == 0 ? full_spawn : churn_budget);
        update(&world, 0.1f);
        frame_ms[f] = now_ms() - t0;
    }
    /* Worst case: every frame requests MAX_PARTICLES spawns while the pool is
       already full, so every spawn takes the eviction branch. */
    for (int f = 0; f < frames_b; f++) {
        double t0 = now_ms();
        spawn_frame(&world, MAX_PARTICLES);
        update(&world, 0.1f);
        frame_ms[frames_a + f] = now_ms() - t0;
    }
    double run_ms = now_ms() - t_run_start;
    getrusage(RUSAGE_SELF, &ru1);

    long da = n_alloc - a0, dr = n_realloc - r0, df = n_free - f0, db = bytes_alloc - b0;

    /* Canary: go through the public allocator API (which the linker routes
       through our __wrap_ symbols) so a dead interceptor cannot produce a
       misleading "zero" result. */
    void *c = malloc(16);
    c = realloc(c, 32);
    free(c);
    if (n_alloc - a0 != da + 2 || n_realloc - r0 != dr + 1 || n_free - f0 != df + 1) {
        printf("CANARY FAIL: interceptor counters are not live\n");
        return 2;
    }

    double sorted[2048];
    for (int i = 0; i < frames; i++) sorted[i] = frame_ms[i];
    /* insertion sort, no allocation */
    for (int i = 1; i < frames; i++) {
        double key = sorted[i]; int j = i - 1;
        while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
        sorted[j + 1] = key;
    }
    double max_ms = sorted[frames - 1];
    double p99_ms = sorted[(frames * 99) / 100];
    double worst_ms = max_of(&frame_ms[frames_a], frames_b);
    double avg_ms = run_ms / frames;
    double sq_sum = 0.0;
    for (int i = 0; i < frames; i++) sq_sum += (frame_ms[i] - avg_ms) * (frame_ms[i] - avg_ms);
    double stddev_ms;
    {
        double x = sq_sum / frames, guess = x > 0 ? x : 1e-12;
        for (int it = 0; it < 24; it++) guess = 0.5 * (guess + x / guess);
        stddev_ms = guess;
    }

    long rss_kb = ru1.ru_maxrss - ru0.ru_maxrss > 0
                ? ru1.ru_maxrss - ru0.ru_maxrss : 0;

    printf("\n== runtime allocation audit (measured section: %d frames, %d worst-case) ==\n",
           frames, frames_b);
    printf("alloc_calls=%ld realloc_calls=%ld free_calls=%ld bytes=%ld\n", da, dr, df, db);
    printf("canary=pass (wrap interceptors verified live)\n");
    printf("\n== state footprint ==\n");
    printf("sizeof(Particle)=%zu sizeof(World)=%zu (%.1f KiB)\n",
           sizeof(Particle), sizeof(World), sizeof(World) / 1024.0);
    printf("alive=%d high_water=%d evictions=%d free_head=%d\n",
           world.alive_count, world.high_water, world.evictions, world.free_head);
    printf("rss_delta_ru_maxrss=%ld KiB (monotonic counter; 0 means no new high-water)\n", rss_kb);
    printf("\n== per-frame timing (ms, %d frames, clock_monotonic) ==\n", frames);
    printf("max=%.4f p99=%.4f avg=%.4f stddev=%.4f total=%.3f\n",
           max_ms, p99_ms, avg_ms, stddev_ms, run_ms);
    printf("worst_case_phase (full pool, %d spawns/frame x %d): max=%.4f ms\n",
           MAX_PARTICLES, frames_b, worst_ms);
    printf("budget_2ms: %s (worst %.4f ms, margin %.2fx)\n",
           worst_ms < 2.0 ? "PASS" : "FAIL", worst_ms, 2.0 / worst_ms);

    int fail = 0;
    if (da != 0 || dr != 0 || df != 0 || db != 0) fail = 1;
    if (worst_ms >= 2.0) fail = 1;
    if (world.high_water > MAX_PARTICLES) fail = 1;
    printf("\noverall=%s\n", fail ? "FAIL" : "PASS");
    return fail;
}
