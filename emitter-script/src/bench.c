/* bench: measure compile and run cost of the example scripts.

   Uses clock_gettime(CLOCK_MONOTONIC) and enough iterations for a stable
   mean. Reports nanoseconds per compile / per run and compares compile cost
   with a single particle step to show the parser is not a hot path. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "ems.h"

static double now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e9 + (double)ts.tv_nsec;
}

static char* read_file(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return NULL;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
    long n = ftell(f);
    if (n < 0) { fclose(f); return NULL; }
    rewind(f);
    char* buf = malloc((size_t)n + 1);
    if (!buf) { fclose(f); return NULL; }
    size_t got = fread(buf, 1, (size_t)n, f);
    buf[got] = '\0';
    fclose(f);
    return buf;
}

static const char* FILES[] = {
    "01_single.emitters", "02_priority.emitters", "03_curve_quota.emitters",
    "04_curve_velocity.emitters", "05_curve_life.emitters", "06_step.emitters",
    "07_cubic.emitters", "08_pool_saturation.emitters", "09_fractional_quota.emitters",
    "10_budget_split.emitters",
};

int main(int argc, char** argv) {
    const char* dir = argc > 1 ? argv[1] : "examples";
    const int compile_iters = 20000;
    const int run_iters = 2000;

    printf("%-32s %8s %10s %10s %10s\n", "script", "bytes", "compile_ns", "run_ns", "spawned");
    double total_compile = 0.0;
    int total_spawned = 0;
    size_t total_bytes = 0;
    for (size_t i = 0; i < sizeof(FILES) / sizeof(FILES[0]); i++) {
        char path[512];
        snprintf(path, sizeof(path), "%s/%s", dir, FILES[i]);
        char* src = read_file(path);
        if (!src) { printf("%s: missing\n", FILES[i]); return 1; }
        EmsProgram prog;
        EmsError err;
        if (ems_compile(src, &prog, &err) != 0) {
            printf("%s: compile failed: %s\n", FILES[i], err.message);
            free(src);
            return 1;
        }
        /* warm up */
        for (int k = 0; k < 1000; k++) ems_compile(src, &prog, &err);

        double t0 = now_ns();
        for (int k = 0; k < compile_iters; k++) ems_compile(src, &prog, &err);
        double compile_ns = (now_ns() - t0) / compile_iters;

        EmsResult res;
        ems_run(&prog, &res);
        t0 = now_ns();
        for (int k = 0; k < run_iters; k++) ems_run(&prog, &res);
        double run_ns = (now_ns() - t0) / run_iters;

        int spawned = 0;
        for (int e = 0; e < prog.emitter_count; e++) spawned += res.spawned[e];
        printf("%-32s %8zu %10.0f %10.0f %10d\n", FILES[i], strlen(src),
               compile_ns, run_ns, spawned);
        total_compile += compile_ns;
        total_spawned += spawned;
        total_bytes += strlen(src);
        free(src);
    }
    printf("mean_compile_ns=%.0f mean_bytes=%.0f\n",
           total_compile / (double)(sizeof(FILES) / sizeof(FILES[0])),
           (double)total_bytes / (double)(sizeof(FILES) / sizeof(FILES[0])));
    printf("spawned_total=%d\n", total_spawned);

    /* a single script-loaded emitter step is the comparison point */
    const char* one = "pool 4096\nbudget 50\nticks 10\ndt 0.1\n"
                      "emitter p priority 0 { quota 50 pos 0,0 vel 1,0.5 life 2 }\n";
    EmsProgram prog;
    EmsError err;
    ems_compile(one, &prog, &err);
    EmsResult res;
    ems_run(&prog, &res);
    double t0 = now_ns();
    for (int k = 0; k < run_iters; k++) ems_run(&prog, &res);
    double step_ns = (now_ns() - t0) / run_iters;
    printf("ten_ticks_ns=%.1f (per_tick_ns=%.1f)\n", step_ns, step_ns / 10.0);
    return 0;
}
