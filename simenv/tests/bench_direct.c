
/* bench_direct: the SAME particle source and workload, but the two inline
   accessors are macro-replaced by direct function calls before the module
   source is included. The simenv header is included first (guard prevents
   re-entry), so only the module's call sites are redirected. This isolates
   the vtable dispatch as the single difference between the two builds. */
#include "simenv/simenv.h"

typedef struct { uint64_t state, inc; } DirectPcg;

static __attribute__((noinline)) uint32_t direct_pcg(DirectPcg* p) {
    uint64_t old = p->state;
    p->state = old * 6364136223846793005ull + p->inc;
    uint32_t xors = (uint32_t)(((old >> 18u) ^ old) >> 27u);
    uint32_t rot = (uint32_t)(old >> 59u);
    return (xors >> rot) | (xors << ((-(int32_t)rot) & 31));
}

static DirectPcg g_pcg = { 0x123456789abcdef0ull, 0x853c49e6748fea9bull };

static float direct_randf(void) {
    return (float)(direct_pcg(&g_pcg) >> 8) * (1.0f / 16777216.0f);
}
static void direct_randf_n(void* env_ignored, float* out, int n) {
    (void)env_ignored;
    for (int i = 0; i < n; i++) out[i] = direct_randf();
}
static double direct_dt(void) { return 0.1; }

#define sim_env_randf(env) direct_randf()
#define sim_env_randf_n(env, out, n) direct_randf_n((env), (out), (n))
#define sim_env_dt(env)    direct_dt()
#define main particle_standalone_direct
#include "../../particle/src/particle.c"
#undef main
#undef sim_env_randf
#undef sim_env_randf_n
#undef sim_env_dt

#define BENCH_FRAMES 50000

double bench_frames_direct(void) {
    World w;
    world_init(&w);
    emitter_add(&w, 1, 40, 2);
    emitter_add(&w, 2, 30, 9);
    emitter_add(&w, 3, 20, 5);
    for (int f = 0; f < BENCH_FRAMES; f++) {
        float dt = (float)direct_dt();
        spawn_frame(&w, 50, 0);
        update(&w, dt);
    }
    double sum = 0.0;
    for (int i = 0; i < MAX_PARTICLES; i++)
        if (w.pool[i].alive) sum += (double)w.pool[i].x + (double)w.pool[i].y;
    return sum;
}
