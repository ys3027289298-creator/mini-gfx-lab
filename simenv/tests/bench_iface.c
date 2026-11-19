
/* bench_iface: the particle module exactly as shipped — time and randomness
   through the SimEnv vtable. main is renamed so the module links as a library. */
#include "simenv/simenv.h"

#define main particle_standalone_iface
#include "../../particle/src/particle.c"
#undef main

#define BENCH_FRAMES 50000

/* returns a position checksum so the optimizer cannot drop the work */
double bench_frames_iface(void) {
    static const double one_dt[1] = { 0.1 };
    Clock* clock = sim_clock_scripted(one_dt, 1);
    Rng* rng = sim_rng_pcg32(0xC0FFEEull);
    SimEnv env = { clock, rng };
    World w;
    world_init(&w);
    emitter_add(&w, 1, 40, 2);
    emitter_add(&w, 2, 30, 9);
    emitter_add(&w, 3, 20, 5);
    for (int f = 0; f < BENCH_FRAMES; f++) {
        float dt = (float)sim_env_dt(&env);
        spawn_frame(&w, 50, &env);
        update(&w, dt);
    }
    double sum = 0.0;
    for (int i = 0; i < MAX_PARTICLES; i++)
        if (w.pool[i].alive) sum += (double)w.pool[i].x + (double)w.pool[i].y;
    sim_clock_free(clock);
    sim_rng_free(rng);
    return sum;
}
