
/* tape-cross: prove a tape survives the process boundary.
   usage:
     tape-cross write <path>   run + record, save tape, print frame hashes
     tape-cross read  <path>   rebuild env from the tape, print frame hashes
   The two printed hash columns must be identical. */
#include "simenv/simenv.h"
#include <stdio.h>
#include <string.h>

#define main particle_standalone_tc
#include "../../particle/src/particle.c"
#undef main

#define FRAMES 20
#define BUDGET 50

static const double frame_dts[FRAMES] = {
    0.10, 0.12, 0.08, 0.11, 0.09, 0.13, 0.07, 0.10, 0.14, 0.08,
    0.10, 0.11, 0.09, 0.12, 0.10, 0.08, 0.13, 0.10, 0.11, 0.09
};

static uint32_t hash_world(const World* w) {
    uint32_t h = 2166136261u;
    const unsigned char* b = (const unsigned char*)w->pool;
    for (size_t i = 0; i < sizeof(w->pool); i++) { h ^= b[i]; h *= 16777219u; }
    return h;
}

int main(int argc, char** argv) {
    if (argc < 3) { printf("usage: tape-cross write|read <path>\n"); return 2; }
    const char* path = argv[2];
    World w;
    SimEnv env;
    Clock* clock; Rng* rng;
    SimTape tape;

    if (strcmp(argv[1], "write") == 0) {
        Clock* inner_c = sim_clock_scripted(frame_dts, FRAMES);
        Rng* inner_r = sim_rng_pcg32(0x5EED1234ull);
        clock = sim_clock_recorder(inner_c);
        rng = sim_rng_recorder(inner_r);
        env.clock = clock; env.rng = rng;
        world_init(&w);
        emitter_add(&w, 1, 40, 2); emitter_add(&w, 2, 30, 9); emitter_add(&w, 3, 20, 5);
        for (int f = 0; f < FRAMES; f++) {
            float dt = (float)sim_env_dt(&env);
            spawn_frame(&w, BUDGET, &env);
            update(&w, dt);
            printf("%u\n", hash_world(&w));
        }
        int nd = 0, nr = 0;
        const double* dts = sim_clock_recorder_dts(clock, &nd);
        const uint32_t* rvs = sim_recorder_values(rng, &nr);
        tape.magic = SIM_TAPE_MAGIC; tape.version = SIM_TAPE_VERSION;
        tape.seed = 0x5EED1234ull; tape.n_dts = nd; tape.n_rand = nr;
        tape.dts = (double*)dts; tape.rand = (uint32_t*)rvs;
        if (sim_tape_save(&tape, path) != 0) { printf("save failed\n"); return 1; }
        sim_clock_recorder_free(clock); sim_recorder_free(rng);
        sim_clock_free(inner_c); sim_rng_free(inner_r);
        return 0;
    }

    if (strcmp(argv[1], "read") == 0) {
        if (sim_tape_load(&tape, path) != 0) { printf("load failed\n"); return 1; }
        if (sim_tape_env(&tape, &env, &clock, &rng) != 0) { printf("env failed\n"); return 1; }
        world_init(&w);
        emitter_add(&w, 1, 40, 2); emitter_add(&w, 2, 30, 9); emitter_add(&w, 3, 20, 5);
        for (int f = 0; f < FRAMES; f++) {
            float dt = (float)sim_env_dt(&env);
            spawn_frame(&w, BUDGET, &env);
            update(&w, dt);
            printf("%u\n", hash_world(&w));
        }
        sim_tape_free(&tape);
        sim_clock_free(clock); sim_rng_free(rng);
        return 0;
    }

    printf("unknown mode\n");
    return 2;
}
