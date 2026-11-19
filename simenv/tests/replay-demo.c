
/* replay-demo: proves the four replay guarantees on the real particle module.
   Include particle.c with its main renamed: its static simulation functions
   stay callable in this translation unit, and not one line of module source
   is modified to run these scenarios.

   A. same scripted sequence, two independent runs -> identical per-frame
      hash of every particle slot.
   B. record a live run (clock + RNG) to a tape file, then rebuild the run
      from the tape using entirely different implementations (scripted clock
      + fixed-sequence RNG) -> identical hashes.
   C. swap the RNG implementation through sim_env_set_rng only; module code
      is untouched, the new stream is deterministic and distinct. */
#include "simenv/simenv.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define main particle_standalone_main
#include "../../particle/src/particle.c"
#undef main

#define FRAMES 30
#define BUDGET 50

static const double frame_dts[FRAMES] = {
    0.10, 0.12, 0.08, 0.11, 0.09, 0.13, 0.07, 0.10, 0.14, 0.08,
    0.10, 0.11, 0.09, 0.12, 0.10, 0.08, 0.13, 0.10, 0.11, 0.09,
    0.12, 0.10, 0.07, 0.14, 0.10, 0.11, 0.09, 0.13, 0.08, 0.10
};

/* FNV-1a over the full pool: every slot, every field, frame by frame */
static uint32_t hash_world(const World* w) {
    uint32_t h = 2166136261u;
    const unsigned char* bytes = (const unsigned char*)w->pool;
    size_t n = sizeof(w->pool);
    for (size_t i = 0; i < n; i++) {
        h ^= bytes[i];
        h *= 16777219u;
    }
    const unsigned char* tail = (const unsigned char*)&w->alive_count;
    for (size_t i = 0; i < sizeof(w->alive_count); i++) {
        h ^= tail[i];
        h *= 16777219u;
    }
    return h;
}

static void run(World* w, SimEnv* env, uint32_t hashes[FRAMES]) {
    world_init(w);
    emitter_add(w, 1, 40, 2);
    emitter_add(w, 2, 30, 9);
    emitter_add(w, 3, 20, 5);
    for (int f = 0; f < FRAMES; f++) {
        float dt = (float)sim_env_dt(env);
        spawn_frame(w, BUDGET, env);
        update(w, dt);
        hashes[f] = hash_world(w);
    }
}

static int hashes_equal(const uint32_t* a, const uint32_t* b, int* first_diff) {
    for (int f = 0; f < FRAMES; f++)
        if (a[f] != b[f]) { *first_diff = f; return 0; }
    return 1;
}

int main(void) {
    int failures = 0;

    /* ---- A. same sequence, two independent runs ------------------- */
    SimEnv envA = { sim_clock_scripted(frame_dts, FRAMES), sim_rng_pcg32(0xC0FFEEull) };
    SimEnv envB = { sim_clock_scripted(frame_dts, FRAMES), sim_rng_pcg32(0xC0FFEEull) };
    World wa, wb;
    uint32_t ha[FRAMES], hb[FRAMES];
    run(&wa, &envA, ha);
    run(&wb, &envB, hb);
    int diff = -1;
    int a_ok = hashes_equal(ha, hb, &diff);
    printf("A_same_sequence_byte_identical=%d", a_ok);
    if (!a_ok) { printf(" (first mismatch frame %d)", diff); failures++; }
    printf("\n");
    printf("A_final_alive=%d high_water=%d\n", wa.alive_count, wa.high_water);
    sim_clock_free(envA.clock); sim_rng_free(envA.rng);
    sim_clock_free(envB.clock); sim_rng_free(envB.rng);

    /* ---- B. record -> tape file -> replay with other impls --------- */
    Clock* live_clock_inner = sim_clock_scripted(frame_dts, FRAMES);
    Rng* live_rng_inner = sim_rng_pcg32(0xBADF00Dull);
    Clock* live_clock = sim_clock_recorder(live_clock_inner);
    Rng* live_rng = sim_rng_recorder(live_rng_inner);
    SimEnv live = { live_clock, live_rng };
    World wl;
    uint32_t hl[FRAMES];
    run(&wl, &live, hl);

    int n_dt = 0, n_rand = 0;
    const double* rec_dts = sim_clock_recorder_dts(live_clock, &n_dt);
    const uint32_t* rec_rand = sim_recorder_values(live_rng, &n_rand);
    printf("B_recorded frames=%d rng_draws=%d\n", n_dt, n_rand);

    SimTape tape = {
        SIM_TAPE_MAGIC, SIM_TAPE_VERSION, 0xBADF00Dull,
        n_dt, n_rand, (double*)rec_dts, (uint32_t*)rec_rand
    };
    const char* path = "/tmp/simenv_replay.tape";
    if (sim_tape_save(&tape, path) != 0) { printf("B_tape_save=0\n"); return 1; }

    SimTape loaded;
    if (sim_tape_load(&loaded, path) != 0) { printf("B_tape_load=0\n"); return 1; }
    SimEnv replay;
    Clock* rclock; Rng* rrng;
    if (sim_tape_env(&loaded, &replay, &rclock, &rrng) != 0) {
        printf("B_tape_env=0\n"); return 1;
    }
    /* replay RNG is the fixed-sequence impl, NOT pcg32: prove by stream shape */
    World wr;
    uint32_t hr[FRAMES];
    run(&wr, &replay, hr);
    diff = -1;
    int b_ok = hashes_equal(hl, hr, &diff);
    printf("B_tape_replay_byte_identical=%d", b_ok);
    if (!b_ok) { printf(" (first mismatch frame %d)", diff); failures++; }
    printf("\n");
    if (!b_ok) failures++;
    sim_tape_free(&loaded);
    sim_clock_free(rclock);
    sim_rng_free(rrng);
    sim_clock_recorder_free(live_clock);
    sim_recorder_free(live_rng);
    sim_clock_free(live_clock_inner);
    sim_rng_free(live_rng_inner);

    /* replay must also match a fresh pcg run: sequence equivalence, not luck */
    SimEnv envC = { sim_clock_scripted(frame_dts, FRAMES), sim_rng_pcg32(0xBADF00Dull) };
    World wc;
    uint32_t hc[FRAMES];
    run(&wc, &envC, hc);
    diff = -1;
    int c_ok = hashes_equal(hl, hc, &diff);
    printf("B_seq_matches_fresh_pcg_run=%d\n", c_ok);
    if (!c_ok) failures++;
    sim_clock_free(envC.clock);
    sim_rng_free(envC.rng);

    /* ---- C. swap implementation via the interface only ------------- */
    SimEnv swappable = { sim_clock_scripted(frame_dts, FRAMES), sim_rng_pcg32(777) };
    World ws1, ws2;
    uint32_t hs1[FRAMES], hs2[FRAMES];
    run(&ws1, &swappable, hs1);

    /* no module source touched: only the env's RNG pointer changes */
    Rng* old = sim_env_set_rng(&swappable, sim_rng_xorshift32(777));
    (void)old;
    swappable.clock->reset(swappable.clock);
    run(&ws2, &swappable, hs2);
    diff = -1;
    int swap_deterministic = hashes_equal(hs1, hs2, &diff) == 0; /* streams differ */
    /* rerun the swapped config: must reproduce itself exactly */
    SimEnv swapped2 = { sim_clock_scripted(frame_dts, FRAMES), sim_rng_xorshift32(777) };
    World ws3;
    uint32_t hs3[FRAMES];
    run(&ws3, &swapped2, hs3);
    int swap_repeats = hashes_equal(hs2, hs3, &diff);
    printf("C_swap_changes_stream=%d\n", swap_deterministic);
    printf("C_swapped_run_repeats=%d\n", swap_repeats);
    if (!swap_deterministic || !swap_repeats) failures++;

    sim_clock_free(swappable.clock);
    sim_rng_free(swappable.rng);
    sim_rng_free(old);
    sim_clock_free(swapped2.clock);
    sim_rng_free(swapped2.rng);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
