
/* simenv self-test: determinism, replaceability, record/tape/replay. */
#include "simenv/simenv.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    /* RNG determinism: same seed, reset -> identical streams */
    Rng* a = sim_rng_pcg32(42);
    Rng* b = sim_rng_pcg32(42);
    int same = 1;
    for (int i = 0; i < 1000; i++)
        if (a->next_u32(a) != b->next_u32(b)) same = 0;
    check("pcg_seed_deterministic", same);

    a->reset(a);
    uint32_t first = a->next_u32(a);
    a->reset(a);
    check("rng_reset_restarts", a->next_u32(a) == first);
    check("rng_peek_preserves", a->peek_u32(a) == a->next_u32(a));

    /* different seeds must differ; different implementations must differ too */
    Rng* other = sim_rng_pcg32(43);
    check("different_seed_differs", other->next_u32(other) != first);
    Rng* xs = sim_rng_xorshift32(42);
    int differ = 0;
    for (int i = 0; i < 100; i++)
        if (xs->next_u32(xs) != a->next_u32(a)) differ = 1;
    check("xorshift_distinct_stream", differ);
    xs->reset(xs);
    uint32_t xs_first = xs->next_u32(xs);
    xs->reset(xs);
    check("xorshift_deterministic", xs->next_u32(xs) == xs_first);

    /* float helper range and range helper bounds */
    SimEnv env = { sim_clock_realtime(), sim_rng_pcg32(7) };
    int float_ok = 1, range_ok = 1;
    for (int i = 0; i < 10000; i++) {
        float f = sim_env_randf(&env);
        if (!(f >= 0.0f && f < 1.0f)) float_ok = 0;
        int r = sim_env_range(&env, -3, 9);
        if (r < -3 || r > 9) range_ok = 0;
    }
    check("randf_in_unit_halfopen", float_ok);
    check("rand_range_bounds", range_ok);

    /* batch fill must match scalar draws bit-for-bit, including the recorder
       path used by tapes */
    Rng* ba = sim_rng_pcg32(2024);
    Rng* bb = sim_rng_pcg32(2024);
    int batch_ok = 1;
    for (int round = 0; round < 50; round++) {
        int n = 1 + (round % 7);           /* no extra RNG draw: streams stay aligned */
        float bulk[8], scalar[8];
        ba->fill_f32(ba, bulk, n);
        for (int i = 0; i < n; i++)
            scalar[i] = (float)(bb->next_u32(bb) >> 8) * (1.0f / 16777216.0f);
        if (memcmp(bulk, scalar, sizeof(float) * (size_t)n) != 0) batch_ok = 0;
    }
    check("fill_matches_scalar", batch_ok);

    Rng* rsrc = sim_rng_pcg32(55);
    Rng* rrec = sim_rng_recorder(rsrc);
    float rf[10];
    rrec->fill_f32(rrec, rf, 10);
    int rn = 0;
    const uint32_t* rlog = sim_recorder_values(rrec, &rn);
    int rec_batch_ok = (rn == 10);
    for (int i = 0; i < 10 && rec_batch_ok; i++)
        if (rf[i] != (float)(rlog[i] >> 8) * (1.0f / 16777216.0f)) rec_batch_ok = 0;
    check("recorder_fill_logged", rec_batch_ok);
    sim_recorder_free(rrec);
    sim_rng_free(rsrc);
    sim_rng_free(ba);
    sim_rng_free(bb);

    /* scripted clock: exact dt sequence, now() accumulates */
    double dts[] = { 0.1, 0.2, 0.3, 0.25 };
    Clock* sc = sim_clock_scripted(dts, 4);
    check("scripted_dt_1", fabs(sc->tick(sc) - 0.1) < 1e-12);
    check("scripted_dt_2", fabs(sc->tick(sc) - 0.2) < 1e-12);
    check("scripted_now", fabs(sc->now(sc) - 0.3) < 1e-12);
    check("scripted_dt_3", fabs(sc->tick(sc) - 0.3) < 1e-12);
    sc->reset(sc);
    check("scripted_reset", fabs(sc->tick(sc) - 0.1) < 1e-12 && fabs(sc->now(sc) - 0.1) < 1e-12);

    /* recorder + tape round trip, then reconstruct via seq RNG */
    Rng* src = sim_rng_pcg32(99);
    Rng* rec = sim_rng_recorder(src);
    uint32_t expect[64];
    for (int i = 0; i < 64; i++) expect[i] = rec->next_u32(rec);
    int nlog = 0;
    const uint32_t* logged = sim_recorder_values(rec, &nlog);
    check("recorder_captures_all", nlog == 64 &&
          memcmp(logged, expect, sizeof(expect)) == 0);

    Clock* crec_inner = sim_clock_scripted(dts, 4);
    Clock* crec = sim_clock_recorder(crec_inner);
    for (int i = 0; i < 4; i++) crec->tick(crec);
    int ndt = 0;
    const double* logged_dts = sim_clock_recorder_dts(crec, &ndt);
    check("clock_recorder_captures", ndt == 4 && fabs(logged_dts[2] - 0.3) < 1e-12);

    SimTape tape = {
        SIM_TAPE_MAGIC, SIM_TAPE_VERSION, 99, ndt, nlog,
        (double*)logged_dts, (uint32_t*)logged
    };
    const char* path = "/tmp/simenv_selftest.tape";
    check("tape_save", sim_tape_save(&tape, path) == 0);

    SimTape loaded;
    check("tape_load", sim_tape_load(&loaded, path) == 0);
    check("tape_header", loaded.seed == 99 && loaded.n_dts == 4 && loaded.n_rand == 64);

    SimEnv re = {0};
    Clock* rclock; Rng* rrng;
    check("tape_env", sim_tape_env(&loaded, &re, &rclock, &rrng) == 0);
    int replay_same = 1;
    for (int i = 0; i < 64; i++)
        if (re.rng->next_u32(re.rng) != expect[i]) replay_same = 0;
    check("seq_rng_replays_stream", replay_same);
    check("scripted_clock_replays_dts", fabs(re.clock->tick(re.clock) - 0.1) < 1e-12 &&
          fabs(re.clock->tick(re.clock) - 0.2) < 1e-12);

    /* tamper detection */
    SimTape bad;
    check("tape_rejects_garbage", sim_tape_load(&bad, "/dev/null") != 0);

    /* swap on an env without touching call sites */
    SimEnv sw = { sim_clock_scripted(dts, 4), sim_rng_pcg32(1) };
    uint32_t before = sim_env_u32(&sw);
    Rng* old = sim_env_set_rng(&sw, sim_rng_xorshift32(1));
    uint32_t after = sim_env_u32(&sw);
    check("env_swap_rng", before != after && old != 0);

    /* release every heap allocation so LSan stays clean */
    sim_tape_free(&loaded);
    sim_rng_free(a); sim_rng_free(b); sim_rng_free(other); sim_rng_free(xs);
    sim_clock_free(env.clock);
    sim_rng_free(env.rng);
    sim_clock_free(sc);
    sim_rng_free(src);
    sim_recorder_free(rec);
    sim_clock_recorder_free(crec);
    sim_clock_free(crec_inner);
    sim_clock_free(rclock);
    sim_rng_free(rrng);
    sim_clock_free(sw.clock);
    sim_rng_free(sw.rng);
    sim_rng_free(old);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
