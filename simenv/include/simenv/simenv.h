
/* simenv: injectable time source and randomness source.
   Every module that needs time or randomness goes through this header only:
     - time:  sim_now(), sim_dt()
     - random: sim_randu32(), sim_randf(), sim_rand_range()
   The active implementations are reached through a SimEnv (function tables).
   Swapping the implementation never touches module sources: either replace the
   tables on one SimEnv (sim_env_set_*), or change the ambient default
   (sim_set_ambient). All wrappers are static inline so the call cost is one
   indirect call with no extra frame. */
#ifndef SIMENV_SIMENV_H
#define SIMENV_SIMENV_H

#include <stdint.h>

/* ------------------------------------------------------------------ clock */

typedef struct Clock Clock;
struct Clock {
    double (*now)(Clock* self);   /* seconds since an arbitrary origin */
    double (*tick)(Clock* self);  /* advance one frame, return its dt */
    void   (*reset)(Clock* self);
};

/* wall clock: now() reads CLOCK_MONOTONIC, tick() measures the real delta */
Clock* sim_clock_realtime(void);

/* scripted clock: advances along a fixed list of per-frame deltas.
   Deterministic; exhausting the list keeps repeating the final delta. */
Clock* sim_clock_scripted(const double* dts, int n);
void   sim_clock_free(Clock* clock);

/* --------------------------------------------------------------- rng impls */

typedef struct Rng Rng;
struct Rng {
    uint32_t (*next_u32)(Rng* self);
    uint32_t (*peek_u32)(Rng* self);   /* value the next next_u32 would return */
    void     (*reset)(Rng* self);      /* back to construction seed/state */
    /* bulk path: n uniforms in [0,1) through ONE indirect call; the draw loop
       runs inside the implementation with direct calls, so a consumer that
       needs several draws per event pays dispatch once, not per draw */
    void     (*fill_f32)(Rng* self, float* out, int n);
};

/* PCG32 (PCG-XSH-RR), good default PRNG, fully specified and portable. */
Rng* sim_rng_pcg32(uint64_t seed);

/* xorshift32: second implementation proving the interface is replaceable. */
Rng* sim_rng_xorshift32(uint32_t seed);

/* sequence RNG: replays an exact recorded uint32 sequence, then wraps.
   Used to re-run a past simulation frame-for-frame without the original RNG. */
Rng* sim_rng_seq(const uint32_t* values, int n);
void sim_rng_free(Rng* rng);

/* ------------------------------------------------------------- sim env v1 */

typedef struct SimEnv {
    Clock* clock;
    Rng*   rng;
} SimEnv;

/* ambient default env, lazily initialised to realtime clock + PCG32. */
SimEnv* sim_ambient(void);
void    sim_set_ambient(SimEnv* env);

/* swap one source on an env; returns the previous source. */
Clock* sim_env_set_clock(SimEnv* env, Clock* clock);
Rng*   sim_env_set_rng(SimEnv* env, Rng* rng);

/* ------------------------------------------------------------- tape v1 */

/* A tape freezes one run: the clock's frame deltas followed by every u32 the
   RNG produced. Replaying reconstructs an identical simulation without the
   original clock/RNG implementations (realtime clock -> scripted, any RNG ->
   seq). File layout is little-endian fixed-width, same-platform portable. */

#define SIM_TAPE_MAGIC   0x45504d53u   /* "SMPE" */
#define SIM_TAPE_VERSION 1u

typedef struct SimTape {
    uint32_t magic, version;
    uint64_t seed;
    int      n_dts;
    int      n_rand;
    double*  dts;
    uint32_t* rand;
} SimTape;

/* RNG decorator: records every value the inner RNG emits. */
Rng* sim_rng_recorder(Rng* inner);
const uint32_t* sim_recorder_values(Rng* recorder, int* out_n);
void sim_recorder_reset(Rng* recorder);
/* frees the recorder and its log; the borrowed inner RNG is not freed */
void sim_recorder_free(Rng* recorder);

/* clock decorator: records every dt tick() returns. */
Clock* sim_clock_recorder(Clock* inner);
const double* sim_clock_recorder_dts(Clock* recorder, int* out_n);
void sim_clock_recorder_reset(Clock* recorder);
/* frees the recorder and its log; the borrowed inner clock is not freed */
void sim_clock_recorder_free(Clock* recorder);

int  sim_tape_save(const SimTape* tape, const char* path);
int  sim_tape_load(SimTape* tape, const char* path);   /* allocates buffers */
void sim_tape_free(SimTape* tape);
/* build scripted+seq sources from a loaded tape and bind them onto env.
   The sources are owned by the caller; free with sim_clock_free/sim_rng_free. */
int  sim_tape_env(const SimTape* tape, SimEnv* env, Clock** out_clock, Rng** out_rng);

/* ------------------------------------------------------ inline accessors */
/* These are the only symbols modules should call. `env == NULL` means the
   ambient default, so a module can stay source-identical in standalone and
   injected builds. */

static inline double sim_env_now(SimEnv* env) {
    SimEnv* e = env ? env : sim_ambient();
    return e->clock->now(e->clock);
}

static inline double sim_env_dt(SimEnv* env) {
    SimEnv* e = env ? env : sim_ambient();
    return e->clock->tick(e->clock);
}

static inline uint32_t sim_env_u32(SimEnv* env) {
    SimEnv* e = env ? env : sim_ambient();
    return e->rng->next_u32(e->rng);
}

static inline uint32_t sim_env_peek(SimEnv* env) {
    SimEnv* e = env ? env : sim_ambient();
    return e->rng->peek_u32(e->rng);
}

/* uniform float in [0, 1), exact division-by-2^24, identical on every RNG */
static inline float sim_env_randf(SimEnv* env) {
    return (float)(sim_env_u32(env) >> 8) * (1.0f / 16777216.0f);
}

/* bulk uniforms: one dispatch for the whole batch, same values and order as
   n scalar sim_env_randf calls */
static inline void sim_env_randf_n(SimEnv* env, float* out, int n) {
    SimEnv* e = env ? env : sim_ambient();
    e->rng->fill_f32(e->rng, out, n);
}

/* uniform integer in [lo, hi] inclusive; hi must be >= lo */
static inline int sim_env_range(SimEnv* env, int lo, int hi) {
    uint32_t span = (uint32_t)(hi - lo + 1);
    return lo + (int)(sim_env_u32(env) % span);
}

/* ambient shorthands for modules that keep no SimEnv pointer */
static inline double sim_now(void)   { return sim_env_now(0); }
static inline double sim_dt(void)    { return sim_env_dt(0); }
static inline uint32_t sim_randu32(void) { return sim_env_u32(0); }
static inline float  sim_randf(void) { return sim_env_randf(0); }
static inline int    sim_rand_range(int lo, int hi) { return sim_env_range(0, lo, hi); }

#endif /* SIMENV_SIMENV_H */
