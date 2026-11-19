
/* simenv implementation: see include/simenv/simenv.h for the contract. */
#define _POSIX_C_SOURCE 199309L   /* clock_gettime under -std=c11 */
#include "simenv/simenv.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ============================================================ ambient */

static Clock* default_clock;
static Rng*   default_rng;
static SimEnv default_env;

SimEnv* sim_ambient(void) {
    if (!default_env.clock) {
        if (!default_clock) default_clock = sim_clock_realtime();
        if (!default_rng) {
            /* seed mixes time + address so two fresh processes differ */
            uint64_t seed = (uint64_t)time(0);
            seed ^= (uint64_t)(uintptr_t)&default_env;
            seed = seed * 6364136223846793005ull + 1442695040888963407ull;
            default_rng = sim_rng_pcg32(seed);
        }
        default_env.clock = default_clock;
        default_env.rng = default_rng;
    }
    return &default_env;
}

void sim_set_ambient(SimEnv* env) {
    default_env.clock = env ? env->clock : 0;
    default_env.rng   = env ? env->rng : 0;
}

Clock* sim_env_set_clock(SimEnv* env, Clock* clock) {
    Clock* old = env->clock;
    env->clock = clock;
    return old;
}

Rng* sim_env_set_rng(SimEnv* env, Rng* rng) {
    Rng* old = env->rng;
    env->rng = rng;
    return old;
}

/* ============================================================ clocks */

/* ---- realtime ------------------------------------------------------ */

typedef struct {
    Clock vtable;
    struct timespec origin;
    struct timespec last;
} RealtimeClock;

static double ts_diff_sec(const struct timespec* a, const struct timespec* b) {
    return (double)(a->tv_sec - b->tv_sec)
         + (double)(a->tv_nsec - b->tv_nsec) / 1e9;
}

static double rt_now(Clock* self) {
    RealtimeClock* c = (RealtimeClock*)self;
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return ts_diff_sec(&t, &c->origin);
}

static double rt_tick(Clock* self) {
    RealtimeClock* c = (RealtimeClock*)self;
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    double dt = ts_diff_sec(&t, &c->last);
    c->last = t;
    return dt;
}

static void rt_reset(Clock* self) {
    RealtimeClock* c = (RealtimeClock*)self;
    clock_gettime(CLOCK_MONOTONIC, &c->origin);
    c->last = c->origin;
}

Clock* sim_clock_realtime(void) {
    RealtimeClock* c = (RealtimeClock*)calloc(1, sizeof(*c));
    if (!c) return 0;
    c->vtable.now = rt_now;
    c->vtable.tick = rt_tick;
    c->vtable.reset = rt_reset;
    rt_reset((Clock*)c);
    return (Clock*)c;
}

/* ---- scripted ------------------------------------------------------ */

typedef struct {
    Clock vtable;
    const double* dts;
    int n, pos;
    double t;
} ScriptedClock;

static double sc_now(Clock* self) {
    return ((ScriptedClock*)self)->t;
}

static double sc_tick(Clock* self) {
    ScriptedClock* c = (ScriptedClock*)self;
    double dt = c->n > 0 ? c->dts[c->pos < c->n ? c->pos : c->n - 1] : 0.0;
    if (c->pos < c->n) c->pos++;
    c->t += dt;
    return dt;
}

static void sc_reset(Clock* self) {
    ScriptedClock* c = (ScriptedClock*)self;
    c->pos = 0;
    c->t = 0.0;
}

Clock* sim_clock_scripted(const double* dts, int n) {
    ScriptedClock* c = (ScriptedClock*)calloc(1, sizeof(*c));
    if (!c) return 0;
    c->vtable.now = sc_now;
    c->vtable.tick = sc_tick;
    c->vtable.reset = sc_reset;
    c->dts = dts;
    c->n = n;
    return (Clock*)c;
}

void sim_clock_free(Clock* clock) { free(clock); }

/* ============================================================ rngs */

/* ---- PCG32 (PCG-XSH-RR), reference stream --------------------------- */

typedef struct {
    Rng vtable;
    uint64_t state;
    uint64_t inc;
    uint64_t init_state;
    uint64_t init_inc;
} Pcg32;

static uint32_t pcg_step(Pcg32* p) {
    uint64_t old = p->state;
    p->state = old * 6364136223846793005ull + p->inc;
    uint32_t xors = (uint32_t)(((old >> 18u) ^ old) >> 27u);
    uint32_t rot = (uint32_t)(old >> 59u);
    return (xors >> rot) | (xors << ((-(int32_t)rot) & 31));
}

static uint32_t pcg_next(Rng* self) {
    return pcg_step((Pcg32*)self);
}

static void pcg_fill_f32(Rng* self, float* out, int n) {
    Pcg32* p = (Pcg32*)self;
    for (int i = 0; i < n; i++)
        out[i] = (float)(pcg_step(p) >> 8) * (1.0f / 16777216.0f);
}

static void pcg_reset(Rng* self) {
    Pcg32* p = (Pcg32*)self;
    p->state = p->init_state;
    p->inc = p->init_inc;
}

static uint32_t pcg_peek(Rng* self) {
    Pcg32 snapshot = *(Pcg32*)self;
    return pcg_next((Rng*)&snapshot);
}

Rng* sim_rng_pcg32(uint64_t seed) {
    Pcg32* p = (Pcg32*)calloc(1, sizeof(*p));
    if (!p) return 0;
    p->vtable.next_u32 = pcg_next;
    p->vtable.peek_u32 = pcg_peek;
    p->vtable.reset = pcg_reset;
    p->vtable.fill_f32 = pcg_fill_f32;
    /* standard PCG seeding: odd increment, two warmup steps */
    p->inc = (seed << 1u) | 1u;
    p->state = 0;
    pcg_next((Rng*)p);
    p->state += seed;
    pcg_next((Rng*)p);
    p->init_state = p->state;
    p->init_inc = p->inc;
    return (Rng*)p;
}

/* ---- xorshift32, alternative implementation ------------------------ */

typedef struct {
    Rng vtable;
    uint32_t state;
    uint32_t init_state;
} Xorshift32;

static uint32_t xs_step(Xorshift32* x) {
    uint32_t s = x->state;
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    x->state = s;
    return s;
}

static uint32_t xs_next(Rng* self) {
    return xs_step((Xorshift32*)self);
}

static void xs_fill_f32(Rng* self, float* out, int n) {
    Xorshift32* x = (Xorshift32*)self;
    for (int i = 0; i < n; i++)
        out[i] = (float)(xs_step(x) >> 8) * (1.0f / 16777216.0f);
}

static void xs_reset(Rng* self) {
    ((Xorshift32*)self)->state = ((Xorshift32*)self)->init_state;
}

static uint32_t xs_peek(Rng* self) {
    Xorshift32 snapshot = *(Xorshift32*)self;
    return xs_next((Rng*)&snapshot);
}

Rng* sim_rng_xorshift32(uint32_t seed) {
    Xorshift32* x = (Xorshift32*)calloc(1, sizeof(*x));
    if (!x) return 0;
    x->vtable.next_u32 = xs_next;
    x->vtable.peek_u32 = xs_peek;
    x->vtable.reset = xs_reset;
    x->vtable.fill_f32 = xs_fill_f32;
    x->state = x->init_state = seed ? seed : 0x9e3779b9u;
    return (Rng*)x;
}

/* ---- fixed sequence ------------------------------------------------ */

typedef struct {
    Rng vtable;
    const uint32_t* values;
    int n, pos;
} SeqRng;

static uint32_t seq_next(Rng* self) {
    SeqRng* q = (SeqRng*)self;
    if (q->n <= 0) return 0;
    uint32_t v = q->values[q->pos % q->n];
    q->pos++;
    return v;
}

static void seq_fill_f32(Rng* self, float* out, int n) {
    SeqRng* q = (SeqRng*)self;
    for (int i = 0; i < n; i++) {
        uint32_t v = q->n > 0 ? q->values[q->pos % q->n] : 0;
        q->pos++;
        out[i] = (float)(v >> 8) * (1.0f / 16777216.0f);
    }
}

static uint32_t seq_peek(Rng* self) {
    SeqRng* q = (SeqRng*)self;
    return q->n > 0 ? q->values[q->pos % q->n] : 0;
}

static void seq_reset(Rng* self) {
    ((SeqRng*)self)->pos = 0;
}

Rng* sim_rng_seq(const uint32_t* values, int n) {
    SeqRng* q = (SeqRng*)calloc(1, sizeof(*q));
    if (!q) return 0;
    q->vtable.next_u32 = seq_next;
    q->vtable.peek_u32 = seq_peek;
    q->vtable.reset = seq_reset;
    q->vtable.fill_f32 = seq_fill_f32;
    q->values = values;
    q->n = n;
    return (Rng*)q;
}

void sim_rng_free(Rng* rng) { free(rng); }

/* ============================================================ recorders */

#define REC_CAP 1 << 20

typedef struct {
    Rng vtable;
    Rng* inner;
    uint32_t* log;
    int count, cap;
} RngRecorder;

static uint32_t rec_next(Rng* self) {
    RngRecorder* r = (RngRecorder*)self;
    uint32_t v = r->inner->next_u32(r->inner);
    if (r->count < r->cap) r->log[r->count++] = v;
    return v;
}

static void rec_fill_f32(Rng* self, float* out, int n) {
    RngRecorder* r = (RngRecorder*)self;
    for (int i = 0; i < n; i++) {
        uint32_t v = r->inner->next_u32(r->inner);
        if (r->count < r->cap) r->log[r->count++] = v;
        out[i] = (float)(v >> 8) * (1.0f / 16777216.0f);
    }
}

static uint32_t rec_peek(Rng* self) {
    return ((RngRecorder*)self)->inner->peek_u32(((RngRecorder*)self)->inner);
}

static void rec_reset(Rng* self) {
    RngRecorder* r = (RngRecorder*)self;
    r->inner->reset(r->inner);
    r->count = 0;
}

Rng* sim_rng_recorder(Rng* inner) {
    RngRecorder* r = (RngRecorder*)calloc(1, sizeof(*r));
    if (!r) return 0;
    r->cap = REC_CAP;
    r->log = (uint32_t*)malloc(sizeof(uint32_t) * (size_t)r->cap);
    if (!r->log) { free(r); return 0; }
    r->vtable.next_u32 = rec_next;
    r->vtable.peek_u32 = rec_peek;
    r->vtable.reset = rec_reset;
    r->vtable.fill_f32 = rec_fill_f32;
    r->inner = inner;
    return (Rng*)r;
}

const uint32_t* sim_recorder_values(Rng* recorder, int* out_n) {
    RngRecorder* r = (RngRecorder*)recorder;
    if (out_n) *out_n = r->count;
    return r->log;
}

void sim_recorder_reset(Rng* recorder) { rec_reset(recorder); }

void sim_recorder_free(Rng* recorder) {
    RngRecorder* r = (RngRecorder*)recorder;
    if (!r) return;
    free(r->log);
    free(r);
}

typedef struct {
    Clock vtable;
    Clock* inner;
    double* log;
    int count, cap;
} ClockRecorder;

static double crec_now(Clock* self) {
    ClockRecorder* c = (ClockRecorder*)self;
    return c->inner->now(c->inner);
}

static double crec_tick(Clock* self) {
    ClockRecorder* c = (ClockRecorder*)self;
    double dt = c->inner->tick(c->inner);
    if (c->count < c->cap) c->log[c->count++] = dt;
    return dt;
}

static void crec_reset(Clock* self) {
    ClockRecorder* c = (ClockRecorder*)self;
    c->inner->reset(c->inner);
    c->count = 0;
}

Clock* sim_clock_recorder(Clock* inner) {
    ClockRecorder* c = (ClockRecorder*)calloc(1, sizeof(*c));
    if (!c) return 0;
    c->cap = REC_CAP;
    c->log = (double*)malloc(sizeof(double) * (size_t)c->cap);
    if (!c->log) { free(c); return 0; }
    c->vtable.now = crec_now;
    c->vtable.tick = crec_tick;
    c->vtable.reset = crec_reset;
    c->inner = inner;
    return (Clock*)c;
}

const double* sim_clock_recorder_dts(Clock* recorder, int* out_n) {
    ClockRecorder* c = (ClockRecorder*)recorder;
    if (out_n) *out_n = c->count;
    return c->log;
}

void sim_clock_recorder_reset(Clock* recorder) { crec_reset(recorder); }

void sim_clock_recorder_free(Clock* recorder) {
    ClockRecorder* c = (ClockRecorder*)recorder;
    if (!c) return;
    free(c->log);
    free(c);
}

/* ============================================================ tape io */

/* all multi-byte fields are written little-endian, fixed width */
static void put_u32(FILE* f, uint32_t v) {
    unsigned char b[4] = {
        (unsigned char)(v), (unsigned char)(v >> 8),
        (unsigned char)(v >> 16), (unsigned char)(v >> 24)
    };
    fwrite(b, 1, 4, f);
}

static void put_u64(FILE* f, uint64_t v) {
    for (int i = 0; i < 8; i++) {
        unsigned char b = (unsigned char)(v >> (8 * i));
        fwrite(&b, 1, 1, f);
    }
}

static uint32_t get_u32(FILE* f) {
    unsigned char b[4];
    if (fread(b, 1, 4, f) != 4) return 0;
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8)
         | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

static uint64_t get_u64(FILE* f) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) {
        unsigned char b;
        if (fread(&b, 1, 1, f) != 1) return 0;
        v |= (uint64_t)b << (8 * i);
    }
    return v;
}

int sim_tape_save(const SimTape* tape, const char* path) {
    FILE* f = fopen(path, "wb");
    if (!f) return -1;
    put_u32(f, SIM_TAPE_MAGIC);
    put_u32(f, SIM_TAPE_VERSION);
    put_u64(f, tape->seed);
    put_u32(f, (uint32_t)tape->n_dts);
    put_u32(f, (uint32_t)tape->n_rand);
    fwrite(tape->dts, sizeof(double), (size_t)tape->n_dts, f);
    for (int i = 0; i < tape->n_rand; i++) put_u32(f, tape->rand[i]);
    int ok = ferror(f) ? -1 : 0;
    fclose(f);
    return ok;
}

int sim_tape_load(SimTape* tape, const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return -1;
    memset(tape, 0, sizeof(*tape));
    tape->magic = get_u32(f);
    tape->version = get_u32(f);
    if (tape->magic != SIM_TAPE_MAGIC || tape->version != SIM_TAPE_VERSION) {
        fclose(f);
        return -1;
    }
    tape->seed = get_u64(f);
    tape->n_dts = (int)get_u32(f);
    tape->n_rand = (int)get_u32(f);
    if (tape->n_dts < 0 || tape->n_rand < 0 ||
        tape->n_dts > REC_CAP || tape->n_rand > REC_CAP) {
        fclose(f);
        return -1;
    }
    tape->dts = (double*)malloc(sizeof(double) * (size_t)(tape->n_dts + 1));
    tape->rand = (uint32_t*)malloc(sizeof(uint32_t) * (size_t)(tape->n_rand + 1));
    if (!tape->dts || !tape->rand) {
        free(tape->dts); free(tape->rand);
        fclose(f);
        return -1;
    }
    if (fread(tape->dts, sizeof(double), (size_t)tape->n_dts, f) != (size_t)tape->n_dts ||
        ferror(f)) {
        sim_tape_free(tape);
        fclose(f);
        return -1;
    }
    for (int i = 0; i < tape->n_rand; i++) tape->rand[i] = get_u32(f);
    fclose(f);
    return 0;
}

void sim_tape_free(SimTape* tape) {
    free(tape->dts);
    free(tape->rand);
    memset(tape, 0, sizeof(*tape));
}

int sim_tape_env(const SimTape* tape, SimEnv* env, Clock** out_clock, Rng** out_rng) {
    Clock* clock = sim_clock_scripted(tape->dts, tape->n_dts);
    Rng* rng = sim_rng_seq(tape->rand, tape->n_rand);
    if (!clock || !rng) {
        sim_clock_free(clock);
        sim_rng_free(rng);
        return -1;
    }
    env->clock = clock;
    env->rng = rng;
    if (out_clock) *out_clock = clock;
    if (out_rng) *out_rng = rng;
    return 0;
}
