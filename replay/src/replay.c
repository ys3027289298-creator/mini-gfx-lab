/* replay: record a deterministic input stream, play it back in order, and
   diff the simulated state against the recording frame by frame.

   Log layout (all integers little-endian):
     header 24B: magic "MGLR", version, flags, sim seed, frame count,
                 input record size, state record size, reserved
     per frame: input record (12B) [+ state snapshot (40B) when the
                snapshot flag is set]

   Both record kinds have a fixed size, so the log is exactly
     24 + frames * 52 bytes      (with snapshots)
     24 + frames * 12 bytes      (inputs only)
   i.e. strictly linear in duration. The simulation uses Q16.16 fixed point
   only, so a replay is bit-for-bit reproducible. */
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* deterministic simulation                                           */
/* ------------------------------------------------------------------ */

#define FIX_ONE 65536
typedef int32_t fix;

enum {
    BTN_LEFT   = 1u << 0,
    BTN_RIGHT  = 1u << 1,
    BTN_UP     = 1u << 2,
    BTN_DOWN   = 1u << 3,
    BTN_ATTACK = 1u << 4
};

typedef struct {
    uint32_t frame;
    uint16_t buttons;
    int16_t ax, ay;          /* analog axes, -256..256 */
} Input;

typedef struct {
    uint32_t frame;
    fix x, y;                /* position, world units */
    fix vx, vy;              /* velocity, units/frame */
    int32_t hp;
    uint32_t rng;
    int32_t combo;
    uint64_t score;
} SimState;

static uint32_t rng_next(uint32_t* s) {
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

static fix fix_mul(fix a, fix b) {
    return (fix)(((int64_t)a * (int64_t)b) >> 16);
}

static int32_t axis_clamp(int32_t v) {
    return v < -256 ? -256 : v > 256 ? 256 : v;
}

static void sim_init(SimState* s, uint32_t seed) {
    memset(s, 0, sizeof(*s));
    s->x = 2 * FIX_ONE;
    s->y = 2 * FIX_ONE;
    s->hp = 100;
    s->rng = seed ? seed : 0x1234567u;
}

static void sim_step(SimState* s, const Input* in) {
    const fix accel = FIX_ONE / 4;
    const fix vmax = 4 * FIX_ONE;
    int32_t ix = in->ax + ((in->buttons & BTN_RIGHT) ? 256 : 0)
               - ((in->buttons & BTN_LEFT) ? 256 : 0);
    int32_t iy = in->ay + ((in->buttons & BTN_DOWN) ? 256 : 0)
               - ((in->buttons & BTN_UP) ? 256 : 0);
    ix = axis_clamp(ix);
    iy = axis_clamp(iy);

    s->vx += (fix)(((int64_t)ix * accel) >> 8);
    s->vy += (fix)(((int64_t)iy * accel) >> 8);
    if (ix == 0) s->vx = fix_mul(s->vx, (fix)(((int64_t)FIX_ONE * 94) / 100));
    if (iy == 0) s->vy = fix_mul(s->vy, (fix)(((int64_t)FIX_ONE * 94) / 100));
    if (s->vx > vmax) s->vx = vmax;
    else if (s->vx < -vmax) s->vx = -vmax;
    if (s->vy > vmax) s->vy = vmax;
    else if (s->vy < -vmax) s->vy = -vmax;

    /* The RNG is advanced exactly once per frame before any branch, so the
       random stream is independent of the inputs. */
    uint32_t r = rng_next(&s->rng);
    if (in->buttons & BTN_ATTACK) {
        s->combo += 1;
        s->score += (uint64_t)(r % 13u) + (uint64_t)(s->combo * 3);
    } else {
        s->combo = 0;
    }
    if ((r & 0xffu) == 0u && s->hp > 0) s->hp -= 1;

    s->x += s->vx;
    s->y += s->vy;
    s->frame += 1;
}

/* ------------------------------------------------------------------ */
/* deterministic input generation (test driver)                       */
/* ------------------------------------------------------------------ */

typedef struct { uint32_t r; } InputGen;

static void input_gen_init(InputGen* g, uint32_t seed) {
    g->r = seed ? seed : 1u;
}

static Input input_gen_next(InputGen* g, uint32_t frame) {
    Input in;
    g->r = g->r * 1664525u + 1013904223u;
    in.frame = frame;
    in.buttons = 0;
    if (((g->r >> 28) & 3u) == 0u) in.buttons |= BTN_ATTACK;
    if (((g->r >> 20) & 7u) == 0u) in.buttons |= BTN_LEFT;
    if (((g->r >> 16) & 7u) == 0u) in.buttons |= BTN_RIGHT;
    in.ax = (int16_t)((int32_t)((g->r >> 8) % 513u) - 256);
    in.ay = (int16_t)((int32_t)((g->r >> 18) % 257u) - 128);
    return in;
}

/* ------------------------------------------------------------------ */
/* log serialization                                                  */
/* ------------------------------------------------------------------ */

#define LOG_MAGIC           0x4d474c52u   /* "MGLR" */
#define LOG_VERSION         1u
#define LOG_FLAG_SNAPSHOTS  1u

#define LOG_HEADER_BYTES    24u
#define LOG_INPUT_BYTES     12u
#define LOG_STATE_BYTES     40u
#define LOG_FRAME_BYTES     (LOG_INPUT_BYTES + LOG_STATE_BYTES)

static void put_u16le(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xffu);
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32le(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xffu);
    p[1] = (uint8_t)((v >> 8) & 0xffu);
    p[2] = (uint8_t)((v >> 16) & 0xffu);
    p[3] = (uint8_t)((v >> 24) & 0xffu);
}

static void put_u64le(uint8_t* p, uint64_t v) {
    for (int i = 0; i < 8; i++) p[i] = (uint8_t)((v >> (8 * i)) & 0xffu);
}

static uint16_t get_u16le(const uint8_t* p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32le(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t get_u64le(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v |= (uint64_t)p[i] << (8 * i);
    return v;
}

static void put_i16le(uint8_t* p, int16_t v) { put_u16le(p, (uint16_t)v); }
static void put_i32le(uint8_t* p, int32_t v) { put_u32le(p, (uint32_t)v); }

static int16_t get_i16le(const uint8_t* p) {
    uint16_t u = get_u16le(p);
    return u >= 0x8000u ? (int16_t)((int32_t)u - 65536) : (int16_t)u;
}

static int32_t get_i32le(const uint8_t* p) {
    return (int32_t)get_u32le(p);
}

static void encode_input(uint8_t* p, const Input* in) {
    put_u32le(p + 0, in->frame);
    put_u16le(p + 4, in->buttons);
    put_i16le(p + 6, in->ax);
    put_i16le(p + 8, in->ay);
    put_u16le(p + 10, 0);                /* reserved */
}

static void decode_input(const uint8_t* p, Input* in) {
    in->frame = get_u32le(p + 0);
    in->buttons = get_u16le(p + 4);
    in->ax = get_i16le(p + 6);
    in->ay = get_i16le(p + 8);
}

static void encode_state(uint8_t* p, const SimState* s) {
    put_u32le(p + 0, s->frame);
    put_i32le(p + 4, s->x);
    put_i32le(p + 8, s->y);
    put_i32le(p + 12, s->vx);
    put_i32le(p + 16, s->vy);
    put_i32le(p + 20, s->hp);
    put_u32le(p + 24, s->rng);
    put_i32le(p + 28, s->combo);
    put_u64le(p + 32, s->score);
}

static void decode_state(const uint8_t* p, SimState* s) {
    s->frame = get_u32le(p + 0);
    s->x = (fix)get_i32le(p + 4);
    s->y = (fix)get_i32le(p + 8);
    s->vx = (fix)get_i32le(p + 12);
    s->vy = (fix)get_i32le(p + 16);
    s->hp = get_i32le(p + 20);
    s->rng = get_u32le(p + 24);
    s->combo = get_i32le(p + 28);
    s->score = get_u64le(p + 32);
}

static void encode_header(uint8_t* p, uint32_t seed, uint32_t frames, uint16_t flags) {
    put_u32le(p + 0, LOG_MAGIC);
    put_u16le(p + 4, (uint16_t)LOG_VERSION);
    put_u16le(p + 6, flags);
    put_u32le(p + 8, seed);
    put_u32le(p + 12, frames);
    put_u16le(p + 16, (uint16_t)LOG_INPUT_BYTES);
    put_u16le(p + 18, (uint16_t)LOG_STATE_BYTES);
    put_u32le(p + 20, 0);                /* reserved */
}

static int decode_header(const uint8_t* p, uint32_t* seed, uint32_t* frames,
                         uint16_t* flags) {
    if (get_u32le(p + 0) != LOG_MAGIC) return -1;
    if (get_u16le(p + 4) != LOG_VERSION) return -1;
    *flags = get_u16le(p + 6);
    if (*flags & ~LOG_FLAG_SNAPSHOTS) return -1;
    if (get_u16le(p + 16) != LOG_INPUT_BYTES) return -1;
    if (get_u16le(p + 18) != LOG_STATE_BYTES) return -1;
    *seed = get_u32le(p + 8);
    *frames = get_u32le(p + 12);
    return 0;
}

typedef struct {
    FILE* f;
    uint32_t seed;
    uint32_t frames;
    uint16_t flags;
} LogWriter;

typedef struct {
    FILE* f;
    uint32_t seed;
    uint32_t frames;
    uint32_t next;
    uint16_t flags;
} LogReader;

static int log_open(LogWriter* w, const char* path, uint32_t seed, int snapshots) {
    uint8_t h[LOG_HEADER_BYTES];
    memset(w, 0, sizeof(*w));
    w->f = fopen(path, "wb");
    if (!w->f) return -1;
    w->seed = seed;
    w->flags = snapshots ? LOG_FLAG_SNAPSHOTS : 0;
    encode_header(h, seed, 0, w->flags);
    if (fwrite(h, 1, sizeof(h), w->f) != sizeof(h)) {
        fclose(w->f);
        w->f = NULL;
        return -1;
    }
    return 0;
}

static int log_append(LogWriter* w, const Input* in, const SimState* s) {
    uint8_t buf[LOG_FRAME_BYTES];
    size_t n = LOG_INPUT_BYTES;
    if (!w->f) return -1;
    encode_input(buf, in);
    if (w->flags & LOG_FLAG_SNAPSHOTS) {
        if (!s) return -1;
        encode_state(buf + LOG_INPUT_BYTES, s);
        n += LOG_STATE_BYTES;
    }
    if (fwrite(buf, 1, n, w->f) != n) return -1;
    w->frames++;
    return 0;
}

static int log_close(LogWriter* w) {
    uint8_t h[LOG_HEADER_BYTES];
    int ok = 1;
    if (!w->f) return -1;
    encode_header(h, w->seed, w->frames, w->flags);
    if (fseek(w->f, 0, SEEK_SET) != 0 ||
        fwrite(h, 1, sizeof(h), w->f) != sizeof(h)) ok = 0;
    if (fclose(w->f) != 0) ok = 0;
    w->f = NULL;
    return ok ? 0 : -1;
}

static int log_reader_open(LogReader* r, const char* path) {
    uint8_t h[LOG_HEADER_BYTES];
    memset(r, 0, sizeof(*r));
    r->f = fopen(path, "rb");
    if (!r->f) return -1;
    if (fread(h, 1, sizeof(h), r->f) != sizeof(h) ||
        decode_header(h, &r->seed, &r->frames, &r->flags) != 0) {
        fclose(r->f);
        r->f = NULL;
        return -1;
    }
    return 0;
}

/* 1 = record, 0 = end of log, -1 = truncated/corrupt */
static int log_reader_next(LogReader* r, Input* in, SimState* s) {
    uint8_t buf[LOG_FRAME_BYTES];
    size_t n;
    if (!r->f) return -1;
    if (r->next >= r->frames) return 0;
    n = (r->flags & LOG_FLAG_SNAPSHOTS) ? LOG_FRAME_BYTES : LOG_INPUT_BYTES;
    if (fread(buf, 1, n, r->f) != n) return -1;
    decode_input(buf, in);
    if ((r->flags & LOG_FLAG_SNAPSHOTS) && s) decode_state(buf + LOG_INPUT_BYTES, s);
    r->next++;
    return 1;
}

static void log_reader_close(LogReader* r) {
    if (r->f) fclose(r->f);
    r->f = NULL;
}

/* ------------------------------------------------------------------ */
/* replay + per-frame diff                                            */
/* ------------------------------------------------------------------ */

typedef enum { VAL_DEC, VAL_HEX, VAL_FIX } FieldKind;

typedef struct {
    const char* field;
    int64_t recorded;
    int64_t replayed;
    FieldKind kind;
} FieldDiff;

typedef struct {
    int diverged;
    uint32_t frame;
    const char* field;
    FieldDiff diff;
    uint32_t frames_total;
    uint32_t frames_matched;
    char error[96];
} VerifyResult;

/* Returns 1 and fills the first differing field, or 0 when states match. */
static int state_diff(const SimState* ref, const SimState* got, FieldDiff* d) {
#define DIFF(f, name, k)                                                    \
    do {                                                                    \
        if (ref->f != got->f) {                                             \
            d->field = (name);                                              \
            d->recorded = (int64_t)ref->f;                                  \
            d->replayed = (int64_t)got->f;                                  \
            d->kind = (k);                                                  \
            return 1;                                                       \
        }                                                                   \
    } while (0)
    DIFF(frame, "frame", VAL_DEC);
    DIFF(x, "x", VAL_FIX);
    DIFF(y, "y", VAL_FIX);
    DIFF(vx, "vx", VAL_FIX);
    DIFF(vy, "vy", VAL_FIX);
    DIFF(hp, "hp", VAL_DEC);
    DIFF(rng, "rng", VAL_HEX);
    DIFF(combo, "combo", VAL_DEC);
    DIFF(score, "score", VAL_DEC);
#undef DIFF
    return 0;
}

/* 1 = every frame matched, 0 = diverged, -1 = malformed log. */
static int replay_verify(const char* path, VerifyResult* vr) {
    LogReader r;
    SimState sim;
    int rc = 1;
    memset(vr, 0, sizeof(*vr));
    vr->frame = UINT32_MAX;

    if (log_reader_open(&r, path) != 0) {
        snprintf(vr->error, sizeof(vr->error), "cannot read log header");
        return -1;
    }
    if (!(r.flags & LOG_FLAG_SNAPSHOTS)) {
        snprintf(vr->error, sizeof(vr->error), "log has no state snapshots");
        log_reader_close(&r);
        return -1;
    }
    vr->frames_total = r.frames;
    sim_init(&sim, r.seed);

    for (uint32_t i = 0; i < r.frames; i++) {
        Input in;
        SimState ref;
        int got = log_reader_next(&r, &in, &ref);
        if (got != 1) {
            snprintf(vr->error, sizeof(vr->error),
                     "truncated log at frame %" PRIu32, i);
            rc = -1;
            break;
        }
        if (in.frame != i) {
            vr->diverged = 1;
            vr->frame = i;
            vr->field = "input.frame";
            vr->diff.field = "input.frame";
            vr->diff.recorded = (int64_t)in.frame;
            vr->diff.replayed = (int64_t)i;
            vr->diff.kind = VAL_DEC;
            rc = 0;
            break;
        }
        sim_step(&sim, &in);
        if (state_diff(&ref, &sim, &vr->diff)) {
            vr->diverged = 1;
            vr->frame = i;
            vr->field = vr->diff.field;
            rc = 0;
            break;
        }
        vr->frames_matched++;
    }

    if (rc == 1 && fgetc(r.f) != EOF) {
        snprintf(vr->error, sizeof(vr->error), "trailing data after last frame");
        rc = -1;
    }
    log_reader_close(&r);
    return rc;
}

static void print_divergence(const VerifyResult* vr) {
    const FieldDiff* d = &vr->diff;
    if (!vr->diverged) {
        printf("divergence=none\n");
        return;
    }
    printf("divergence_frame=%" PRIu32 " divergence_field=%s ",
           vr->frame, vr->field);
    if (d->kind == VAL_FIX) {
        printf("recorded=%.4f replayed=%.4f\n",
               (double)d->recorded / FIX_ONE, (double)d->replayed / FIX_ONE);
    } else if (d->kind == VAL_HEX) {
        printf("recorded=0x%" PRIx64 " replayed=0x%" PRIx64 "\n",
               (uint64_t)d->recorded, (uint64_t)d->replayed);
    } else {
        printf("recorded=%" PRId64 " replayed=%" PRId64 "\n",
               d->recorded, d->replayed);
    }
}

/* ------------------------------------------------------------------ */
/* test helpers                                                       */
/* ------------------------------------------------------------------ */

#define TEST_FRAMES 600u
#define TEST_SEED   0xc0ffeeu
#define LOG_PATH    "replay_test.log"
#define TRUNC_PATH  "replay_trunc.log"
#define LIN_PATH    "replay_linear.log"
#define LIN_IN_PATH "replay_linear_inputs.log"

static Input inputs[TEST_FRAMES];

static int failures = 0;

static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int build_reference_log(void) {
    InputGen g;
    SimState sim;
    LogWriter w;
    input_gen_init(&g, TEST_SEED);
    sim_init(&sim, TEST_SEED);
    if (log_open(&w, LOG_PATH, TEST_SEED, 1) != 0) return -1;
    for (uint32_t i = 0; i < TEST_FRAMES; i++) {
        inputs[i] = input_gen_next(&g, i);
        sim_step(&sim, &inputs[i]);
        if (log_append(&w, &inputs[i], &sim) != 0) {
            log_close(&w);
            return -1;
        }
    }
    return log_close(&w);
}

static int record_synthetic(const char* path, uint32_t frames, int snapshots) {
    InputGen g;
    SimState sim;
    LogWriter w;
    input_gen_init(&g, TEST_SEED ^ 0x9e3779b9u);
    sim_init(&sim, TEST_SEED);
    if (log_open(&w, path, TEST_SEED, snapshots) != 0) return -1;
    for (uint32_t i = 0; i < frames; i++) {
        Input in = input_gen_next(&g, i);
        sim_step(&sim, &in);
        if (log_append(&w, &in, snapshots ? &sim : NULL) != 0) {
            log_close(&w);
            return -1;
        }
    }
    return log_close(&w);
}

static int patch_input(const char* path, uint32_t frame, const Input* in) {
    uint8_t buf[LOG_INPUT_BYTES];
    long off = (long)LOG_HEADER_BYTES + (long)frame * (long)LOG_FRAME_BYTES;
    FILE* f = fopen(path, "r+b");
    int ok;
    if (!f) return -1;
    encode_input(buf, in);
    ok = fseek(f, off, SEEK_SET) == 0 && fwrite(buf, 1, sizeof(buf), f) == sizeof(buf);
    if (fclose(f) != 0) ok = 0;
    return ok ? 0 : -1;
}

static int patch_state_hp(const char* path, uint32_t frame, int32_t hp) {
    uint8_t buf[4];
    long off = (long)LOG_HEADER_BYTES + (long)frame * (long)LOG_FRAME_BYTES
             + (long)LOG_INPUT_BYTES + 20;   /* hp field offset in snapshot */
    FILE* f = fopen(path, "r+b");
    int ok;
    if (!f) return -1;
    put_i32le(buf, hp);
    ok = fseek(f, off, SEEK_SET) == 0 && fwrite(buf, 1, sizeof(buf), f) == sizeof(buf);
    if (fclose(f) != 0) ok = 0;
    return ok ? 0 : -1;
}

static int copy_prefix(const char* src, const char* dst, long bytes) {
    uint8_t buf[512];
    long left = bytes;
    int ok = 1;
    FILE* in = fopen(src, "rb");
    FILE* out = NULL;
    if (!in) return -1;
    out = fopen(dst, "wb");
    if (!out) {
        fclose(in);
        return -1;
    }
    while (left > 0) {
        size_t want = left < (long)sizeof(buf) ? (size_t)left : sizeof(buf);
        size_t got = fread(buf, 1, want, in);
        if (got != want || fwrite(buf, 1, got, out) != got) {
            ok = 0;
            break;
        }
        left -= (long)got;
    }
    if (fclose(in) != 0) ok = 0;
    if (fclose(out) != 0) ok = 0;
    return ok ? 0 : -1;
}

static long file_bytes(const char* path) {
    FILE* f = fopen(path, "rb");
    long n;
    if (!f) return -1;
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return -1;
    }
    n = ftell(f);
    fclose(f);
    return n;
}

/* ------------------------------------------------------------------ */
/* self test                                                          */
/* ------------------------------------------------------------------ */

int main(void) {
    VerifyResult vr;
    int rc;

    if (build_reference_log() != 0) {
        printf("build_reference_log=0\nall_ok=0\n");
        return 1;
    }

    /* 1. full replay: every recorded frame must match exactly. */
    rc = replay_verify(LOG_PATH, &vr);
    check("replay_ok", rc == 1);
    check("frame_count", vr.frames_total == TEST_FRAMES);
    check("all_frames_match", vr.frames_matched == vr.frames_total);
    printf("replay_match=%" PRIu32 "/%" PRIu32 "\n",
           vr.frames_matched, vr.frames_total);
    printf("match_rate=%.2f%%\n",
           vr.frames_total ? 100.0 * (double)vr.frames_matched
                                 / (double)vr.frames_total : 0.0);
    check("match_rate_100", vr.frames_matched == vr.frames_total);
    print_divergence(&vr);

    /* inputs are read back in the recorded order. */
    {
        LogReader r;
        uint32_t i = 0;
        int seq_ok = 1;
        if (log_reader_open(&r, LOG_PATH) == 0) {
            for (; i < TEST_FRAMES; i++) {
                Input in;
                SimState st;
                if (log_reader_next(&r, &in, &st) != 1) { seq_ok = 0; break; }
                if (in.frame != inputs[i].frame ||
                    in.buttons != inputs[i].buttons ||
                    in.ax != inputs[i].ax || in.ay != inputs[i].ay) {
                    seq_ok = 0;
                    break;
                }
            }
            log_reader_close(&r);
        } else {
            seq_ok = 0;
        }
        printf("input_sequence_match=%" PRIu32 "/%" PRIu32 "\n", i, TEST_FRAMES);
        check("input_sequence_in_order", seq_ok && i == TEST_FRAMES);
    }

    /* 2. input tamper at frame 37 -> replay diverges on that same frame.
       Toggling ATTACK flips combo only, so the field is unambiguous. */
    {
        Input bad = inputs[37];
        bad.buttons ^= BTN_ATTACK;
        check("patch_input_applied", patch_input(LOG_PATH, 37, &bad) == 0);
        rc = replay_verify(LOG_PATH, &vr);
        check("input_tamper_detected", rc == 0 && vr.diverged);
        printf("input_tamper_frame=%" PRIu32 "\n", vr.frame);
        printf("input_tamper_field=%s\n", vr.field ? vr.field : "none");
        check("input_tamper_frame_exact", vr.frame == 37);
        check("input_tamper_field_exact",
              vr.field != NULL && strcmp(vr.field, "combo") == 0);
        print_divergence(&vr);
    }

    /* 3. recorded-state tamper at frame 90 -> hp, frame 90. */
    if (build_reference_log() != 0) {
        printf("build_reference_log=0\n");
        failures++;
    }
    check("patch_state_applied",
          patch_state_hp(LOG_PATH, 90, 1000) == 0);
    rc = replay_verify(LOG_PATH, &vr);
    check("state_tamper_detected", rc == 0 && vr.diverged);
    check("state_tamper_frame_exact", vr.frame == 90);
    check("state_tamper_field_exact",
          vr.field != NULL && strcmp(vr.field, "hp") == 0);
    print_divergence(&vr);

    /* 4. first divergence wins: corrupt frame 50 and frame 120. */
    if (build_reference_log() != 0) {
        printf("build_reference_log=0\n");
        failures++;
    }
    {
        Input late = inputs[120];
        late.buttons ^= BTN_ATTACK;
        check("patch_two_frames",
              patch_state_hp(LOG_PATH, 50, 7) == 0 &&
              patch_input(LOG_PATH, 120, &late) == 0);
    }
    rc = replay_verify(LOG_PATH, &vr);
    check("first_divergence_detected", rc == 0 && vr.diverged);
    check("first_divergence_is_earliest", vr.frame == 50);
    check("first_divergence_field",
          vr.field != NULL && strcmp(vr.field, "hp") == 0);
    print_divergence(&vr);

    /* 5. truncated logs are rejected instead of silently accepted. */
    if (build_reference_log() != 0) {
        printf("build_reference_log=0\n");
        failures++;
    }
    check("truncate_log",
          copy_prefix(LOG_PATH, TRUNC_PATH,
                      (long)LOG_HEADER_BYTES + 10L * (long)LOG_FRAME_BYTES + 20L) == 0);
    rc = replay_verify(TRUNC_PATH, &vr);
    check("truncated_log_rejected", rc == -1);
    printf("truncated_log_error=%s\n", vr.error);

    /* 6. fixed-size records: log bytes grow strictly linearly with frames. */
    {
        const uint32_t sizes[3] = { 100u, 200u, 400u };
        long bytes[3];
        int ok = 1;
        for (int i = 0; i < 3; i++) {
            long expect = (long)LOG_HEADER_BYTES
                        + (long)sizes[i] * (long)LOG_FRAME_BYTES;
            if (record_synthetic(LIN_PATH, sizes[i], 1) != 0) { ok = 0; break; }
            bytes[i] = file_bytes(LIN_PATH);
            printf("log_bytes frames=%" PRIu32 " bytes=%ld\n", sizes[i], bytes[i]);
            if (bytes[i] != expect) ok = 0;
        }
        check("log_size_fixed_per_frame", ok);
        check("log_size_linear",
              ok && (bytes[1] - (long)LOG_HEADER_BYTES) ==
                        2 * (bytes[0] - (long)LOG_HEADER_BYTES) &&
                    (bytes[2] - (long)LOG_HEADER_BYTES) ==
                        2 * (bytes[1] - (long)LOG_HEADER_BYTES));

        ok = 1;
        for (int i = 0; i < 3; i++) {
            long expect = (long)LOG_HEADER_BYTES
                        + (long)sizes[i] * (long)LOG_INPUT_BYTES;
            if (record_synthetic(LIN_IN_PATH, sizes[i], 0) != 0) { ok = 0; break; }
            if (file_bytes(LIN_IN_PATH) != expect) ok = 0;
        }
        check("log_size_input_only_linear", ok);
        printf("log_bytes_per_frame=%u input_only=%u\n",
               (unsigned)LOG_FRAME_BYTES, (unsigned)LOG_INPUT_BYTES);
    }

    remove(LOG_PATH);
    remove(TRUNC_PATH);
    remove(LIN_PATH);
    remove(LIN_IN_PATH);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
