
/* mini-spatial-audio: binaural positioning of mono sources.
   ITD by azimuth (max 0.6 ms), IID by azimuth, inverse-distance attenuation,
   soft-clip mixing of N sources into an interleaved stereo buffer. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SR 48000
#define MAX_FRAMES 4096
#define MAX_SRC 8

typedef struct {
    float x, y, z;      /* world position, meters; listener at origin facing -z */
    float gain;
} Source;

typedef struct {
    float yaw;          /* radians, 0 = facing -z */
} Listener;

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* azimuth in radians, 0 = front, +pi/2 = right */
static float azimuth_of(const Source* s, const Listener* l) {
    float dx = s->x, dz = s->z;
    float c = cosf(l->yaw), sn = sinf(l->yaw);
    float xr =  c * dx - sn * dz;
    float zr =  sn * dx + c * dz;
    return atan2f(xr, -zr);
}

static int itd_samples(float az) {
    float max_delay = 0.0006f * SR;          /* 0.6 ms, Woodworth-lite */
    return (int)lroundf(sinf(az) * max_delay);
}

static void iid_gains(float az, float* gl, float* gr) {
    float t = (sinf(az) + 1.0f) * 0.5f;      /* 0 = hard left, 1 = hard right */
    *gl = sqrtf(1.0f - t);
    *gr = sqrtf(t);
}

static float distance_gain(const Source* s) {
    float d = sqrtf(s->x * s->x + s->y * s->y + s->z * s->z);
    return 1.0f / (1.0f + d);
}

static float soft_clip(float v) {
    const float k = 0.8f;
    float a = fabsf(v);
    if (a <= k) return v;
    float sign = v < 0 ? -1.0f : 1.0f;
    return sign * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

static void render(const Source* srcs, int n, const Listener* l,
                   const float* mono, int frames, float* out /* interleaved */) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    for (int k = 0; k < n; k++) {
        float az = azimuth_of(&srcs[k], l);
        int delay = itd_samples(az);
        float gl, gr; iid_gains(az, &gl, &gr);
        float g = srcs[k].gain * distance_gain(&srcs[k]);
        for (int i = 0; i < frames; i++) {
            int j = i - delay;
            float s = (j >= 0 && j < frames) ? mono[j] : 0.0f;
            out[2*i + 0] += s * g * gl;
            out[2*i + 1] += s * g * gr;
        }
    }
    for (int i = 0; i < frames * 2; i++) out[i] = soft_clip(out[i]);
}

static float peak(const float* buf, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(buf[i]) > m) m = fabsf(buf[i]);
    return m;
}

/* ============ room reverb: IR convolution + equal-power crossfade ============ */

#define IR_LEN 192          /* impulse-response taps per room */
#define MAX_ROOMS 8
#define MAX_VOICES 3

typedef struct Room {
    int id;
    int ir_len;
    float* ir;          /* unit-energy impulse response; NULL after release */
    int refs;           /* number of voices currently holding this room */
    int used;
} Room;

typedef struct { Room rooms[MAX_ROOMS]; } RoomPool;

typedef struct {
    Room* room;
    float hl[IR_LEN], hr[IR_LEN];   /* per-channel input history */
    int pos;
    int active;
    float start, target, gain;      /* crossfade weight bookkeeping */
} RoomVoice;

typedef struct {
    RoomVoice v[MAX_VOICES];
    int fade_pos, fade_total, fading;
} Reverb;

static unsigned lcg_next(unsigned* s) {
    *s = *s * 1103515245u + 12345u;
    return *s;
}

/* Allocates a room slot and a unit-energy IR: direct path plus exponentially
   decaying shaped noise.  Starts with refs = 0; callers acquire explicitly. */
static Room* room_create(RoomPool* p, int id, float rt60) {
    for (int i = 0; i < MAX_ROOMS; i++)
        if (p->rooms[i].used && p->rooms[i].id == id) return NULL;
    Room* r = NULL;
    for (int i = 0; i < MAX_ROOMS; i++)
        if (!p->rooms[i].used) { r = &p->rooms[i]; break; }
    if (!r) return NULL;
    float* ir = (float*)malloc(sizeof(float) * IR_LEN);
    if (!ir) return NULL;

    unsigned seed = (unsigned)id * 2654435761u + 1u;
    ir[0] = 1.0f;                                   /* direct path */
    float energy = 1.0f;
    for (int k = 1; k < IR_LEN; k++) {
        lcg_next(&seed);
        float noise = ((seed >> 16) & 0x7fff) / 16384.0f - 1.0f;
        float env = expf(-3.0f * (float)k / rt60); /* rt60-style decay */
        ir[k] = noise * env;
        energy += ir[k] * ir[k];
    }
    float norm = 1.0f / sqrtf(energy);              /* unit energy: room gain neutral */
    for (int k = 0; k < IR_LEN; k++) ir[k] *= norm;

    r->id = id; r->ir_len = IR_LEN; r->ir = ir; r->refs = 0; r->used = 1;
    return r;
}

static void room_acquire(Room* r) {
    if (r && r->used) r->refs++;
}

/* One reference per holder.  The IR resource is freed only when the last
   holder drops it, so a room still fading out stays valid until the end. */
static void room_release(Room* r) {
    if (!r || !r->used) return;
    if (r->refs > 0) r->refs--;
    if (r->refs == 0) {
        free(r->ir);
        r->ir = NULL; r->ir_len = 0; r->id = -1; r->used = 0;
    }
}

/* Equal-power crossfade curve for normalized progress u in [0,1]:
   outgoing g_o = cos(pi u / 2), incoming g_i = sin(pi u / 2).
   g_o^2 + g_i^2 = 1 at every sample, so the summed power of the two
   uncorrelated room responses stays constant through the transition.
   g_o'(0) = 0 and g_i'(1) = 0 join steady state slope-continuously, and a
   voice only starts/stops while its gain is exactly 0: no step, no pop. */
static void eq_power_curve(float u, float* go, float* gi) {
    *go = cosf(u * (float)M_PI * 0.5f);
    *gi = sinf(u * (float)M_PI * 0.5f);
}

static void reverb_init(Reverb* rv) { memset(rv, 0, sizeof(*rv)); }

static void reverb_close(Reverb* rv) {
    for (int i = 0; i < MAX_VOICES; i++) {
        if (rv->v[i].active) room_release(rv->v[i].room);
        rv->v[i].active = 0;
        rv->v[i].room = NULL;
    }
    rv->fading = 0;
}

/* Per-sample gains for every active voice.  Two voices interpolate the mixing
   angle theta = atan2(g_in, g_out), which is exactly cos/sin for a fade that
   starts from steady state and stays value-continuous when a fade is
   interrupted/retargeted.  Three voices fall back to norm-preserving linear
   weights.  u == 0 / 1 snap to exact start/target values. */
static void reverb_update_gains(Reverb* rv, float u) {
    int n = 0, idx[MAX_VOICES];
    for (int i = 0; i < MAX_VOICES; i++)
        if (rv->v[i].active) idx[n++] = i;

    if (u <= 0.0f) {
        for (int q = 0; q < n; q++) rv->v[idx[q]].gain = rv->v[idx[q]].start;
        return;
    }
    if (u >= 1.0f) {
        for (int q = 0; q < n; q++) rv->v[idx[q]].gain = rv->v[idx[q]].target;
        return;
    }
    if (n == 1) {
        rv->v[idx[0]].gain = rv->v[idx[0]].target;
        return;
    }
    if (n == 2) {
        RoomVoice* a = &rv->v[idx[0]];
        RoomVoice* b = &rv->v[idx[1]];
        float th0 = atan2f(b->start, a->start);
        float th1 = atan2f(b->target, a->target);
        float th = th0 + (th1 - th0) * u;
        a->gain = cosf(th);
        b->gain = sinf(th);
        return;
    }
    float w[MAX_VOICES], d = 0.0f;
    for (int q = 0; q < n; q++) {
        RoomVoice* x = &rv->v[idx[q]];
        w[q] = x->start + (x->target - x->start) * u;
        d += w[q] * w[q];
    }
    d = sqrtf(d);
    for (int q = 0; q < n; q++) rv->v[idx[q]].gain = w[q] / d;
}

/* Begin a crossfade to target room, taking one reference on it.  The old
   room's reference is kept by its voice until the fade completes.
   Retargeting an in-progress fade snapshots current gains (no discontinuity).
   Returns -1 on bad arguments or when no voice slot is free. */
static int reverb_set_room(Reverb* rv, Room* r, int fade_frames) {
    if (!r || !r->used || fade_frames < 1) return -1;

    int active = 0;
    for (int i = 0; i < MAX_VOICES; i++) if (rv->v[i].active) active++;

    for (int i = 0; i < MAX_VOICES; i++) {
        if (rv->v[i].active && rv->v[i].room == r) {
            if (!rv->fading && active == 1) return 0;   /* already the current room */
            for (int j = 0; j < MAX_VOICES; j++)
                if (rv->v[j].active) {
                    rv->v[j].start = rv->v[j].gain;
                    rv->v[j].target = (j == i) ? 1.0f : 0.0f;
                }
            rv->fade_pos = 0; rv->fade_total = fade_frames; rv->fading = 1;
            return 0;
        }
    }

    int slot = -1;
    for (int i = 0; i < MAX_VOICES; i++)
        if (!rv->v[i].active) { slot = i; break; }
    if (slot < 0) return -1;

    if (active == 0) {                              /* first room: no fade needed */
        RoomVoice* v = &rv->v[slot];
        room_acquire(r);
        v->room = r; v->active = 1; v->pos = 0;
        v->start = v->target = v->gain = 1.0f;
        memset(v->hl, 0, sizeof(v->hl));
        memset(v->hr, 0, sizeof(v->hr));
        return 0;
    }

    for (int j = 0; j < MAX_VOICES; j++)
        if (rv->v[j].active) {
            rv->v[j].start = rv->v[j].gain;
            rv->v[j].target = 0.0f;
        }
    RoomVoice* v = &rv->v[slot];
    room_acquire(r);
    v->room = r; v->active = 1; v->pos = 0;
    v->start = 0.0f; v->target = 1.0f; v->gain = 0.0f;
    memset(v->hl, 0, sizeof(v->hl));
    memset(v->hr, 0, sizeof(v->hr));
    rv->fade_pos = 0; rv->fade_total = fade_frames; rv->fading = 1;
    return 0;
}

static float voice_convolve(RoomVoice* v, float* hist, float x) {
    hist[v->pos] = x;
    const float* ir = v->room->ir;
    float y = 0.0f;
    for (int k = 0; k < v->room->ir_len; k++) {
        int j = v->pos - k;
        if (j < 0) j += IR_LEN;
        y += ir[k] * hist[j];
    }
    return y;
}

/* Processes interleaved stereo.  Both rooms' IRs run in parallel for the
   whole fade and their outputs are summed with the per-sample curve gains. */
static void reverb_process(Reverb* rv, const float* in, int frames, float* out) {
    for (int i = 0; i < frames; i++) {
        if (rv->fading) {
            rv->fade_pos++;
            float u = (float)rv->fade_pos / (float)rv->fade_total;
            reverb_update_gains(rv, u);
            if (rv->fade_pos >= rv->fade_total) {
                rv->fading = 0;
                for (int q = 0; q < MAX_VOICES; q++) {
                    RoomVoice* x = &rv->v[q];
                    if (!x->active) continue;
                    if (x->target == 0.0f) {            /* fade-out finished: release */
                        room_release(x->room);
                        x->active = 0; x->room = NULL; x->gain = 0.0f;
                    } else {
                        x->start = x->target = x->gain = 1.0f;
                    }
                }
            }
        }

        float xl = in[2 * i], xr = in[2 * i + 1];
        float yl = 0.0f, yr = 0.0f;
        for (int q = 0; q < MAX_VOICES; q++) {
            RoomVoice* x = &rv->v[q];
            if (!x->active) continue;
            yl += voice_convolve(x, x->hl, xl) * x->gain;
            yr += voice_convolve(x, x->hr, xr) * x->gain;
            x->pos = (x->pos + 1) % IR_LEN;
        }
        out[2 * i] = yl;
        out[2 * i + 1] = yr;
    }
}

/* reference convolution with zero initial history, same summation order */
static void ref_convolve(const Room* r, const float* in, int frames, float* out) {
    for (int i = 0; i < frames; i++) {
        float y = 0.0f;
        int kmax = i + 1 < r->ir_len ? i + 1 : r->ir_len;
        for (int k = 0; k < kmax; k++) y += r->ir[k] * in[i - k];
        out[i] = y;
    }
}

static float buf_rms(const float* b, int from, int len) {
    float e = 0.0f;
    for (int i = 0; i < len; i++) e += b[from + i] * b[from + i];
    return sqrtf(e / (float)len);
}

static float max_jump(const float* b, int from, int len) {
    float m = 0.0f;
    for (int i = from + 1; i < from + len; i++) {
        float d = fabsf(b[i] - b[i - 1]);
        if (d > m) m = d;
    }
    return m;
}

/* left-channel views of an interleaved buffer: jump/RMS on one channel only,
   since adjacent interleaved samples alternate between L and R */
static void left_view(const float* stereo, float* l, int frames) {
    for (int i = 0; i < frames; i++) l[i] = stereo[2 * i];
}

static void process_chunks(Reverb* rv, const float* in, int total,
                           int chunk, float* out) {
    for (int off = 0; off < total; off += chunk) {
        int n = chunk < total - off ? chunk : total - off;
        reverb_process(rv, in + 2 * off, n, out + 2 * off);
    }
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    float mono[MAX_FRAMES];
    float out[MAX_FRAMES * 2];
    for (int i = 0; i < 256; i++) mono[i] = (i == 64) ? 1.0f : 0.0f;  /* impulse at 64 */

    Listener l = { 0.0f };
    Source right = { 3.0f, 0.0f, 0.0f, 1.0f };
    render(&right, 1, &l, mono, 256, out);
    int best_l = 0, best_r = 0;
    for (int i = 1; i < 256; i++) {
        if (fabsf(out[2*i+0]) > fabsf(out[2*best_l+0])) best_l = i;
        if (fabsf(out[2*i+1]) > fabsf(out[2*best_r+1])) best_r = i;
    }
    printf("right_source peakL=%d peakR=%d (impulse@64)\n", best_l, best_r);

    Source left = { -3.0f, 0.0f, 0.0f, 1.0f };
    render(&left, 1, &l, mono, 256, out);
    int bl = 0, br = 0;
    for (int i = 1; i < 256; i++) {
        if (fabsf(out[2*i+0]) > fabsf(out[2*best_l+0])) bl = i;
        if (fabsf(out[2*i+1]) > fabsf(out[2*best_r+1])) br = i;
    }
    printf("left_source peakL=%d peakR=%d (impulse@64)\n", bl, br);

    Source near = { 0.0f, 0.0f, -1.0f, 1.0f };
    Source far  = { 0.0f, 0.0f, -9.0f, 1.0f };
    render(&near, 1, &l, mono, 256, out); float pn = peak(out, 256*2);
    render(&far, 1, &l, mono, 256, out);  float pf = peak(out, 256*2);
    printf("attenuation near=%.4f far=%.4f ratio=%.3f\n", pn, pf, pf / pn);

    Source many[4] = { {1,0,0,1}, {-1,0,0,1}, {0,0,-1,1}, {0,0,1,1} };
    float loud[MAX_FRAMES];
    for (int i = 0; i < 256; i++) loud[i] = 0.9f;
    render(many, 4, &l, loud, 256, out);
    printf("mix_peak=%.4f (soft clip 上限 1.0)\n", peak(out, 256*2));

    /* ---------- room crossfade tests ---------- */
    enum { TFRAMES = 1024, FADE = 128, CHUNK = 37 };
    float in2[TFRAMES * 2], rev_out[TFRAMES * 2];
    float mono_ref[TFRAMES], ref_out[TFRAMES];
    unsigned rng = 0x12345678u;
    for (int i = 0; i < TFRAMES; i++) {
        rng = lcg_next(&rng);
        float x = ((rng >> 16) & 0x7fff) / 16384.0f - 1.0f;
        in2[2 * i] = x;
        in2[2 * i + 1] = x * 0.5f;
        mono_ref[i] = x;
    }

    RoomPool pool;
    memset(&pool, 0, sizeof(pool));
    Room* ra = room_create(&pool, 1, 40.0f);
    Room* rb = room_create(&pool, 2, 90.0f);
    check("room_alloc", ra && rb);
    if (ra) {
        float e = 0.0f;
        for (int k = 0; k < ra->ir_len; k++) e += ra->ir[k] * ra->ir[k];
        check("ir_unit_energy", fabsf(e - 1.0f) < 1e-4f);
        float early = 0.0f, late = 0.0f;
        for (int k = 1; k < 24; k++) early += ra->ir[k] * ra->ir[k];
        for (int k = IR_LEN - 24; k < IR_LEN; k++) late += ra->ir[k] * ra->ir[k];
        check("ir_decays", early > late * 4.0f);
        float max_tap = 0.0f;
        for (int k = 1; k < ra->ir_len; k++)
            if (fabsf(ra->ir[k]) > max_tap) max_tap = fabsf(ra->ir[k]);
        check("ir_direct_path", fabsf(ra->ir[0]) > max_tap);
    }
    check("dup_id_rejected", room_create(&pool, 1, 40.0f) == NULL);
    check("no_ref_before_acquire", ra->refs == 0 && rb->refs == 0);

    Reverb rv;
    reverb_init(&rv);
    check("bad_args_rejected",
          reverb_set_room(&rv, NULL, FADE) == -1
          && reverb_set_room(&rv, ra, 0) == -1);

    /* first room takes a reference and passes through with no pop/fade */
    check("first_room_ok", reverb_set_room(&rv, ra, FADE) == 0);
    check("first_room_ref", ra->refs == 1);
    process_chunks(&rv, in2, TFRAMES, CHUNK, rev_out);
    ref_convolve(ra, mono_ref, TFRAMES, ref_out);
    float max_err = 0.0f;
    for (int i = 0; i < TFRAMES; i++) {
        float d = fabsf(rev_out[2 * i] - ref_out[i]);
        if (d > max_err) max_err = d;
    }
    check("steady_matches_ir", max_err < 1e-5f);
    check("first_room_no_fade", rv.fading == 0);

    /* equal-power curve: endpoints exact, midpoint 1/sqrt(2), squared sum = 1 */
    float go, gi, curve_err = 0.0f;
    eq_power_curve(0.0f, &go, &gi);
    curve_err += fabsf(go - 1.0f) + fabsf(gi);
    eq_power_curve(1.0f, &go, &gi);
    curve_err += fabsf(go) + fabsf(gi - 1.0f);
    eq_power_curve(0.5f, &go, &gi);
    curve_err += fabsf(go - 0.70710678f) + fabsf(gi - 0.70710678f);
    for (int s = 0; s <= 64; s++) {
        eq_power_curve(s / 64.0f, &go, &gi);
        curve_err += fabsf(go * go + gi * gi - 1.0f);
    }
    check("eq_power_curve", curve_err < 1e-5f);

    /* ---- crossfade: no pop, energy continuous, refs & release ---- */
    const int sw = TFRAMES - FADE;
    reverb_set_room(&rv, rb, FADE);
    check("fade_start_refs", ra->refs == 1 && rb->refs == 1);
    process_chunks(&rv, in2, TFRAMES, CHUNK, rev_out);
    check("fade_done", rv.fading == 0);

    /* reference response of the new room alone, for identity check */
    Reverb h1;
    reverb_init(&h1); reverb_set_room(&h1, rb, FADE);
    float ho1[TFRAMES * 2];
    reverb_process(&h1, in2, TFRAMES, ho1);
    reverb_close(&h1);                         /* drops its rb reference */
    float left_xf[TFRAMES];
    left_view(rev_out, left_xf, TFRAMES);

    /* ---- no-pop: isolated pool (rooms here have their own lifetime).
       Run each room far past its IR length on constant DC so the diffuse
       tail has decayed and only the level (sum of IR taps) remains.
       A hard switch jumps that level in one sample; the equal-power fade
       spreads the same change over FADE samples. ---- */
    float dc2[TFRAMES * 2];
    for (int i = 0; i < TFRAMES * 2; i++) dc2[i] = (i % 2 == 0) ? 0.5f : 0.25f;

    RoomPool pop_pool;
    memset(&pop_pool, 0, sizeof(pop_pool));
    Room* pa = room_create(&pop_pool, 101, 40.0f);
    Room* pb = room_create(&pop_pool, 102, 90.0f);

    enum { WARM = 1024 };
    float* warm = (float*)calloc((size_t)WARM * 2, sizeof(float));
    Reverb hra, hrb, xf;
    reverb_init(&hra); reverb_set_room(&hra, pa, FADE);
    reverb_init(&hrb); reverb_set_room(&hrb, pb, FADE);
    reverb_init(&xf);  reverb_set_room(&xf,  pa, FADE);
    for (int off = 0; off < WARM; off += 64) {
        int n = 64 < WARM - off ? 64 : WARM - off;
        reverb_process(&hra, dc2 + 2 * off, n, warm + 2 * off);
        reverb_process(&hrb, dc2 + 2 * off, n, warm + 2 * off);
        reverb_process(&xf,  dc2 + 2 * off, n, warm + 2 * off);
    }
    free(warm);

    float a_st[FADE * 2 * 2], b_st[FADE * 2 * 2], xf_st[FADE * 2 * 2];
    reverb_process(&hra, dc2, FADE * 2, a_st); reverb_close(&hra);
    reverb_process(&hrb, dc2, FADE * 2, b_st); reverb_close(&hrb);
    reverb_set_room(&xf, pb, FADE);
    reverb_process(&xf, dc2, FADE * 2, xf_st); reverb_close(&xf);

    /* hard switch = pa's last steady level, then pb's first steady level */
    float lvl_a = a_st[0], lvl_b = b_st[0];
    float hard_l[FADE * 2], xf_l[FADE * 2];
    hard_l[0] = lvl_a;
    for (int i = 1; i < FADE * 2; i++) hard_l[i] = lvl_b;
    for (int i = 0; i < FADE * 2; i++) xf_l[i] = xf_st[2 * i];
    float hard_jump = max_jump(hard_l, 0, FADE * 2);
    float xf_jump = max_jump(xf_l, 0, FADE * 2);
    printf("jumps hard=%.6f xfade=%.6f (levels %.4f -> %.4f)\n",
           hard_jump, xf_jump, lvl_a, lvl_b);
    check("no_pop", xf_jump < hard_jump * 0.1f);
    check("pop_pool_clean", pa->used == 0 && pb->used == 0
          && pa->refs == 0 && pb->refs == 0);

    check("old_room_released", ra->refs == 0 && ra->used == 0 && ra->ir == NULL);
    check("new_room_held", rb->refs == 1 && rb->used == 1);

    /* steady output after the fade is exactly the new room's IR response */
    float post_err = 0.0f;
    for (int i = IR_LEN; i < TFRAMES; i++) {
        float d = fabsf(rev_out[2 * i] - ho1[2 * i]);
        if (d > post_err) post_err = d;
    }
    check("post_fade_identity", post_err < 1e-5f);

    /* energy continuity: RMS of windows along the fade stays within +/-30% */
    float base = buf_rms(left_xf, sw - 96, 96);
    int worst = 1;
    for (int off = sw; off + 32 <= TFRAMES; off += 32) {
        float r = buf_rms(left_xf, off, 32);
        if (r < base * 0.70f || r > base * 1.30f) worst = 0;
    }
    check("energy_continuous", worst);

    /* ---- interrupted fade: retarget mid-crossfade, still no pop ---- */
    Reverb rv2;
    reverb_init(&rv2);
    Room* rc = room_create(&pool, 3, 140.0f);
    Room* rd = room_create(&pool, 4, 25.0f);
    check("more_rooms_alloc", rc && rd);
    reverb_set_room(&rv2, rc, FADE);
    reverb_process(&rv2, dc2, IR_LEN, rev_out);          /* warm up history */
    reverb_set_room(&rv2, rd, FADE);
    float acc[(FADE * 3) * 2], acc_l[FADE * 3];
    reverb_process(&rv2, dc2, FADE / 2, acc);
    reverb_set_room(&rv2, rc, FADE);                       /* retarget halfway */
    reverb_process(&rv2, dc2, FADE * 2 + FADE / 2,
                   acc + (FADE / 2) * 2);
    left_view(acc, acc_l, FADE * 3);
    check("retarget_no_pop",
          max_jump(acc_l, 1, FADE * 3 - 1) < hard_jump * 0.1f);
    reverb_close(&rv2);
    check("retarget_refs_balanced", rc->refs == 0 && rd->refs == 0
          && rc->used == 0 && rd->used == 0);

    /* ---- voice slots exhausted ---- */
    Reverb rv3;
    reverb_init(&rv3);
    reverb_set_room(&rv3, rb, FADE);                       /* rb already exists */
    Room* r5 = room_create(&pool, 5, 60.0f);
    Room* r6 = room_create(&pool, 6, 60.0f);
    check("extras_alloc", r5 && r6);
    reverb_set_room(&rv3, r5, FADE);                       /* voices: rb, r5 */
    reverb_set_room(&rv3, r6, FADE);                       /* voices: rb, r5, r6 */
    Room* r7 = room_create(&pool, 7, 60.0f);
    check("fourth_voice_rejected", r7 && reverb_set_room(&rv3, r7, FADE) == -1);
    check("rejected_room_not_referenced", r7->refs == 0);
    reverb_close(&rv3);
    check("voices_released_after_close",
          r5->refs == 0 && r6->refs == 0
          && !r5->used && !r6->used);

    reverb_close(&rv);
    check("close_releases_rb", rb->refs == 0 && rb->used == 0);

    /* same-room request while steady is a no-op and adds no reference */
    Reverb rv4;
    reverb_init(&rv4);
    reverb_set_room(&rv4, r7, FADE);
    check("same_room_noop", reverb_set_room(&rv4, r7, FADE) == 0
          && r7->refs == 1);
    reverb_close(&rv4);
    check("close_frees_last", r7->refs == 0 && r7->used == 0);

    /* pool slot freed on release can be reused */
    Room* reused = room_create(&pool, 1, 40.0f);
    check("slot_reused", reused != NULL);
    int leak = 0;
    for (int i = 0; i < MAX_ROOMS; i++)
        if (pool.rooms[i].used && pool.rooms[i].refs != 0) leak = 1;
    check("no_refcount_leak", !leak);
    /* test-only teardown: every remaining slot must be unreferenced */
    for (int i = 0; i < MAX_ROOMS; i++) {
        Room* r = &pool.rooms[i];
        if (r->used) {
            check("teardown_unreferenced", r->refs == 0);
            free(r->ir);
            r->ir = NULL; r->used = 0;
        }
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
