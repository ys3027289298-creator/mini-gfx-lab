
/* mixer: multi-bus audio mixing with per-bus gain, sidechain ducking and
   soft clipping. All buffers are interleaved stereo.

   Latency compensation: each bus's insert chain may add a different
   processing latency. The mixer measures it (impulse -> peak offset),
   then delays every bus by (max_latency - bus_latency) frames so all
   buses are aligned to the slowest one before summing. Compensation is
   relative to the loudest-latency bus *active in the call*, so a single
   bus soloed on its own gets zero compensation and its samples pass
   through unchanged. Assumes bus latencies are stable across calls;
   changing the active bus set mid-stream shifts compensation and may
   glitch. */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_SRC 8
#define FRAMES 128
#define MAX_LATENCY 256              /* frames, per bus */
#define COMP_RING (MAX_LATENCY + 1)  /* delay-line length, frames */
#define PI 3.14159265358979323846f

typedef struct {
    int bus;        /* 0 = sfx, 1 = music, 2 = voice */
    float gain;
} Source;

typedef struct {
    float gain[3];
    float duck_depth;    /* 0..1, applied to bus 0 when voice is active */
    int duck_enabled;
    int latency[3];      /* measured insert latency per bus, frames */
    int comp[3];         /* compensation applied in the last mix call */
    float comp_buf[3][COMP_RING * 2];  /* per-bus delay lines, stereo */
    int comp_pos[3];     /* write positions */
} Mixer;

/* insert processor: mono, stateless across the given block */
typedef void (*InsertFn)(const float* in, float* out, int n);

/* measure an insert's processing latency: feed an impulse, return the
   frame offset of the output peak */
static int measure_latency(InsertFn fn) {
    enum { N = 1024 };
    float in[N], out[N];
    memset(in, 0, sizeof(in));
    memset(out, 0, sizeof(out));
    in[0] = 1.0f;
    fn(in, out, N);
    int best = 0;
    for (int i = 1; i < N; i++)
        if (fabsf(out[i]) > fabsf(out[best])) best = i;
    return best;
}

/* compensation currently applied to a bus, in frames */
static int mixer_compensation(const Mixer* mx, int bus) {
    if (bus < 0 || bus > 2) return 0;
    return mx->comp[bus];
}

static float soft_clip(float v) {
    const float k = 0.9f;
    float a = fabsf(v);
    if (a <= k) return v;
    float s = v < 0 ? -1.0f : 1.0f;
    return s * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

static void mix(Mixer* mx, const Source* srcs, int n,
                const float* const* in, float* out, int frames) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    float bus[3][FRAMES * 2];
    memset(bus, 0, sizeof(bus));

    for (int s = 0; s < n; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b > 2) continue;
        for (int i = 0; i < frames * 2; i++)
            bus[b][i] += in[s][i] * srcs[s].gain;
    }

    /* align all active buses to the slowest one */
    int max_lat = 0;
    for (int s = 0; s < n; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b > 2) continue;
        int l = mx->latency[b];
        if (l > MAX_LATENCY) l = MAX_LATENCY;
        if (l > max_lat) max_lat = l;
    }
    for (int b = 0; b < 3; b++) {
        int l = mx->latency[b];
        if (l > MAX_LATENCY) l = MAX_LATENCY;
        mx->comp[b] = max_lat - l;
    }

    float dg = 1.0f;
    if (mx->duck_enabled) {
        /* voice bus energy drives ducking of the sfx bus */
        float e = 0.0f;
        for (int i = 0; i < frames * 2; i++) e += fabsf(bus[2][i]);
        e /= (float)(frames * 2);
        if (e > 0.01f) dg = 1.0f - mx->duck_depth;
    }

    float bg[3] = { mx->gain[0] * dg, mx->gain[1], mx->gain[2] };
    for (int i = 0; i < frames; i++) {
        float l = 0.0f, r = 0.0f;
        for (int b = 0; b < 3; b++) {
            int w = mx->comp_pos[b];
            mx->comp_buf[b][w * 2]     = bus[b][i * 2];
            mx->comp_buf[b][w * 2 + 1] = bus[b][i * 2 + 1];
            int d = w - mx->comp[b];
            if (d < 0) d += COMP_RING;
            mx->comp_pos[b] = (w + 1) % COMP_RING;
            l += mx->comp_buf[b][d * 2] * bg[b];
            r += mx->comp_buf[b][d * 2 + 1] * bg[b];
        }
        out[i * 2] = soft_clip(l);
        out[i * 2 + 1] = soft_clip(r);
    }
}

static float peak(const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(b[i]) > m) m = fabsf(b[i]);
    return m;
}

/* magnitude of one frequency bin (Goertzel-style DFT), left channel */
static float mag_at(const float* x, int frames, float freq, float sr) {
    float re = 0.0f, im = 0.0f;
    for (int i = 0; i < frames; i++) {
        float ph = 2.0f * PI * freq * (float)i / sr;
        re += x[i * 2] * cosf(ph);
        im -= x[i * 2] * sinf(ph);
    }
    return 2.0f * sqrtf(re * re + im * im) / (float)frames;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* demo inserts: passthrough (0 latency) and a 48-frame delay */
static void insert_passthrough(const float* in, float* out, int n) {
    memcpy(out, in, sizeof(float) * (size_t)n);
}
#define DEMO_DELAY 48
static void insert_delay48(const float* in, float* out, int n) {
    for (int i = 0; i < n; i++)
        out[i] = i >= DEMO_DELAY ? in[i - DEMO_DELAY] : 0.0f;
}

int main(void) {
    float sfx[FRAMES * 2], music[FRAMES * 2], voice[FRAMES * 2], out[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++) {
        sfx[i] = 0.5f; music[i] = 0.25f; voice[i] = 0.0f;
    }
    const float* in[3] = { sfx, music, voice };
    Source srcs[3] = { {0, 1.0f}, {1, 1.0f}, {2, 1.0f} };
    Mixer mx = { .gain = {1.0f, 1.0f, 1.0f}, .duck_depth = 0.5f, .duck_enabled = 1 };

    mix(&mx, srcs, 3, in, out, FRAMES);
    check("sum_ok", fabsf(out[0] - 0.75f) < 1e-5f);
    check("no_clip_when_low", peak(out, FRAMES * 2) < 0.99f);

    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.3f;
    mix(&mx, srcs, 3, in, out, FRAMES);
    float with_duck = out[0];

    mx.duck_enabled = 0;
    mix(&mx, srcs, 3, in, out, FRAMES);
    float without_duck = out[0];
    check("duck_lowers_sfx", with_duck < without_duck);

    mx.duck_enabled = 1;
    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.0f;
    mix(&mx, srcs, 3, in, out, FRAMES);
    check("duck_idle_passthrough", fabsf(out[0] - 0.75f) < 1e-5f);

    for (int s = 0; s < 3; s++) srcs[s].gain = 4.0f;
    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.5f;
    mix(&mx, srcs, 3, in, out, FRAMES);
    check("soft_clip_bounds", peak(out, FRAMES * 2) <= 1.0f + 1e-6f);

    srcs[0].bus = 7;   /* invalid bus must be ignored */
    mix(&mx, srcs, 3, in, out, FRAMES);
    check("bad_bus_ignored", peak(out, FRAMES * 2) <= 1.0f + 1e-6f);

    /* --- latency compensation --- */
    check("measure_passthrough", measure_latency(insert_passthrough) == 0);
    int music_lat = measure_latency(insert_delay48);
    check("measure_delay48", music_lat == DEMO_DELAY);

    /* Two buses carry the same multi-tone signal; the music bus insert
       delays it by 48 frames = half a period at 500 Hz / 48 kHz, so
       without alignment the mix is comb-filtered: odd multiples of
       500 Hz cancel, even multiples pass. */
    const float sr = 48000.0f;
    const float tones[] = { 250, 500, 750, 1000, 1500 };
    enum { BLOCKS = 8, TOTAL = BLOCKS * FRAMES };
    static float sig[TOTAL * 2], mus[TOTAL * 2];
    static float rec_before[TOTAL * 2], rec_after[TOTAL * 2];
    for (int i = 0; i < TOTAL; i++) {
        float v = 0.0f, d = 0.0f;
        for (int t = 0; t < 5; t++) {
            v += 0.15f * sinf(2.0f * PI * tones[t] * (float)i / sr);
            if (i >= DEMO_DELAY)
                d += 0.15f * sinf(2.0f * PI * tones[t] * (float)(i - DEMO_DELAY) / sr);
        }
        sig[i * 2] = sig[i * 2 + 1] = v;
        mus[i * 2] = mus[i * 2 + 1] = d;
    }

    Mixer mx2 = { .gain = {1.0f, 1.0f, 1.0f} };
    Source s2[2] = { {0, 1.0f}, {1, 1.0f} };
    for (int blk = 0; blk < BLOCKS; blk++) {
        const float* bin[2] = { sig + blk * FRAMES * 2, mus + blk * FRAMES * 2 };
        mix(&mx2, s2, 2, bin, rec_before + blk * FRAMES * 2, FRAMES);
    }

    Mixer mx3 = { .gain = {1.0f, 1.0f, 1.0f} };
    mx3.latency[1] = music_lat;   /* measured, not hardcoded */
    for (int blk = 0; blk < BLOCKS; blk++) {
        const float* bin[2] = { sig + blk * FRAMES * 2, mus + blk * FRAMES * 2 };
        mix(&mx3, s2, 2, bin, rec_after + blk * FRAMES * 2, FRAMES);
    }

    /* spectrum comparison, steady state (skip the first block) */
    const float* wb = rec_before + FRAMES * 2;
    const float* wa = rec_after + FRAMES * 2;
    int aw = TOTAL - FRAMES;
    printf("freq_hz  before   after\n");
    float mb500 = 0.0f, ma500 = 0.0f, mb1000 = 0.0f, ma1000 = 0.0f;
    for (int f = 0; f < 5; f++) {
        float mb = mag_at(wb, aw, tones[f], sr);
        float ma = mag_at(wa, aw, tones[f], sr);
        printf("%7.0f  %7.4f  %7.4f\n", tones[f], mb, ma);
        if (tones[f] == 500.0f)  { mb500 = mb;  ma500 = ma; }
        if (tones[f] == 1000.0f) { mb1000 = mb; ma1000 = ma; }
    }
    check("unaligned_cancels_500hz", mb500 < 0.02f);
    check("aligned_restores_500hz", ma500 > 0.25f);
    check("unaligned_passes_1000hz", mb1000 > 0.25f);
    check("aligned_restores_1000hz", ma1000 > 0.25f);

    /* compensation is queryable */
    check("comp_query_sfx", mixer_compensation(&mx3, 0) == DEMO_DELAY);
    check("comp_query_music", mixer_compensation(&mx3, 1) == 0);
    check("comp_query_bad_bus", mixer_compensation(&mx3, 9) == 0);

    /* a single bus soloed gets zero compensation: samples pass through
       unchanged even though its measured latency is nonzero */
    Mixer mx4 = { .gain = {1.0f, 1.0f, 1.0f} };
    mx4.latency[1] = music_lat;
    float solo_in[FRAMES * 2], solo_out[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++)
        solo_in[i] = 0.3f * sinf(2.0f * PI * (float)i / 64.0f);
    const float* sin_ptr[1] = { solo_in };
    Source s_solo[1] = { {1, 1.0f} };
    mix(&mx4, s_solo, 1, sin_ptr, solo_out, FRAMES);
    int identical = 1;
    for (int i = 0; i < FRAMES * 2; i++)
        if (solo_out[i] != solo_in[i]) identical = 0;
    check("solo_samples_unchanged", identical);
    check("solo_comp_zero", mixer_compensation(&mx4, 1) == 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
