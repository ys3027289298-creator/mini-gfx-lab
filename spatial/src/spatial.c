
/* mini-spatial-audio: binaural positioning of mono sources.
   ITD by azimuth (max 0.6 ms), IID by azimuth, inverse-distance attenuation,
   soft-clip mixing of N sources into an interleaved stereo buffer.

   Network path: a sender packs each source into a 31-bit code (see
   spatial_quant.h); the receiver decodes, interpolates between packets and
   renders from the decoded position. ITD and IID are carried explicitly in the
   code, so a decoded frame makes exactly the sender's decisions. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "binaural.h"
#include "spatial_quant.h"

#define SR BINAURAL_SR
#define MAX_FRAMES 4096
#define MAX_SRC SQ_MAX_SOURCES

typedef struct {
    float x, y, z;      /* world position, meters; listener at origin facing -z */
    float gain;
} Source;

typedef struct {
    float yaw;          /* radians, 0 = facing -z */
} Listener;

/* One network packet: sender count plus one code per source. */
typedef struct {
    int count;
    uint32_t code[MAX_SRC];
} Packet;

static float source_distance(const Source* s) {
    return sqrtf(s->x * s->x + s->y * s->y + s->z * s->z);
}

static float distance_gain_from(float d) {
    return 1.0f / (1.0f + d);
}

static float soft_clip(float v) {
    const float k = 0.8f;
    float a = fabsf(v);
    if (a <= k) return v;
    float sign = v < 0 ? -1.0f : 1.0f;
    return sign * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

/* mix one source into `out`, reading mono[base + i - delay] */
static void mix_source(float* out, int frames, const float* mono, int mono_len,
                       int base, int delay, float g, float gl, float gr) {
    for (int i = 0; i < frames; i++) {
        int j = base + i - delay;
        float s = (j >= 0 && j < mono_len) ? mono[j] : 0.0f;
        out[2*i + 0] += s * g * gl;
        out[2*i + 1] += s * g * gr;
    }
}

static void soft_clip_buffer(float* out, int frames) {
    for (int i = 0; i < frames * 2; i++) out[i] = soft_clip(out[i]);
}

/* reference render: positions in float */
static void render(const Source* srcs, int n, const Listener* l,
                   const float* mono, int frames, float* out) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    for (int k = 0; k < n; k++) {
        float az = binaural_azimuth(srcs[k].x, srcs[k].z, l->yaw);
        int delay = binaural_itd_samples(az);
        float gl, gr;
        binaural_iid_gains(az, &gl, &gr);
        float g = srcs[k].gain * distance_gain_from(source_distance(&srcs[k]));
        mix_source(out, frames, mono, frames, 0, delay, g, gl, gr);
    }
    soft_clip_buffer(out, frames);
}

/* ---- sender / receiver -------------------------------------------------- */

static void packet_send(const Source* srcs, int n, const Listener* l, Packet* p) {
    p->count = n;
    for (int k = 0; k < n; k++) {
        double az = binaural_azimuth(srcs[k].x, srcs[k].z, l->yaw);
        p->code[k] = sq_encode(az, source_distance(&srcs[k]));
    }
}

/* Reconstruct one window from the packet at its start and the packet at its
   end (`a` and `b` may be the same packet: hold). Azimuth and distance gain
   are interpolated per sample along the shortest arc, so the trajectory is
   continuous between packets and continuous across the +-pi seam. Note the
   shortest arc is ambiguous for a source that passes exactly over the head
   between two packets; a motion vector would be needed to resolve that, and
   at 60 Hz packet rates it cannot happen for real sources. */
static void packet_render(const Packet* a, const Packet* b, const float* src_gain,
                          const float* mono, int mono_len,
                          int base, int frames, int win, float* out) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    int n = a->count < b->count ? a->count : b->count;
    for (int k = 0; k < n; k++) {
        SQuantized qa = sq_decode(a->code[k]);
        SQuantized qb = sq_decode(b->code[k]);
        float ga = src_gain[k] * distance_gain_from((float)qa.dist);
        float gb = src_gain[k] * distance_gain_from((float)qb.dist);
        for (int i = 0; i < frames; i++) {
            float t = (float)i / (float)(win > 0 ? win : 1);
            float az = (float)sq_interp_az(qa.az, qb.az, t);
            int delay = binaural_itd_samples(az);
            float gl, gr;
            binaural_iid_gains(az, &gl, &gr);
            float g = ga + (gb - ga) * t;
            int j = base + i - delay;
            float s = (j >= 0 && j < mono_len) ? mono[j] : 0.0f;
            out[2*i + 0] += s * g * gl;
            out[2*i + 1] += s * g * gr;
        }
    }
    soft_clip_buffer(out, frames);
}

/* ---- tests -------------------------------------------------------------- */

/* The narrowest and widest azimuth bands in which both ITD and IID stay
   unchanged, over the whole sphere. The quantizer splits each band into 16
   sub-cells, so the worst-case azimuth error is at most one thirty-second of
   the widest band. The bands are non-uniform: the ITD/IID grids are uniform in
   sin(az), so they are ~1.8 deg wide near 0/180 deg and up to ~14.4 deg near
   the ears, which is where the worst case comes from. */
static void decision_interval_bounds(double* wmin, double* wmax) {
    double lo_w = 1e9, hi_w = 0.0;
    for (int k = -SQ_ITD_CODE_MAX; k <= SQ_ITD_CODE_MAX; k++) {
        double l1, h1, l2, h2;
        sq_itd_u_interval(k, &l1, &h1);
        for (int lv = 0; lv < SQ_IID_LEVELS; lv++) {
            sq_iid_u_interval(lv, &l2, &h2);
            double ulo = l1 > l2 ? l1 : l2;
            double uhi = h1 < h2 ? h1 : h2;
            if (!(ulo < uhi)) continue;
            double alo, ahi, blo, bhi;
            sq_az_bounds(ulo, uhi, 0, &alo, &ahi);
            sq_az_bounds(ulo, uhi, 1, &blo, &bhi);
            double w = fmax(ahi - alo, bhi - blo);
            if (w < lo_w) lo_w = w;
            if (w > hi_w) hi_w = w;
        }
    }
    *wmin = lo_w;
    *wmax = hi_w;
}

/* Control: a uniform azimuth grid with the same bits spent on angle and no
   explicit decision bits. Counts how often the decoded angle falls into a
   different ITD/IID cell than the true angle did. */
static void test_uniform_grid_comparison(int bits) {
    const int N = 2000000;
    int itd_flip = 0, iid_flip = 0, any_flip = 0;
    double step = 6.283185307179586 / (double)(1 << bits);
    for (int i = 0; i < N; i++) {
        double az = -3.14159265358979323846 + (i + 0.5) * (6.283185307179586 / N);
        double azq = lround(az / step) * step;
        if (azq >  3.14159265358979323846) azq -= 6.283185307179586;
        if (azq < -3.14159265358979323846) azq += 6.283185307179586;
        int a = sq_itd_code(azq) != sq_itd_code(az);
        int b = sq_iid_code(azq) != sq_iid_code(az);
        itd_flip += a;
        iid_flip += b;
        any_flip += (a || b);
    }
    printf("  uniform az grid %2d bits: itd_flips=%d iid_flips=%d any_flips=%d\n",
           bits, itd_flip, iid_flip, any_flip);
}

/* Round trip over the whole sphere and 7 decades of distance. */
static void test_roundtrip_sweep(void) {
    const int N = 2000000;
    double max_az_err = 0.0, max_dist_rel = 0.0, max_gain_rel = 0.0;
    int itd_bad = 0, iid_bad = 0, nan_seen = 0, seam_bad = 0;
    for (int i = 0; i < N; i++) {
        double az = -3.14159265358979323846 + (i + 0.5) * (6.283185307179586 / N);
        double d = 0.01 * pow(1.0e7, (double)i / (double)(N - 1));  /* 0.01 .. 1e5 m */
        int itd0 = sq_itd_code(az);
        int iid0 = sq_iid_code(az);
        SQuantized q = sq_decode(sq_encode(az, d));

        double e = fabs((double)q.az - az);
        if (e > 3.14159265358979323846) e = fabs(e - 6.283185307179586);
        if ((double)q.az != (double)q.az || q.dist != q.dist) nan_seen++;
        if (e > max_az_err) max_az_err = e;
        if (fabs((double)q.az) > 3.14159265358979323846) seam_bad++;
        /* re-run the decisions on the reconstructed angle with the shared code */
        if (sq_itd_code(q.az) != itd0) itd_bad++;
        if (sq_iid_code(q.az) != iid0) iid_bad++;
        double dr = fabs(q.dist - d) / d;
        if (dr > max_dist_rel) max_dist_rel = dr;
        double gr = fabs(distance_gain_from((float)q.dist) - distance_gain_from((float)d))
                  * (1.0f + (float)d);
        if (gr > max_gain_rel) max_gain_rel = gr;
    }
    printf("roundtrip: max_az_err=%.4f deg  max_dist_rel=%.5f%%  max_gain_rel=%.5f%%\n",
           max_az_err * 180.0 / 3.14159265358979323846, max_dist_rel * 100.0,
           max_gain_rel * 100.0);
    printf("roundtrip: itd_flips=%d iid_flips=%d nan=%d |az|>pi=%d\n",
           itd_bad, iid_bad, nan_seen, seam_bad);
    printf("roundtrip_ok=%d\n",
           max_az_err * 180.0 / 3.14159265358979323846 < 1.0 &&
           itd_bad == 0 && iid_bad == 0 && !nan_seen && seam_bad == 0);
}

/* Comparison table: the same angle goes through (a) the float render's
   decision helpers and (b) the 31-bit code, with the azimuth error that the
   code costs. */
static void test_decision_table(void) {
    static const double deg[] = { -179.0, -170.0, -135.0, -91.0, -90.0, -60.0,
                                  -10.0, 0.0, 10.0, 60.0, 89.0, 90.0, 91.0,
                                  135.0, 170.0, 179.0 };
    int n = (int)(sizeof(deg) / sizeof(deg[0]));
    int mism = 0;
    printf("  az_deg | float itd/iid | code itd/iid | az' err\n");
    for (int i = 0; i < n; i++) {
        double az = deg[i] * 3.14159265358979323846 / 180.0;
        SQuantized q = sq_decode(sq_encode(az, 5.0));
        int fi = binaural_itd_samples((float)az);
        int fl = binaural_iid_level((float)az);
        double err = fabs((double)q.az - az) * 180.0 / 3.14159265358979323846;
        printf("  %7.1f | %3d/%2d        | %3d/%2d        | %.3f deg\n",
               deg[i], fi, fl, q.itd, q.iid, err);
        if (fi != q.itd || fl != q.iid) mism++;
    }
    printf("decision_mismatch=%d/%d\n", mism, n);
}

/* A source orbits the listener at 0.25 deg per packet, one packet per window.
   Two things are checked:
   1. continuity of the quantized+interpolated path: sampled per sample, the
      step is bounded by the real motion plus one quantizer sub-cell, no jump
      at the +-pi seam, ITD changes by at most one sample per step;
   2. bounded deviation from the true path: |decoded(t) - true(t)| stays under
      the quantizer bound at every sample of the whole orbit, not just at the
      packet instants, and does not accumulate. */
static void test_motion_continuity(int win) {
    const int packets = 1440;                       /* full circle */
    const int frames = win;
    float mono[4096];
    for (int i = 0; i < frames && i < 4096; i++) mono[i] = (i == 32) ? 1.0f : 0.0f;
    static float out[4096 * 2];

    double prev_dec = 0.0, max_step = 0.0, max_dev = 0.0;
    int wall_jump = 0, nan_seen = 0, itd_step_bad = 0, prev_itd = 0;
    Packet pa, pb;
    Listener l = { 0.0f };
    float gain = 1.0f;

    for (int p = 0; p < packets; p++) {
        float a0 = (float)(p * 0.25 * 3.14159265358979323846 / 180.0);
        float a1 = (float)((p + 1) * 0.25 * 3.14159265358979323846 / 180.0);
        Source s0 = { 2.0f * sinf(a0), 0.0f, -2.0f * cosf(a0), 1.0f };
        Source s1 = { 2.0f * sinf(a1), 0.0f, -2.0f * cosf(a1), 1.0f };
        packet_send(&s0, 1, &l, &pa);
        packet_send(&s1, 1, &l, &pb);
        packet_render(&pa, &pb, &gain, mono, frames, 0, frames, win, out);

        double qa = sq_decode(pa.code[0]).az;
        double qb = sq_decode(pb.code[0]).az;
        for (int i = 0; i < frames; i++) {
            double t = (double)i / (double)win;
            double dec = sq_interp_az(qa, qb, t);
            if (dec != dec) { nan_seen++; continue; }
            /* true float azimuth of the source at this same instant */
            float at = (float)(a0 + (a1 - a0) * (float)t);
            Source st = { 2.0f * sinf(at), 0.0f, -2.0f * cosf(at), 1.0f };
            double tru = binaural_azimuth(st.x, st.z, l.yaw);
            double dev = fabs(dec - tru);
            if (dev > 3.14) dev = 6.283185307179586 - dev;
            if (dev > max_dev) max_dev = dev;
            if (p > 0 || i > 0) {
                double step = fabs(dec - prev_dec);
                if (step > 3.14) step = 6.283185307179586 - step;  /* seam, not a jump */
                if (step > max_step) max_step = step;
                if (step > 3.14159265358979323846 / 2) wall_jump++;
                int itd = binaural_itd_samples((float)dec);
                int ds = itd - prev_itd;
                if (ds > 1 || ds < -1) itd_step_bad++;
                prev_itd = itd;
            } else {
                prev_itd = binaural_itd_samples((float)dec);
            }
            prev_dec = dec;
        }
    }
    double rad2deg = 180.0 / 3.14159265358979323846;
    double step_deg = max_step * rad2deg;
    double dev_deg = max_dev * rad2deg;
    /* per-sample motion plus the chord-to-arc sagitta of a 0.25 deg segment */
    double step_bound = 0.25 / (double)win + 8.277 / 16.0;
    /* half a sub-cell, plus the sagitta of the true arc over one packet */
    double dev_bound = 8.277 / 16.0 / 2.0 + 0.25 * 0.25 / 8.0 * 0.0174533;
    printf("continuity win=%d: step=%.4f deg (bound %.3f) dev_from_float=%.4f deg "
           "(bound %.3f) seam_jump=%d nan=%d itd_step>1=%d ok=%d\n",
           win, step_deg, step_bound, dev_deg, dev_bound,
           wall_jump, nan_seen, itd_step_bad,
           wall_jump == 0 && nan_seen == 0 && itd_step_bad == 0 &&
           step_deg <= step_bound && dev_deg <= dev_bound);
}

/* Bit-in/bit-out check of the audio path: static source, no interpolation, so
   the only differences are the quantizer's sub-cell angle and distance code. */
static void test_render_ab(void) {
    const int frames = 256;
    float mono[frames], wave[frames * 2], wq[frames * 2];
    for (int i = 0; i < frames; i++) mono[i] = (i == 64) ? 1.0f : 0.0f;

    Listener l = { 0.0f };
    Packet pkt;
    float gain = 1.0f;
    int total_flip = 0;
    double max_diff = 0.0;

    Source probe[7] = {
        { 3.0f, 0.0f, 0.0f, 1.0f },
        { -3.0f, 0.0f, 0.0f, 1.0f },
        { 0.0f, 0.0f, -4.0f, 1.0f },
        { 2.0f, 0.0f, 2.0f, 1.0f },
        { -2.0f, 0.0f, 2.0f, 1.0f },
        { 6.0f, 1.0f, -8.0f, 1.0f },
        { 0.3f, 0.0f, 30.0f, 1.0f },
    };
    for (int k = 0; k < 7; k++) {
        render(&probe[k], 1, &l, mono, frames, wave);
        packet_send(&probe[k], 1, &l, &pkt);
        packet_render(&pkt, &pkt, &gain, mono, frames, 0, frames, frames, wq);
        double md = 0.0;
        for (int i = 0; i < frames * 2; i++) {
            double d = fabs((double)wave[i] - (double)wq[i]);
            if (d > md) md = d;
        }
        if (md > max_diff) max_diff = md;
        SQuantized q = sq_decode(pkt.code[0]);
        float az = binaural_azimuth(probe[k].x, probe[k].z, l.yaw);
        int fi = binaural_itd_samples(az), fl = binaural_iid_level(az);
        int flips = (fi != q.itd) + (fl != q.iid);
        total_flip += flips;
        double az_err = fabs((double)q.az - az) * 180.0 / 3.14159265358979323846;
        printf("  src(%5.1f,%5.1f,%5.1f): itd %d->%d iid %d->%d az_err=%.3f deg "
               "peak_diff=%.2e\n", probe[k].x, probe[k].y, probe[k].z,
               fi, q.itd, fl, q.iid, az_err, md);
    }
    printf("render_ab: decision_flips=%d max_sample_diff=%.3e ab_ok=%d\n",
           total_flip, max_diff, total_flip == 0);
}

int main(void) {
    const int frames = 256;
    float mono[MAX_FRAMES];
    float out[MAX_FRAMES * 2];
    for (int i = 0; i < 256; i++) mono[i] = (i == 64) ? 1.0f : 0.0f;  /* impulse at 64 */

    /* ---- existing behaviour demos (float path) ---- */
    Listener l = { 0.0f };
    Source right = { 3.0f, 0.0f, 0.0f, 1.0f };
    render(&right, 1, &l, mono, frames, out);
    int best_l = 0, best_r = 0;
    for (int i = 1; i < frames; i++) {
        if (fabsf(out[2*i+0]) > fabsf(out[2*best_l+0])) best_l = i;
        if (fabsf(out[2*i+1]) > fabsf(out[2*best_r+1])) best_r = i;
    }
    printf("right_source peakL=%d peakR=%d (impulse@64)\n", best_l, best_r);

    Source left = { -3.0f, 0.0f, 0.0f, 1.0f };
    render(&left, 1, &l, mono, frames, out);
    int bl = 0, br = 0;
    for (int i = 1; i < frames; i++) {
        if (fabsf(out[2*i+0]) > fabsf(out[2*bl+0])) bl = i;
        if (fabsf(out[2*i+1]) > fabsf(out[2*br+1])) br = i;
    }
    printf("left_source peakL=%d peakR=%d (impulse@64)\n", bl, br);

    Source near = { 0.0f, 0.0f, -1.0f, 1.0f };
    Source far  = { 0.0f, 0.0f, -9.0f, 1.0f };
    render(&near, 1, &l, mono, frames, out);
    float pn = 0.0f;
    for (int i = 0; i < frames * 2; i++) if (fabsf(out[i]) > pn) pn = fabsf(out[i]);
    render(&far, 1, &l, mono, frames, out);
    float pf = 0.0f;
    for (int i = 0; i < frames * 2; i++) if (fabsf(out[i]) > pf) pf = fabsf(out[i]);
    printf("attenuation near=%.4f far=%.4f ratio=%.3f\n", pn, pf, pf / pn);

    Source many[4] = { {1,0,0,1}, {-1,0,0,1}, {0,0,-1,1}, {0,0,1,1} };
    float loud[MAX_FRAMES];
    for (int i = 0; i < frames; i++) loud[i] = 0.9f;
    render(many, 4, &l, loud, frames, out);
    float pk = 0.0f;
    for (int i = 0; i < frames * 2; i++) if (fabsf(out[i]) > pk) pk = fabsf(out[i]);
    printf("mix_peak=%.4f (soft clip 上限 1.0)\n", pk);

    /* ---- quantization ---- */
    double wmin = 0.0, wmax = 0.0;
    decision_interval_bounds(&wmin, &wmax);
    printf("\ncodec: %d bits/source, %d bits/packet; decision band %.3f..%.3f deg, "
           "16 sub-cells -> worst-case az error %.3f deg\n",
           SQ_CODE_BITS, SQ_PACKET_BITS,
           wmin * 180.0 / 3.14159265358979323846,
           wmax * 180.0 / 3.14159265358979323846,
           wmax / SQ_AZ_SUB * 180.0 / 3.14159265358979323846);
    test_roundtrip_sweep();
    printf("\nquantized vs float decisions:\n");
    test_decision_table();
    printf("\nuniform-grid control (same angle, no explicit decision bits):\n");
    test_uniform_grid_comparison(10);
    test_uniform_grid_comparison(11);
    printf("\nquantized vs float audio (static source, no interpolation):\n");
    test_render_ab();
    printf("\nmotion continuity (one packet per window, one packet per window, linear interpolation):\n");
    test_motion_continuity(48);
    test_motion_continuity(480);
    test_motion_continuity(1920);
    return 0;
}
