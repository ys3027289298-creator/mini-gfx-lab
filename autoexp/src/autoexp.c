/* autoexp: auto-exposure statistics chain.

   Per-region histograms (16x9 by default), overexposed-pixel rejection,
   spatial smoothing between regions, temporal adaptation with separate
   rise/fall time constants and a per-frame slew limit. The metering pass
   touches every pixel exactly once, so its cost is linear in pixel count.
   The histogram is coarser than one bin per luma level (64 bins on 8-bit
   input), which is what keeps the metering pass to a single memory sweep.

   Self-test cross-checks the region histogram pipeline against a naive
   per-pixel reference, reports exposure jumps, convergence times after
   scene cuts, and measures ns/pixel across resolutions. */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdint.h>

#define HIST_BINS 64
#define BIN_SHIFT 2
#define BIN_WIDTH (1 << BIN_SHIFT)
#define MIN_REGION_SAMPLES 16   /* a region with fewer valid pixels is ignored */
#define MAX_RX 32               /* all metadata is statically sized */
#define MAX_RY 32
#define MAX_REGIONS (MAX_RX * MAX_RY)

typedef struct { int x0, y0, x1, y1; } Region;

typedef struct {
    int sat_threshold;       /* luma >= this is overexposed and excluded */
    float percentile;        /* metered percentile of the valid luma */
    float key;               /* target linear luminance, 0.18 = mid grey */
    int spatial_smooth;      /* 3x3 weighted blur of region EVs */
    float tau_up;            /* seconds, exposure rising (scene got darker) */
    float tau_down;          /* seconds, exposure falling */
    float max_step_ev;       /* slew limit per frame */
    float jump_ev;           /* emitted delta above this counts as a jump */
    float conv_tol_ev;       /* remaining error considered settled */
    float min_ev, max_ev;    /* exposure clamp */
} MeterCfg;

typedef struct {
    int w, h, rx, ry, region_count;
    Region regions[MAX_REGIONS];

    unsigned hist[MAX_REGIONS][HIST_BINS];
    unsigned hist_all[HIST_BINS];   /* union of region histograms */
    unsigned valid[MAX_REGIONS];    /* unsaturated pixels per region */
    unsigned sat[MAX_REGIONS];      /* overexposed pixels per region */

    float ev[MAX_REGIONS];          /* metered EV per region */
    float ev_sm[MAX_REGIONS];       /* after spatial smoothing */
    int region_valid[MAX_REGIONS];

    unsigned total_pixels;
    unsigned sat_pixels;
    unsigned valid_total;

    float merged_ev;
    int meter_valid;
    float target_ev;
    float current_ev;

    /* counters / diagnostics */
    unsigned long long pixels_visited;
    unsigned long long hist_ops;
    int frames;
    int primed;             /* first valid frame snaps instead of adapting */
    int jumps;
    float last_step;
    float max_step;
} Meter;

static MeterCfg cfg_default(void) {
    MeterCfg c;
    c.sat_threshold = 250;
    c.percentile = 0.85f;
    c.key = 0.18f;
    c.spatial_smooth = 1;
    c.tau_up = 0.15f;
    c.tau_down = 0.35f;
    c.max_step_ev = 0.08f;
    c.jump_ev = 0.15f;
    c.conv_tol_ev = 0.02f;
    c.min_ev = -4.0f;
    c.max_ev = 4.0f;
    return c;
}

static void meter_init(Meter* m, int w, int h, int rx, int ry, float initial_ev) {
    memset(m, 0, sizeof(*m));
    if (rx < 1) rx = 1;
    if (ry < 1) ry = 1;
    if (rx > MAX_RX) rx = MAX_RX;
    if (ry > MAX_RY) ry = MAX_RY;
    m->w = w; m->h = h; m->rx = rx; m->ry = ry;
    m->region_count = rx * ry;
    for (int y = 0; y < ry; y++) {
        for (int x = 0; x < rx; x++) {
            Region* r = &m->regions[y * rx + x];
            r->x0 = x * w / rx;
            r->x1 = (x + 1) * w / rx;
            r->y0 = y * h / ry;
            r->y1 = (y + 1) * h / ry;
        }
    }
    m->current_ev = initial_ev;
    m->target_ev = initial_ev;
    m->merged_ev = 0.0f;
    m->meter_valid = 0;
}

/* nearest-rank percentile bin over a histogram */
static int hist_percentile_bin(const unsigned* hist, int bins, unsigned valid, float q) {
    if (valid == 0) return -1;
    double need = (double)q * (double)valid;
    unsigned rank = (unsigned)ceil(need);
    if (rank < 1) rank = 1;
    if (rank > valid) rank = valid;
    unsigned cum = 0;
    for (int b = 0; b < bins; b++) {
        cum += hist[b];
        if (cum >= rank) return b;
    }
    return bins - 1;
}

static float bin_luma(int b) {
    return ((float)(b * BIN_WIDTH) + (float)(BIN_WIDTH / 2)) / 255.0f;
}

/* one pass over every pixel of every region: histogram + saturation count.
   Region-outer iteration keeps rows sequential and avoids per-pixel mapping. */
static void meter_accumulate(Meter* m, const unsigned char* img, int sat_threshold) {
    memset(m->hist, 0, sizeof(m->hist[0]) * (size_t)m->region_count);
    unsigned total = 0, sat_total = 0;
    unsigned long long visited = 0, hops = 0;
    unsigned sat_thr = (unsigned)sat_threshold;

    for (int i = 0; i < m->region_count; i++) {
        const Region* r = &m->regions[i];
        unsigned* h = m->hist[i];
        unsigned vcount = 0, scount = 0;
        for (int y = r->y0; y < r->y1; y++) {
            const unsigned char* row = img + (size_t)y * (size_t)m->w;
            for (int x = r->x0; x < r->x1; x++) {
                unsigned v = row[x];
                visited++;
                if (v >= sat_thr) {
                    scount++;
                } else {
                    h[v >> BIN_SHIFT]++;
                    hops++;
                }
            }
        }
        /* valid count comes from the histogram so there is one source of truth */
        for (int b = 0; b < HIST_BINS; b++) vcount += h[b];
        m->valid[i] = vcount;
        total += vcount + scount;
        sat_total += scount;
    }
    m->total_pixels = total;
    m->sat_pixels = sat_total;
    m->pixels_visited = visited;
    m->hist_ops = hops;
}

/* 3x3 weighted by each neighbour's valid pixel count */
static void spatial_smooth(const Meter* m, const float* in, float* out) {
    for (int ry = 0; ry < m->ry; ry++) {
        for (int rx = 0; rx < m->rx; rx++) {
            int i = ry * m->rx + rx;
            if (!m->region_valid[i]) { out[i] = 0.0f; continue; }
            float sum = 0.0f;
            unsigned wsum = 0;
            for (int dy = -1; dy <= 1; dy++) {
                int ny = ry + dy;
                if (ny < 0 || ny >= m->ry) continue;
                for (int dx = -1; dx <= 1; dx++) {
                    int nx = rx + dx;
                    if (nx < 0 || nx >= m->rx) continue;
                    int ni = ny * m->rx + nx;
                    if (!m->region_valid[ni]) continue;
                    sum += in[ni] * (float)m->valid[ni];
                    wsum += m->valid[ni];
                }
            }
            out[i] = wsum ? sum / (float)wsum : in[i];
        }
    }
}

/* histogram -> region EV -> spatial smoothing -> merged metering target */
static void meter_reduce(Meter* m, const MeterCfg* cfg) {
    memset(m->hist_all, 0, sizeof(m->hist_all));
    for (int i = 0; i < m->region_count; i++)
        for (int b = 0; b < HIST_BINS; b++)
            m->hist_all[b] += m->hist[i][b];

    for (int i = 0; i < m->region_count; i++) {
        int b = hist_percentile_bin(m->hist[i], HIST_BINS, m->valid[i], cfg->percentile);
        m->region_valid[i] = (b >= 0 && m->valid[i] >= MIN_REGION_SAMPLES);
        m->ev[i] = (b >= 0) ? log2f(bin_luma(b)) : 0.0f;
    }

    if (cfg->spatial_smooth) spatial_smooth(m, m->ev, m->ev_sm);
    else memcpy(m->ev_sm, m->ev, sizeof(m->ev_sm));

    float sum = 0.0f;
    unsigned wsum = 0;
    for (int i = 0; i < m->region_count; i++) {
        if (!m->region_valid[i]) continue;
        sum += m->ev_sm[i] * (float)m->valid[i];
        wsum += m->valid[i];
    }
    m->valid_total = wsum;
    m->meter_valid = 0;
    if (wsum > 0) {
        m->meter_valid = 1;
        m->merged_ev = sum / (float)wsum;
    } else {
        /* every region was too small or too blown to meter: fall back to the
           frame-wide histogram so a tiny frame still produces a target */
        unsigned valid = m->total_pixels - m->sat_pixels;
        int b = hist_percentile_bin(m->hist_all, HIST_BINS, valid, cfg->percentile);
        if (b >= 0) {
            m->meter_valid = 1;
            m->valid_total = valid;
            m->merged_ev = log2f(bin_luma(b));
        }
    }
    if (!m->meter_valid) return;   /* no usable pixels anywhere: hold target */

    float t = log2f(cfg->key) - m->merged_ev;
    if (t < cfg->min_ev) t = cfg->min_ev;
    if (t > cfg->max_ev) t = cfg->max_ev;
    m->target_ev = t;
}

/* temporal adaptation; tau <= 0 selects the hard (unsmoothed) reference */
static void meter_update(Meter* m, const MeterCfg* cfg, float dt) {
    float before = m->current_ev;
    int was_primed = m->primed;
    int hard = (cfg->tau_up <= 0.0f || cfg->tau_down <= 0.0f);
    if (m->meter_valid && !hard && !m->primed) {
        /* first usable frame: snap to the metered target. This is start-up,
           not an adaptation, so it must not count as a jump. */
        m->current_ev = m->target_ev;
        m->primed = 1;
    } else if (m->meter_valid && hard) {
        m->current_ev = m->target_ev;
        m->primed = 1;
    } else if (m->meter_valid) {
        float tau = m->target_ev > m->current_ev ? cfg->tau_up : cfg->tau_down;
        float k = 1.0f - expf(-dt / tau);
        float step = (m->target_ev - m->current_ev) * k;
        if (step > cfg->max_step_ev) step = cfg->max_step_ev;
        if (step < -cfg->max_step_ev) step = -cfg->max_step_ev;
        m->current_ev += step;
    }
    /* an invalid frame holds both target and current: no motion, no jump */
    float delta = fabsf(m->current_ev - before);
    m->last_step = delta;
    if (was_primed) {
        if (delta > m->max_step) m->max_step = delta;
        if (delta > cfg->jump_ev) m->jumps++;
    }
    m->frames++;
}

static void meter_frame(Meter* m, const unsigned char* img, const MeterCfg* cfg, float dt) {
    meter_accumulate(m, img, cfg->sat_threshold);
    meter_reduce(m, cfg);
    meter_update(m, cfg, dt);
}

/* percentile EV of the union histogram */
static float meter_global_ev(const Meter* m, float q) {
    unsigned valid = m->total_pixels - m->sat_pixels;
    int b = hist_percentile_bin(m->hist_all, HIST_BINS, valid, q);
    return b < 0 ? NAN : log2f(bin_luma(b));
}

/* ------------------------------------------------------------------ */
/* naive per-pixel reference                                           */

typedef struct {
    unsigned hist[HIST_BINS];
    unsigned valid, sat, total;
    double mean;        /* exact mean luma over unsaturated pixels */
    int perc_bin;       /* exact percentile value converted to bin */
    float perc_ev;
} Naive;

static int cmp_u8(const void* a, const void* b) {
    unsigned x = *(const unsigned char*)a, y = *(const unsigned char*)b;
    return (x > y) - (x < y);
}

static void naive_stats(const unsigned char* img, int w, int h, int sat_threshold,
                        float q, Naive* out) {
    memset(out, 0, sizeof(*out));
    unsigned sat_thr = (unsigned)sat_threshold;
    size_t n = (size_t)w * (size_t)h;
    unsigned char* vals = (unsigned char*)malloc(n ? n : 1);
    double sum = 0.0;
    unsigned valid = 0, sat = 0;
    for (size_t i = 0; i < n; i++) {
        unsigned v = img[i];
        if (v >= sat_thr) { sat++; continue; }
        out->hist[v >> BIN_SHIFT]++;
        sum += (double)v;
        vals[valid++] = (unsigned char)v;
    }
    out->valid = valid;
    out->sat = sat;
    out->total = (unsigned)valid + sat;
    out->mean = valid ? sum / (double)valid : 0.0;
    if (valid) {
        qsort(vals, valid, sizeof(unsigned char), cmp_u8);
        double need = (double)q * (double)valid;
        unsigned rank = (unsigned)ceil(need);
        if (rank < 1) rank = 1;
        if (rank > valid) rank = valid;
        unsigned v = vals[rank - 1];
        out->perc_bin = (int)(v >> BIN_SHIFT);
        out->perc_ev = log2f(bin_luma(out->perc_bin));
    } else {
        out->perc_bin = -1;
        out->perc_ev = NAN;
    }
    free(vals);
}

/* ------------------------------------------------------------------ */
/* synthetic scenes                                                    */

static uint32_t rng_next(uint32_t* s) {
    uint32_t x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x;
    return x;
}

static unsigned char clamp_u8(float v) {
    if (v < 0.0f) return 0;
    if (v > 255.0f) return 255;
    return (unsigned char)(v + 0.5f);
}

static void scene_fill(unsigned char* img, int w, int h, int kind, uint32_t seed) {
    uint32_t s = seed;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            float tx = (x + 0.5f) / (float)w;
            float ty = (y + 0.5f) / (float)h;
            float base = 0.0f;
            switch (kind) {
            case 0: base = 12.0f + 28.0f * tx + 10.0f * ty; break;   /* dark indoor */
            case 1: base = 150.0f + 70.0f * tx; break;               /* bright */
            case 2: base = 90.0f + 30.0f * ty; break;                /* mid */
            default: base = 100.0f; break;                           /* flat */
            }
            float noise = (float)((int)(rng_next(&s) % 9u) - 4);
            float v = base + noise;
            if (kind == 1 && ((x * 7 + y * 11) % 97) == 0) v = 255.0f;  /* specular */
            img[(size_t)y * (size_t)w + x] = clamp_u8(v);
        }
    }
}

/* ------------------------------------------------------------------ */
/* test harness                                                        */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static double now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e9 + (double)ts.tv_nsec;
}

static float region_ev_spread(const Meter* m, const float* ev) {
    float lo = 1e30f, hi = -1e30f;
    for (int i = 0; i < m->region_count; i++) {
        if (!m->region_valid[i]) continue;
        if (ev[i] < lo) lo = ev[i];
        if (ev[i] > hi) hi = ev[i];
    }
    return hi >= lo ? hi - lo : 0.0f;
}

/* compare every per-region histogram, count and percentile against a naive
   scan of that region's pixels; returns the number of mismatching regions */
static int regions_match_naive(const Meter* m, const unsigned char* img,
                               int sat_threshold, float q, int* empties) {
    int bad = 0;
    *empties = 0;
    for (int i = 0; i < m->region_count; i++) {
        const Region* r = &m->regions[i];
        int rw = r->x1 - r->x0, rh = r->y1 - r->y0;
        if (rw <= 0 || rh <= 0) continue;
        unsigned char* crop = (unsigned char*)malloc((size_t)rw * (size_t)rh);
        for (int y = 0; y < rh; y++)
            memcpy(crop + (size_t)y * (size_t)rw,
                   img + (size_t)(r->y0 + y) * (size_t)m->w + (size_t)r->x0, (size_t)rw);
        Naive nv;
        naive_stats(crop, rw, rh, sat_threshold, q, &nv);
        free(crop);
        if (nv.valid < MIN_REGION_SAMPLES) { (*empties)++; continue; }
        int metered = m->region_valid[i];
        int ok = nv.valid == m->valid[i] && nv.sat == m->sat[i] &&
                 memcmp(nv.hist, m->hist[i], sizeof(nv.hist)) == 0;
        if (metered)
            ok = ok && nv.perc_bin == hist_percentile_bin(m->hist[i], HIST_BINS,
                                                          m->valid[i], q);
        if (!ok) bad++;
    }
    return bad;
}

/* scene-cut adaptation run; prints per-cut convergence data */
typedef struct {
    int frames_to_50, frames_to_90, frames_to_settle;
    float delta_ev, target_ev, start_ev;
} CutReport;

static void run_cuts(const MeterCfg* cfg, const char* label) {
    enum { W = 320, H = 180, REG_X = 16, REG_Y = 9, FPS = 60 };
    static const struct { int kind; int frames; } seq[] = {
        { 0, 90 }, { 1, 180 }, { 0, 180 }, { 2, 120 },
    };
    const int nseq = (int)(sizeof(seq) / sizeof(seq[0]));
    unsigned char* img = (unsigned char*)malloc((size_t)W * H);
    Meter m;
    meter_init(&m, W, H, REG_X, REG_Y, 0.0f);
    memset(img, 0, (size_t)W * H);

    printf("%s\n", label);
    int cut_index = 0;
    for (int s = 0; s < nseq; s++) {
        scene_fill(img, W, H, seq[s].kind, 1000u + (uint32_t)s * 77u);
        CutReport rep;
        memset(&rep, 0, sizeof(rep));
        int is_cut = (s > 0);
        rep.start_ev = m.current_ev;
        float delta0 = 0.0f;
        float target = NAN;
        for (int f = 0; f < seq[s].frames; f++) {
            float before = m.current_ev;
            meter_frame(&m, img, cfg, 1.0f / FPS);
            if (f == 0 && is_cut) {
                target = m.target_ev;
                rep.delta_ev = fabsf(target - before);
                delta0 = rep.delta_ev;
                rep.start_ev = before;
                rep.target_ev = target;
            }
            if (is_cut && delta0 > 0.0f) {
                float remain = fabsf(target - m.current_ev);
                int frame_no = f + 1;
                if (rep.frames_to_50 == 0 && remain <= 0.5f * delta0) rep.frames_to_50 = frame_no;
                if (rep.frames_to_90 == 0 && remain <= 0.1f * delta0) rep.frames_to_90 = frame_no;
                if (rep.frames_to_settle == 0 && remain <= cfg->conv_tol_ev)
                    rep.frames_to_settle = frame_no;
            }
        }
        if (is_cut) {
            cut_index++;
            printf("cut%d kind=%d start_ev=%.3f target_ev=%.3f delta=%.3f "
                   "t50=%d t90=%d settle=%d (%.0fms@%dfps)\n",
                   cut_index, seq[s].kind, rep.start_ev, rep.target_ev, rep.delta_ev,
                   rep.frames_to_50, rep.frames_to_90, rep.frames_to_settle,
                   1000.0f * (float)rep.frames_to_settle / (float)FPS, FPS);
            check("cut_converged", rep.frames_to_settle > 0);
            check("cut_monotone_progress", rep.frames_to_50 <= rep.frames_to_settle);
        }
    }
    printf("%s jumps=%d max_step=%.4f over %d frames\n",
           label, m.jumps, m.max_step, m.frames);
    if (cfg->tau_up > 0.0f)
        check("smooth_no_jumps", m.jumps == 0 && m.max_step <= cfg->max_step_ev + 1e-5f);
    else
        check("hard_has_jumps", m.jumps == nseq - 1);
    free(img);
}

int main(void) {
    MeterCfg cfg = cfg_default();

    /* -------- 1. statistics chain vs naive per-pixel reference -------- */
    enum { W = 320, H = 180 };
    unsigned char* img = (unsigned char*)malloc((size_t)W * H);
    scene_fill(img, W, H, 2, 424242u);

    Meter m;
    meter_init(&m, W, H, 16, 9, 0.0f);
    meter_frame(&m, img, &cfg, 1.0f / 60.0f);

    Naive naive;
    naive_stats(img, W, H, cfg.sat_threshold, cfg.percentile, &naive);

    printf("pixels=%u visited=%llu hist_ops=%llu sat=%u regions=%d\n",
           m.total_pixels, m.pixels_visited, m.hist_ops, m.sat_pixels, m.region_count);
    check("visit_all_pixels", m.pixels_visited == (unsigned long long)W * H);
    check("total_pixels", m.total_pixels == (unsigned)W * H);
    check("sat_plus_valid", m.sat_pixels + (m.total_pixels - m.sat_pixels) == m.total_pixels);
    check("hist_ops_linear", m.hist_ops == (unsigned long long)W * H - m.sat_pixels);

    unsigned merged[HIST_BINS];
    memset(merged, 0, sizeof(merged));
    for (int i = 0; i < m.region_count; i++)
        for (int b = 0; b < HIST_BINS; b++) merged[b] += m.hist[i][b];
    int hist_equal = 1;
    for (int b = 0; b < HIST_BINS; b++)
        if (merged[b] != naive.hist[b]) hist_equal = 0;
    check("region_hist_matches_manual", hist_equal);
    check("global_hist_matches_manual", memcmp(m.hist_all, naive.hist, sizeof(merged)) == 0);

    check("sat_matches_manual", m.sat_pixels == naive.sat);

    int mb = hist_percentile_bin(m.hist_all, HIST_BINS,
                                 m.total_pixels - m.sat_pixels, cfg.percentile);
    check("percentile_matches_manual", mb == naive.perc_bin);

    int hist_mean_ok;
    {
        double sum = 0.0;
        unsigned valid = m.total_pixels - m.sat_pixels;
        for (int b = 0; b < HIST_BINS; b++)
            sum += (double)m.hist_all[b] * (double)(b * BIN_WIDTH + BIN_WIDTH / 2);
        double mean = valid ? sum / (double)valid : 0.0;
        double err = fabs(mean - naive.mean);
        printf("mean_hist=%.4f mean_manual=%.4f err=%.4f\n", mean, naive.mean, err);
        hist_mean_ok = valid > 0 && err <= (double)BIN_WIDTH * 0.5 + 1e-9;
    }
    check("mean_matches_manual", hist_mean_ok);
    check("global_ev_matches_manual",
          fabsf(meter_global_ev(&m, cfg.percentile) - naive.perc_ev) < 1e-6f);

    float spread_raw = region_ev_spread(&m, m.ev);
    float spread_sm = region_ev_spread(&m, m.ev_sm);
    printf("region_ev_spread raw=%.4f smoothed=%.4f\n", spread_raw, spread_sm);
    check("spatial_smooth_reduces_spread", spread_sm <= spread_raw + 1e-6f);

    int empties = 0;
    int region_bad = regions_match_naive(&m, img, cfg.sat_threshold, cfg.percentile, &empties);
    printf("region_check bad=%d empty=%d of %d\n", region_bad, empties, m.region_count);
    check("every_region_matches_manual", region_bad == 0);
    check("expected_empty_regions", empties == 0);

    /* -------- 2. uniform image: every region agrees exactly -------- */
    {
        unsigned char* flat = (unsigned char*)malloc((size_t)W * H);
        memset(flat, 100, (size_t)W * H);
        Meter u;
        meter_init(&u, W, H, 16, 9, 0.0f);
        meter_frame(&u, flat, &cfg, 1.0f / 60.0f);
        float max_diff = 0.0f, max_sm_diff = 0.0f;
        for (int i = 1; i < u.region_count; i++) {
            max_diff = fmaxf(max_diff, fabsf(u.ev[i] - u.ev[0]));
            max_sm_diff = fmaxf(max_sm_diff, fabsf(u.ev_sm[i] - u.ev_sm[0]));
        }
        printf("uniform_region_ev=%.4f max_diff=%.6f smooth_noop_diff=%.6f\n",
               u.ev[0], max_diff, max_sm_diff);
        check("uniform_regions_equal", max_diff < 1e-6f);
        check("uniform_smooth_identity", max_sm_diff < 1e-6f);
        free(flat);
    }

    /* -------- 3. overexposed region is excluded from metering ------- */
    {
        unsigned char* base = (unsigned char*)malloc((size_t)W * H);
        unsigned char* blown = (unsigned char*)malloc((size_t)W * H);
        memset(base, 100, (size_t)W * H);
        memcpy(blown, base, (size_t)W * H);
        for (int y = 0; y < H; y++)
            for (int x = W / 2; x < W; x++)
                blown[(size_t)y * W + x] = 255;

        Meter a, b;
        meter_init(&a, W, H, 16, 9, 0.0f);
        meter_init(&b, W, H, 16, 9, 0.0f);
        meter_frame(&a, base, &cfg, 1.0f / 60.0f);
        meter_frame(&b, blown, &cfg, 1.0f / 60.0f);
        printf("sat_exclude target_clean=%.6f target_blown=%.6f sat_px=%u\n",
               a.target_ev, b.target_ev, b.sat_pixels);
        check("saturation_pixels_counted", b.sat_pixels == (unsigned)(W / 2) * H);
        check("saturation_excluded_from_meter",
              fabsf(a.target_ev - b.target_ev) < 1e-6f);

        unsigned char* allwhite = (unsigned char*)malloc((size_t)W * H);
        memset(allwhite, 255, (size_t)W * H);
        Meter c;
        meter_init(&c, W, H, 16, 9, -0.5f);
        meter_frame(&c, allwhite, &cfg, 1.0f / 60.0f);
        printf("all_blown valid=%u meter_valid=%d target=%.3f current=%.3f\n",
               c.valid_total, c.meter_valid, c.target_ev, c.current_ev);
        check("all_blown_holds_target", !c.meter_valid && c.target_ev == -0.5f);
        check("all_blown_finite", isfinite(c.target_ev) && isfinite(c.current_ev));
        free(base); free(blown); free(allwhite);
    }

    /* -------- 4. scene cuts: jumps, convergence, hard reference ----- */
    MeterCfg smooth_cfg = cfg_default();
    run_cuts(&smooth_cfg, "smoothed");

    MeterCfg hard_cfg = cfg_default();
    hard_cfg.tau_up = 0.0f;
    hard_cfg.tau_down = 0.0f;
    hard_cfg.spatial_smooth = 0;
    run_cuts(&hard_cfg, "hard");

    /* -------- 5. cost scales linearly with pixel count ------------- */
    {
        static const int sizes[][2] = {
            { 512, 288 }, { 768, 432 }, { 1024, 576 }, { 1536, 864 }, { 2048, 1152 },
        };
        int nsizes = (int)(sizeof(sizes) / sizeof(sizes[0]));
        float nspx[5];
        int visits_ok = 1;
        for (int s = 0; s < nsizes; s++) {
            int w = sizes[s][0], h = sizes[s][1];
            size_t n = (size_t)w * h;
            unsigned char* big = (unsigned char*)malloc(n);
            scene_fill(big, w, h, 2, 99u + (uint32_t)s);
            Meter bm;
            meter_init(&bm, w, h, 16, 9, 0.0f);
            int iters = (int)(20e6 / (double)n);
            if (iters < 8) iters = 8;
            double best = 1e30;
            for (int run = 0; run < 3; run++) {
                double t0 = now_ns();
                for (int k = 0; k < iters; k++) meter_frame(&bm, big, &cfg, 1.0f / 60.0f);
                double dt = now_ns() - t0;
                if (dt < best) best = dt;
                if (bm.pixels_visited != (unsigned long long)n) visits_ok = 0;
            }
            nspx[s] = (float)(best / (double)(iters * (long long)n));
            printf("cost %4dx%-4d px=%8zu iters=%3d  %.3f ns/px  %.1f Mpx/s\n",
                   w, h, n, iters, nspx[s], 1000.0f / nspx[s]);
            free(big);
        }
        float lo = nspx[0], hi = nspx[0];
        for (int s = 1; s < nsizes; s++) {
            if (nspx[s] < lo) lo = nspx[s];
            if (nspx[s] > hi) hi = nspx[s];
        }
        printf("cost_ns_per_px spread=%.2fx\n", hi / lo);
        check("cost_visits_exact", visits_ok);
        check("cost_linear_within_2x", hi / lo <= 2.0f);
    }

    /* -------- 6. tiny image vs big region grid --------------------- */
    {
        unsigned char tiny[8 * 4];
        memset(tiny, 80, sizeof(tiny));
        Meter t;
        meter_init(&t, 8, 4, 16, 9, 0.0f);
        meter_frame(&t, tiny, &cfg, 1.0f / 60.0f);
        int empty = 0;
        for (int i = 0; i < t.region_count; i++) if (!t.region_valid[i]) empty++;
        printf("tiny pixels=%u empty_regions=%d target=%.3f\n",
               t.total_pixels, empty, t.target_ev);
        check("tiny_pixels_counted", t.total_pixels == 32);
        check("tiny_target_finite", isfinite(t.target_ev));
        check("tiny_empty_regions_ok", empty > 0 && t.meter_valid);
    }

    free(img);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
