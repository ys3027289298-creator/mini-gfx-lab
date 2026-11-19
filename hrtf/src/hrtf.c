/* hrtf: head-related impulse response grid, bilinear interpolation over
   (azimuth, elevation), and block-streamed binaural convolution.

   Dataset format (text, one line per ear and direction, '#' starts a comment):

       @hrtf 1
       @rate 48000
       @taps 80
       @elev -40 -20 0 20 40
       @az 0 15 30 ... 345
       <az> <el> L0 L1 ... L{T-1}
       <az> <el> R0 R1 ... R{T-1}

   Two consecutive data lines make an L/R pair for one direction. Azimuth is
   degrees, 0 = front, +90 = right ear, wrapped to [0,360). Elevation is
   degrees, +90 = up. If @az/@elev are present they define the interpolation
   grid; otherwise every distinct angle seen in the file is used. Every grid
   node must be supplied - a missing node is a load error, never zero-filled.

   The HRIR carries its own onset delay, so the renderer convolves the whole
   response at the requested direction. No extra ITD shift is layered on top,
   which is what keeps onset time, ITD and spectral shape continuous at node
   boundaries. Pairs are RMS-normalised to a common level so loudness does not
   step between nodes, then low-passed (default 8 kHz) so linear interpolation
   does not comb-filter two differently-delayed tails. */
#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define HRTF_PI 3.14159265358979323846
#define SR 48000
#define TAPS 80
#define MAX_AZ 128
#define MAX_EL 32
#define MAX_TAPS 512
#define BLOCK 128
#define HRTF_LINE_MAX 65536
#define MAX_DIR MAX_AZ *MAX_EL

typedef struct {
    float h[2][TAPS]; /* [ear][tap], ear 0 = left, 1 = right */
} Hrir;

typedef struct {
    int n_az, n_el;
    float az[MAX_AZ]; /* degrees in [0,360), sorted */
    float el[MAX_EL]; /* degrees, sorted ascending */
    Hrir node[MAX_DIR];
    unsigned char have[MAX_DIR];
    int taps;
    int rate;
    float lp_fc;   /* band limit applied at load, Hz; <= 0 means none */
    float level;   /* common RMS every pair was scaled to */
} Hrtf;

/* Loader diagnostics go to stderr. The negative tests expect them, so they
   turn this off to keep `make test` output readable; a real caller sees them. */
static int g_quiet = 0;

static int fail(const char* msg, const char* path, int line) {
    if (g_quiet) return -1;
    if (line > 0) fprintf(stderr, "hrtf: %s:%d: %s\n", path, line, msg);
    else fprintf(stderr, "hrtf: %s: %s\n", path, msg);
    return -1;
}

/* ---------- load ---------- */

static int cmp_float(const void* a, const void* b) {
    float x = *(const float*)a, y = *(const float*)b;
    return (x > y) - (x < y);
}

static int sort_unique(float* v, int n) {
    qsort(v, (size_t)n, sizeof(float), cmp_float);
    int m = 0;
    for (int i = 0; i < n; i++)
        if (m == 0 || v[i] > v[m - 1] + 1e-4f) v[m++] = v[i];
    return m;
}

static float wrap_az(float az) {
    az = fmodf(az, 360.0f);
    if (az < 0.0f) az += 360.0f;
    if (az >= 360.0f) az -= 360.0f;
    return az;
}

static int az_index(const Hrtf* hr, float az) {
    az = wrap_az(az);
    for (int i = 0; i < hr->n_az; i++)
        if (fabsf(hr->az[i] - az) < 1e-3f) return i;
    return -1;
}

static int el_index(const Hrtf* hr, float el) {
    for (int i = 0; i < hr->n_el; i++)
        if (fabsf(hr->el[i] - el) < 1e-3f) return i;
    return -1;
}

/* 4th-order Butterworth, one biquad section run on a buffer in place. */
static void biquad_lp(double* x, int n, double g, double a1, double a2) {
    double x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;
    for (int i = 0; i < n; i++) {
        double v = g * (x[i] + 2.0 * x1 + x2) - a1 * y1 - a2 * y2;
        x2 = x1; x1 = x[i];
        y2 = y1; y1 = v;
        x[i] = v;
    }
}

/* Forward filter, reverse, filter again, reverse: zero phase, so the onset
   position stored in the dataset is preserved (ITD untouched). */
static void lowpass_hrir(Hrir* h, int taps, float fc, int rate) {
    if (fc <= 0.0f || fc >= 0.5f * (float)rate) return;
    const double w = tan(HRTF_PI * (double)fc / (double)rate);
    const double w2 = w * w;
    const double q[2] = { 0.54119610, 1.30656296 }; /* 1 / (2 cos(pi k / 8)) */
    double g[2], a1[2], a2[2];
    for (int s = 0; s < 2; s++) {
        double k = 1.0 / q[s];
        double norm = 1.0 / (1.0 + k * w + w2);
        g[s] = w2 * norm;
        a1[s] = 2.0 * (w2 - 1.0) * norm;
        a2[s] = (1.0 - k * w + w2) * norm;
    }
    for (int e = 0; e < 2; e++) {
        double x[TAPS];
        for (int i = 0; i < taps; i++) x[i] = (double)h->h[e][i];
        for (int s = 0; s < 2; s++) biquad_lp(x, taps, g[s], a1[s], a2[s]);
        for (int i = 0; i < taps / 2; i++) {
            double t = x[i];
            x[i] = x[taps - 1 - i];
            x[taps - 1 - i] = t;
        }
        for (int s = 0; s < 2; s++) biquad_lp(x, taps, g[s], a1[s], a2[s]);
        for (int i = 0; i < taps / 2; i++) {
            double t = x[i];
            x[i] = x[taps - 1 - i];
            x[taps - 1 - i] = t;
        }
        for (int i = 0; i < taps; i++) h->h[e][i] = (float)x[i];
    }
}

static float hrir_rms(const Hrir* h, int taps) {
    double e = 0.0;
    for (int c = 0; c < 2; c++)
        for (int i = 0; i < taps; i++) e += (double)h->h[c][i] * h->h[c][i];
    return (float)sqrt(e / (2.0 * taps));
}

/* every pair to the same RMS, so interpolation is a weighted average of
   equally loud responses; the dataset's absolute level is not preserved */
static void normalize_pair(Hrir* h, int taps, float target) {
    float r = hrir_rms(h, taps);
    if (r < 1e-12f) return;
    float s = target / r;
    for (int c = 0; c < 2; c++)
        for (int i = 0; i < taps; i++) h->h[c][i] *= s;
}

static int read_floats(const char* s, float* out, int max, int* count) {
    int n = 0;
    char* end = NULL;
    while (*s && n < max) {
        double v = strtod(s, &end);
        if (end == s) break;
        out[n++] = (float)v;
        s = end;
    }
    *count = n;
    return 0;
}

/* Scratch buffers for the loader live in BSS: a full 128x32 grid of 80-tap
   pairs is several MB and would not fit on the stack. */
static float raw_l[MAX_DIR][TAPS];
static float raw_r[MAX_DIR][TAPS];
static float raw_az[MAX_DIR];
static float raw_el[MAX_DIR];

int hrtf_load(Hrtf* hr, const char* path, float lp_fc) {
    memset(hr, 0, sizeof(*hr));
    hr->rate = SR;
    hr->taps = TAPS;

    FILE* f = fopen(path, "r");
    if (!f) return fail("cannot open", path, 0);

    float decl_az[MAX_AZ];
    int n_decl_az = 0;
    int n_raw = 0;
    int expect = 0; /* 0 = next data line is L, 1 = next is R */
    float pend_az = 0.0f, pend_el = 0.0f;
    int lineno = 0;
    int rc = 0;

    char* line = malloc(HRTF_LINE_MAX);
    if (!line) { fclose(f); return fail("out of memory", path, 0); }

    while (fgets(line, HRTF_LINE_MAX, f)) {
        lineno++;
        char* p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == '\n' || *p == '#') continue;
        if (*p == '@') {
            char key[32] = {0};
            int off = 0;
            if (sscanf(p, "@%31s %n", key, &off) < 1) {
                rc = fail("bad directive", path, lineno);
                break;
            }
            char* rest = p + off;
            if (strcmp(key, "hrtf") == 0) {
                int v = 0;
                if (sscanf(rest, "%d", &v) != 1 || v != 1) {
                    rc = fail("unsupported format version", path, lineno);
                    break;
                }
            } else if (strcmp(key, "rate") == 0) {
                int v = 0;
                if (sscanf(rest, "%d", &v) != 1 || v <= 0) {
                    rc = fail("bad rate", path, lineno);
                    break;
                }
                hr->rate = v;
            } else if (strcmp(key, "taps") == 0) {
                int v = 0;
                if (sscanf(rest, "%d", &v) != 1 || v < 1 || v > TAPS) {
                    rc = fail("bad tap count", path, lineno);
                    break;
                }
                hr->taps = v;
            } else if (strcmp(key, "az") == 0) {
                read_floats(rest, decl_az, MAX_AZ, &n_decl_az);
                if (n_decl_az < 1) {
                    rc = fail("bad @az list", path, lineno);
                    break;
                }
            } else if (strcmp(key, "elev") == 0) {
                float tmp[MAX_EL];
                int n = 0;
                read_floats(rest, tmp, MAX_EL, &n);
                if (n < 1) {
                    rc = fail("bad @elev list", path, lineno);
                    break;
                }
                for (int i = 0; i < n; i++) hr->el[i] = tmp[i];
                hr->n_el = n;
            } else {
                fprintf(stderr, "hrtf: %s:%d: unknown directive @%s (ignored)\n",
                        path, lineno, key);
            }
            continue;
        }

        if (n_raw >= MAX_DIR) {
            rc = fail("too many directions", path, lineno);
            break;
        }
        float vals[MAX_TAPS + 2];
        int n = 0;
        read_floats(p, vals, MAX_TAPS + 2, &n);
        if (n != hr->taps + 2) {
            char msg[96];
            snprintf(msg, sizeof msg, "expected az el + %d taps, got %d values",
                     hr->taps, n);
            rc = fail(msg, path, lineno);
            break;
        }
        if (expect == 0) {
            pend_az = vals[0];
            pend_el = vals[1];
            for (int i = 0; i < hr->taps; i++) raw_l[n_raw][i] = vals[2 + i];
            expect = 1;
        } else {
            if (fabsf(pend_az - vals[0]) > 1e-3f || fabsf(pend_el - vals[1]) > 1e-3f) {
                rc = fail("L/R lines must share azimuth and elevation", path, lineno);
                break;
            }
            for (int i = 0; i < hr->taps; i++) raw_r[n_raw][i] = vals[2 + i];
            raw_az[n_raw] = vals[0];
            raw_el[n_raw] = vals[1];
            n_raw++;
            expect = 0;
        }
    }
    free(line);
    fclose(f);
    if (rc != 0) return rc;
    if (expect != 0) return fail("dangling left-ear line without a right-ear line", path, lineno);
    if (n_raw == 0) return fail("no directions", path, 0);

    /* azimuth grid */
    {
        float tmp[MAX_DIR];
        int m;
        if (n_decl_az > 0) {
            for (int i = 0; i < n_decl_az; i++) tmp[i] = wrap_az(decl_az[i]);
            m = sort_unique(tmp, n_decl_az);
        } else {
            for (int i = 0; i < n_raw; i++) tmp[i] = wrap_az(raw_az[i]);
            m = sort_unique(tmp, n_raw);
        }
        if (m > MAX_AZ) return fail("too many distinct azimuths", path, 0);
        for (int i = 0; i < m; i++) hr->az[i] = tmp[i];
        hr->n_az = m;
    }
    if (hr->n_az < 1) return fail("empty azimuth grid", path, 0);

    /* elevation grid */
    if (hr->n_el == 0) {
        for (int i = 0; i < n_raw; i++) hr->el[i] = raw_el[i];
        int m = sort_unique(hr->el, n_raw);
        if (m > MAX_EL) return fail("too many distinct elevations", path, 0);
        hr->n_el = m;
    } else {
        hr->n_el = sort_unique(hr->el, hr->n_el);
    }
    if (hr->n_el < 1) return fail("empty elevation grid", path, 0);

    /* place nodes */
    for (int i = 0; i < n_raw; i++) {
        int ai = az_index(hr, raw_az[i]);
        int ei = el_index(hr, raw_el[i]);
        if (ai < 0) {
            char msg[96];
            snprintf(msg, sizeof msg, "azimuth %.3g not in declared grid", raw_az[i]);
            return fail(msg, path, 0);
        }
        if (ei < 0) {
            char msg[96];
            snprintf(msg, sizeof msg, "elevation %.3g not in declared grid", raw_el[i]);
            return fail(msg, path, 0);
        }
        int idx = ai * hr->n_el + ei;
        if (hr->have[idx]) {
            char msg[96];
            snprintf(msg, sizeof msg, "duplicate direction az=%.3g el=%.3g",
                     raw_az[i], raw_el[i]);
            return fail(msg, path, 0);
        }
        hr->have[idx] = 1;
        for (int k = 0; k < TAPS; k++) {
            hr->node[idx].h[0][k] = k < hr->taps ? raw_l[i][k] : 0.0f;
            hr->node[idx].h[1][k] = k < hr->taps ? raw_r[i][k] : 0.0f;
        }
    }
    for (int i = 0; i < hr->n_az * hr->n_el; i++) {
        if (!hr->have[i]) {
            char msg[96];
            snprintf(msg, sizeof msg, "missing node at az=%.3g el=%.3g",
                     hr->az[i / hr->n_el], hr->el[i % hr->n_el]);
            return fail(msg, path, 0);
        }
    }

    /* common level, then a band limit that keeps interpolation from combing */
    float target = 0.0f;
    for (int i = 0; i < hr->n_az * hr->n_el; i++)
        target += hrir_rms(&hr->node[i], hr->taps);
    target /= (float)(hr->n_az * hr->n_el);
    hr->level = target;
    hr->lp_fc = lp_fc;
    for (int i = 0; i < hr->n_az * hr->n_el; i++) {
        normalize_pair(&hr->node[i], hr->taps, target);
        lowpass_hrir(&hr->node[i], hr->taps, lp_fc, hr->rate);
    }
    return 0;
}

/* ---------- direction interpolation ---------- */

/* Wrapped azimuth bracket; frac is 0 when both ends collapse onto a node. */
static void bracket_az(const Hrtf* hr, float az, int* lo, int* hi, float* frac) {
    az = wrap_az(az);
    int n = hr->n_az;
    if (n == 1) {
        *lo = *hi = 0;
        *frac = 0.0f;
        return;
    }
    /* az is in [0,360). Find the first node greater than az; everything
       below the first node belongs to the wrap cell that ends at it. */
    int upper = 0;
    while (upper < n && hr->az[upper] <= az) upper++;
    if (upper == 0 || upper >= n) {
        /* wrap cell: last node -> first node */
        const float base = hr->az[n - 1];
        const float span = hr->az[0] + 360.0f - base;
        float off = az - base;
        if (off < 0.0f) off += 360.0f;
        *lo = n - 1;
        *hi = 0;
        *frac = span > 0.0f ? off / span : 0.0f;
    } else {
        *lo = upper - 1;
        *hi = upper;
        float span = hr->az[upper] - hr->az[upper - 1];
        *frac = span > 0.0f ? (az - hr->az[upper - 1]) / span : 0.0f;
    }
}

/* Elevation bracket, clamped beyond the grid ends. */
static void bracket_el(const Hrtf* hr, float el, int* lo, int* hi, float* frac) {
    int n = hr->n_el;
    if (n == 1 || el <= hr->el[0]) {
        *lo = *hi = 0;
        *frac = 0.0f;
        return;
    }
    if (el >= hr->el[n - 1]) {
        *lo = *hi = n - 1;
        *frac = 0.0f;
        return;
    }
    for (int i = 0; i < n - 1; i++) {
        if (el >= hr->el[i] && el <= hr->el[i + 1]) {
            *lo = i;
            *hi = i + 1;
            float span = hr->el[i + 1] - hr->el[i];
            *frac = span > 0.0f ? (el - hr->el[i]) / span : 0.0f;
            return;
        }
    }
    *lo = *hi = n - 1;
    *frac = 0.0f;
}

static int nearest_az(const Hrtf* hr, float az) {
    int lo, hi;
    float f;
    bracket_az(hr, az, &lo, &hi, &f);
    return f <= 0.5f ? lo : hi;
}

static int nearest_el(const Hrtf* hr, float el) {
    int lo, hi;
    float f;
    bracket_el(hr, el, &lo, &hi, &f);
    return f <= 0.5f ? lo : hi;
}

void hrtf_hrir_bilinear(const Hrtf* hr, float az, float el, float* hl, float* hright) {
    int a0, a1, e0, e1;
    float ta, te;
    bracket_az(hr, az, &a0, &a1, &ta);
    bracket_el(hr, el, &e0, &e1, &te);
    const int ne = hr->n_el;
    const float w00 = (1.0f - ta) * (1.0f - te);
    const float w10 = ta * (1.0f - te);
    const float w01 = (1.0f - ta) * te;
    const float w11 = ta * te;
    const Hrir* n00 = &hr->node[a0 * ne + e0];
    const Hrir* n10 = &hr->node[a1 * ne + e0];
    const Hrir* n01 = &hr->node[a0 * ne + e1];
    const Hrir* n11 = &hr->node[a1 * ne + e1];
    for (int k = 0; k < hr->taps; k++) {
        hl[k] = w00 * n00->h[0][k] + w10 * n10->h[0][k] +
                w01 * n01->h[0][k] + w11 * n11->h[0][k];
        hright[k] = w00 * n00->h[1][k] + w10 * n10->h[1][k] +
                    w01 * n01->h[1][k] + w11 * n11->h[1][k];
    }
}

void hrtf_hrir_nearest(const Hrtf* hr, float az, float el, float* hl, float* hright) {
    int ai = nearest_az(hr, az);
    int ei = nearest_el(hr, el);
    const Hrir* n = &hr->node[ai * hr->n_el + ei];
    for (int k = 0; k < hr->taps; k++) {
        hl[k] = n->h[0][k];
        hright[k] = n->h[1][k];
    }
}

/* ---------- rendering ---------- */

/* Direct-form FIR with a history ring. Kernel length is always the dataset's
   tap count, so changing kernels between blocks never leaves stale tail
   energy in the line. */
typedef struct {
    int taps;
    float coef[TAPS];
    float hist[TAPS]; /* hist[0] = newest input */
} Convolver;

void conv_set(Convolver* cv, const float* coef, int taps) {
    if (taps > TAPS) taps = TAPS;
    cv->taps = taps;
    memcpy(cv->coef, coef, sizeof(float) * (size_t)taps);
}

void conv_reset(Convolver* cv) {
    memset(cv->hist, 0, sizeof cv->hist);
}

static float conv_step(Convolver* cv, float x) {
    int t = cv->taps;
    for (int i = t - 1; i > 0; i--) cv->hist[i] = cv->hist[i - 1];
    cv->hist[0] = x;
    float acc = 0.0f;
    for (int i = 0; i < t; i++) acc += cv->coef[i] * cv->hist[i];
    return acc;
}

/* Render `frames` input samples at one fixed direction into interleaved
   stereo. Call once per audio block (BLOCK samples is the house size) with
   the direction updated by the caller; the convolver history carries across
   calls, so splitting a stream into blocks is bit-identical to one call.
   Interpolation cost is therefore paid once per block, not per sample. */
void hrtf_render_block(const Hrtf* hr, int interp, float az, float el,
                       const float* mono, int frames, float* out, Convolver* cl,
                       Convolver* cr) {
    float hl[TAPS], hh[TAPS];
    if (interp) hrtf_hrir_bilinear(hr, az, el, hl, hh);
    else hrtf_hrir_nearest(hr, az, el, hl, hh);
    conv_set(cl, hl, hr->taps);
    conv_set(cr, hh, hr->taps);
    for (int i = 0; i < frames; i++) {
        float yl = conv_step(cl, mono[i]);
        float yr = conv_step(cr, mono[i]);
        out[2 * i + 0] = yl;
        out[2 * i + 1] = yr;
    }
}

/* ---------- self-test ---------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

#define T_DATASET "hrtf_test.dat"

/* Ground-truth HRIR model used only to build and to score the test dataset:
   lateralised onset, elevation-dependent pinna resonances, a floor bounce. */
static float synth(int rate, float az_deg, float el_deg, int k, int ear) {
    const float ratef = (float)rate;
    float az = az_deg * (float)HRTF_PI / 180.0f;
    float el = el_deg * (float)HRTF_PI / 180.0f;
    float lat = sinf(az) * cosf(el);
    float base = 0.00025f * ratef;
    float itd_half = 0.5f * 0.0006f * ratef * lat;
    float amp = sqrtf(0.5f * (1.0f + (ear == 0 ? -0.65f : 0.65f) * lat));
    float t = (float)k / ratef;
    float f1 = 3200.0f + 1400.0f * sinf(el);
    float f2 = 6200.0f + 900.0f * cosf(el);
    float shift = (ear == 0 ? base + itd_half : base - itd_half) / ratef;
    float two_pi = 2.0f * (float)HRTF_PI;
    float u = t - shift;
    if (u < 0.0f) return 0.0f;
    float v = amp * expf(-u / 0.28e-3f) * sinf(two_pi * f1 * u);
    float u2 = u - (0.30e-3f + 0.12e-3f * sinf(el));
    if (u2 >= 0.0f) v += amp * 0.35f * expf(-u2 / 0.25e-3f) * sinf(two_pi * f2 * u2 + 0.5f);
    float u3 = u - 0.55e-3f;
    if (u3 >= 0.0f) v += amp * 0.18f * expf(-u3 / 0.30e-3f) * sinf(two_pi * 1500.0f * u3);
    return v;
}

static int write_dataset(int rate, int taps, const float* azs, int naz,
                         const float* els, int nel) {
    FILE* f = fopen(T_DATASET, "w");
    if (!f) return -1;
    fprintf(f, "# synthesized HRIR grid used by the hrtf self-test\n");
    fprintf(f, "@hrtf 1\n@rate %d\n@taps %d\n", rate, taps);
    fprintf(f, "@elev");
    for (int j = 0; j < nel; j++) fprintf(f, " %g", els[j]);
    fprintf(f, "\n@az");
    for (int i = 0; i < naz; i++) fprintf(f, " %g", azs[i]);
    fprintf(f, "\n");
    for (int i = 0; i < naz; i++) {
        for (int j = 0; j < nel; j++) {
            for (int ear = 0; ear < 2; ear++) {
                fprintf(f, "%g %g", azs[i], els[j]);
                for (int k = 0; k < taps; k++)
                    fprintf(f, " %.7g", synth(rate, azs[i], els[j], k, ear));
                fprintf(f, "\n");
            }
        }
    }
    fclose(f);
    return 0;
}

/* Reference HRIR for an arbitrary direction: the model response, brought to
   the grid's common level and band limit exactly the way hrtf_load does it.
   This is the target the interpolator should reproduce. */
static void reference_hrir(const Hrtf* hr, float az, float el, float* hl,
                           float* hright) {
    Hrir t;
    for (int ear = 0; ear < 2; ear++) {
        float* h = ear == 0 ? hl : hright;
        for (int k = 0; k < hr->taps; k++) h[k] = synth(hr->rate, az, el, k, ear);
        for (int k = 0; k < TAPS; k++) t.h[ear][k] = k < hr->taps ? h[k] : 0.0f;
    }
    normalize_pair(&t, hr->taps, hr->level);
    lowpass_hrir(&t, hr->taps, hr->lp_fc, hr->rate);
    for (int ear = 0; ear < 2; ear++) {
        float* h = ear == 0 ? hl : hright;
        for (int k = 0; k < hr->taps; k++) h[k] = t.h[ear][k];
    }
}

/* Relative L2 distance to the reference HRIR, both ears. */
static float hrir_error(const Hrtf* hr, const float* hl, const float* hright,
                        float az, float el) {
    float rl[TAPS], rr[TAPS];
    reference_hrir(hr, az, el, rl, rr);
    double num = 0.0, den = 0.0;
    for (int ear = 0; ear < 2; ear++) {
        const float* h = ear == 0 ? hl : hright;
        const float* ref = ear == 0 ? rl : rr;
        for (int k = 0; k < hr->taps; k++) {
            double d = (double)h[k] - ref[k];
            num += d * d;
            den += (double)ref[k] * ref[k];
        }
    }
    return den > 0.0 ? (float)sqrt(num / den) : 0.0f;
}

/* Relative size of the HRIR change between two adjacent directions. Moving
   the source by `step` degrees on a grid of spacing `cell` should change the
   response by roughly step/cell of the node-to-node difference, with no
   resonance or combing on top: that is what makes a sweep smooth. */
static float hrir_step(const Hrtf* hr, int interp, float az, float el, float step) {
    float a[TAPS], b[TAPS], c[TAPS], d[TAPS];
    if (interp) {
        hrtf_hrir_bilinear(hr, az, el, a, b);
        hrtf_hrir_bilinear(hr, az + step, el, c, d);
    } else {
        hrtf_hrir_nearest(hr, az, el, a, b);
        hrtf_hrir_nearest(hr, az + step, el, c, d);
    }
    double num = 0.0, den = 0.0;
    for (int k = 0; k < hr->taps; k++) {
        double dl = (double)a[k] - c[k], dr = (double)b[k] - d[k];
        num += dl * dl + dr * dr;
        den += (double)a[k] * a[k] + (double)b[k] * b[k]
             + (double)c[k] * c[k] + (double)d[k] * d[k];
    }
    return den > 0.0 ? (float)sqrt(2.0 * num / den) : 0.0f;
}

/* Discontinuity at a grid boundary, isolated by sampling `eps` degrees to
   either side. A continuous interpolant gives ~0 as eps -> 0; a nearest
   lookup jumps by the full node-to-node difference. */
static float boundary_jump(const Hrtf* hr, int interp, float az, float el,
                           int along_az, float eps) {
    float a[TAPS], b[TAPS], c[TAPS], d[TAPS];
    float a0 = along_az ? az - eps : az, e0 = along_az ? el : el - eps;
    float a1 = along_az ? az + eps : az, e1 = along_az ? el : el + eps;
    if (interp) {
        hrtf_hrir_bilinear(hr, a0, e0, a, b);
        hrtf_hrir_bilinear(hr, a1, e1, c, d);
    } else {
        hrtf_hrir_nearest(hr, a0, e0, a, b);
        hrtf_hrir_nearest(hr, a1, e1, c, d);
    }
    double num = 0.0, den = 0.0;
    for (int k = 0; k < hr->taps; k++) {
        double dl = (double)a[k] - c[k], dr = (double)b[k] - d[k];
        num += dl * dl + dr * dr;
        den += (double)a[k] * a[k] + (double)b[k] * b[k]
             + (double)c[k] * c[k] + (double)d[k] * d[k];
    }
    return den > 0.0 ? (float)sqrt(2.0 * num / den) : 0.0f;
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

static Hrtf g_hr, g_full, g_bad; /* multi-MB: keep off the stack */

int main(void) {
    Hrtf* hr = &g_hr;
    const float fc = 8000.0f;
    const float azs[] = { 0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165,
                          180, 195, 210, 225, 240, 255, 270, 285, 300, 315,
                          330, 345 };
    const float els[] = { -40, -20, 0, 20, 40 };
    const int naz = (int)(sizeof azs / sizeof azs[0]);
    const int nel = (int)(sizeof els / sizeof els[0]);

    if (write_dataset(SR, TAPS, azs, naz, els, nel) != 0) {
        fprintf(stderr, "hrtf: cannot write %s\n", T_DATASET);
        return 1;
    }
    check("load_ok", hrtf_load(hr, T_DATASET, fc) == 0);
    check("grid_shape", hr->n_az == naz && hr->n_el == nel);
    check("rate_parsed", hr->rate == SR);
    check("taps_parsed", hr->taps == TAPS);

    /* on-grid directions come back as the stored node, modulo the band limit */
    {
        float l[TAPS], r[TAPS];
        hrtf_hrir_bilinear(hr, 90.0f, 0.0f, l, r);
        float err = hrir_error(hr, l, r, 90.0f, 0.0f);
        printf("node_err=%.4f\n", err);
        check("node_matches_model", err < 0.02f);
        hrtf_hrir_bilinear(hr, -30.0f, 20.0f, l, r); /* wraps to 330 */
        check("az_wraps_negative", hrir_error(hr, l, r, 330.0f, 20.0f) < 0.02f);
        hrtf_hrir_bilinear(hr, 330.0f, 20.0f, l, r);
        check("az_wrap_same", hrir_error(hr, l, r, 330.0f, 20.0f) < 0.02f);
    }

    /* off-grid directions: interpolation should beat nearest neighbour */
    {
        static const float pts[][2] = {
            { 7.5f, -10 },  { 22.5f, 10 },  { 52.5f, 30 },  { 97.5f, -30 },
            { 142.5f, 10 }, { 187.5f, -10 }, { 232.5f, 30 }, { 277.5f, -30 },
            { 322.5f, 10 }, { 352.5f, -10 },
        };
        const int np = (int)(sizeof pts / sizeof pts[0]);
        float ei_sum = 0.0f, en_sum = 0.0f;
        int better = 0;
        for (int i = 0; i < np; i++) {
            float l[TAPS], r[TAPS];
            hrtf_hrir_bilinear(hr, pts[i][0], pts[i][1], l, r);
            float ei = hrir_error(hr, l, r, pts[i][0], pts[i][1]);
            hrtf_hrir_nearest(hr, pts[i][0], pts[i][1], l, r);
            float en = hrir_error(hr, l, r, pts[i][0], pts[i][1]);
            ei_sum += ei;
            en_sum += en;
            if (ei < en) better++;
        }
        float gain = en_sum / ei_sum;
        printf("interp_hrir_err=%.4f nearest_hrir_err=%.4f gain=%.2fx\n",
               ei_sum / np, en_sum / np, gain);
        check("interp_better_every_point", better == np);
        check("interp_gain_at_least_1.3x", gain >= 1.3f);
        check("interp_gain_not_absurd", gain < 4.0f);
    }

    /* continuity: no audible jump anywhere, including grid boundaries */
    {
        /* At every cell midpoint the interpolant must be continuous. The
           nearest lookup is where it hands over between nodes, so it must
           show a step there. Node positions are not the interesting test for
           nearest: both sides of a node pick the same node. */
        float worst_int = 0.0f, worst_near = 0.0f;
        for (int a = 0; a < hr->n_az; a++) {
            float m = hr->az[a] + 7.5f;   /* 15 deg grid -> midpoint */
            float jump = boundary_jump(hr, 1, m, 0.0f, 1, 1e-3f);
            if (jump > worst_int) worst_int = jump;
            jump = boundary_jump(hr, 0, m, 0.0f, 1, 1e-3f);
            if (jump > worst_near) worst_near = jump;
        }
        for (int e = 1; e < hr->n_el - 1; e++) {
            float m = 0.5f * (hr->el[e - 1] + hr->el[e]);
            float jump = boundary_jump(hr, 1, 10.0f, m, 0, 1e-3f);
            if (jump > worst_int) worst_int = jump;
            jump = boundary_jump(hr, 0, 10.0f, m, 0, 1e-3f);
            if (jump > worst_near) worst_near = jump;
        }
        printf("boundary_jump interp=%.2e nearest=%.4f\n", worst_int, worst_near);
        check("interp_continuous_at_midpoints", worst_int < 5e-4f);
        check("nearest_has_hard_step", worst_near > 0.1f);
        check("wrap_seam_same_as_interior",
              boundary_jump(hr, 1, 352.5f, 0.0f, 1, 1e-3f) <= worst_int + 1e-6f);

        /* a smooth sweep: change must be linear in the angle moved, so the
           step size alone controls smoothness and no interpolation artefact
           adds a fixed jump */
        float s1 = hrir_step(hr, 1, 10.0f, 10.0f, 1.0f);
        float s2 = hrir_step(hr, 1, 10.0f, 10.0f, 0.5f);
        float s4 = hrir_step(hr, 1, 10.0f, 10.0f, 0.25f);
        printf("sweep interp: 1deg=%.5f 0.5deg=%.5f 0.25deg=%.5f\n", s1, s2, s4);
        check("sweep_halves_1", s2 / s1 > 0.45f && s2 / s1 < 0.55f);
        check("sweep_halves_2", s4 / s2 > 0.45f && s4 / s2 < 0.55f);
        check("sweep_quarter_is_quarter", fabsf(s4 / s1 - 0.25f) < 0.02f);
        check("sweep_1deg_is_cell_fraction", s1 > 0.05f && s1 < 0.20f);

        /* A moving source, block by block: measure the discontinuity at each
           block boundary against the largest step inside a block. The HRIR
           changes at the boundary; if that change is continuous it should not
           stand out from the signal's own slope. Nearest has a visible click
           each time the node flips; interpolation must not. */
        {
            static float mono[BLOCK], out[BLOCK * 2];
            for (int i = 0; i < BLOCK; i++)
                mono[i] = sinf(2.0f * (float)HRTF_PI * 200.0f * (float)i / (float)SR) * 0.3f;
            float edge[2], inside[2];
            for (int mode = 0; mode < 2; mode++) {
                Convolver cl, cr;
                conv_reset(&cl);
                conv_reset(&cr);
                float prev_last = 0.0f, worst_edge = 0.0f, worst_in = 0.0f;
                for (int b = 0; b < 72; b++) {
                    float az = fmodf((float)b * 5.0f, 360.0f);
                    hrtf_render_block(hr, mode == 0, az, 0.0f, mono, BLOCK, out,
                                      &cl, &cr);
                    if (b > 0) {
                        float d = fabsf(out[0] - prev_last);
                        if (d > worst_edge) worst_edge = d;
                    }
                    for (int i = 1; i < BLOCK; i++) {
                        float d = fabsf(out[2 * i] - out[2 * (i - 1)]);
                        if (d > worst_in) worst_in = d;
                    }
                    prev_last = out[2 * (BLOCK - 1)];
                }
                edge[mode] = worst_edge;
                inside[mode] = worst_in;
                printf("sweep_edge %-7s edge=%.5f inside_max=%.5f\n",
                       mode == 0 ? "interp" : "nearest", worst_edge, worst_in);
            }
            /* the edge jump of the interpolated sweep is no worse than the
               signal's own largest sample step; the nearest sweep's click is
               clearly larger than the interpolated one */
            check("sweep_interp_no_click", edge[0] <= 1.05f * inside[0]);
            check("sweep_nearest_clicks", edge[1] > 1.5f * edge[0]);
        }
    }

    /* convolution output against a direct DFT-free reference sum */
    {
        enum { N = 400 };
        static float mono[N], out[N * 2], out2[N * 2], ref[N * 2];
        for (int i = 0; i < N; i++) mono[i] = sinf(0.05f * (float)i) * 0.5f;
        Convolver cl, cr;
        conv_reset(&cl);
        conv_reset(&cr);
        float hl[TAPS], hh[TAPS];
        hrtf_hrir_bilinear(hr, 33.0f, 12.0f, hl, hh);
        hrtf_render_block(hr, 1, 33.0f, 12.0f, mono, N, out, &cl, &cr);
        for (int i = 0; i < N; i++) {
            double al = 0.0, ar = 0.0;
            for (int k = 0; k < hr->taps; k++) {
                int j = i - k;
                if (j < 0) continue;
                al += (double)hl[k] * mono[j];
                ar += (double)hh[k] * mono[j];
            }
            ref[2 * i + 0] = (float)al;
            ref[2 * i + 1] = (float)ar;
        }
        double num = 0.0, den = 0.0;
        for (int i = 0; i < N * 2; i++) {
            double d = (double)out[i] - ref[i];
            num += d * d;
            den += (double)ref[i] * ref[i];
        }
        printf("conv_rel_err=%.3g\n", sqrt(num / den));
        check("conv_matches_direct", sqrt(num / den) < 1e-5);

        conv_reset(&cl);
        conv_reset(&cr);
        for (int off = 0; off < N; off += 100) {
            int n = N - off < 100 ? N - off : 100;
            hrtf_render_block(hr, 1, 33.0f, 12.0f, mono + off, n, out2 + 2 * off,
                              &cl, &cr);
        }
        double dmax = 0.0;
        for (int i = 0; i < N * 2; i++) {
            double d = fabs((double)out[i] - out2[i]);
            if (d > dmax) dmax = d;
        }
        printf("stream_max_diff=%.3g\n", dmax);
        check("stream_matches_block", dmax < 1e-6);
        check("output_finite", isfinite(out[0]) && isfinite(out[2 * N - 1]));
    }

    /* per-frame budget: full-size grid, 128-frame block, moving source */
    {
        float faz[MAX_AZ];
        int fnaz = 0;
        for (float a = 0.0f; a < 360.0f; a += 5.0f) faz[fnaz++] = a;
        const float fels[] = { -80, -60, -40, -20, 0, 20, 40, 60, 80 };
        const int fnel = (int)(sizeof fels / sizeof fels[0]);
        Hrtf* full = &g_full;
        if (write_dataset(SR, TAPS, faz, fnaz, fels, fnel) != 0) {
            check("full_dataset_written", 0);
        } else {
            int ok = hrtf_load(full, T_DATASET, fc) == 0;
            check("full_load_ok", ok);
            check("full_grid_shape", full->n_az == fnaz && full->n_el == fnel);
            if (ok) {
                static float mono[BLOCK], out[BLOCK * 2];
                for (int i = 0; i < BLOCK; i++) mono[i] = sinf(0.03f * (float)i) * 0.3f;
                enum { REP = 4000 };
                double frame[2][2];   /* [mode][0 = avg, 1 = worst] */
                for (int mode = 0; mode < 2; mode++) {
                    Convolver cl, cr;
                    conv_reset(&cl);
                    conv_reset(&cr);
                    double total = 0.0, worst = 0.0;
                    for (int r = 0; r < REP; r++) {
                        float az = fmodf((float)r * 2.3f, 360.0f);
                        float el = 40.0f * sinf((float)r * 0.017f);
                        double t0 = now_ms();
                        hrtf_render_block(full, mode == 0, az, el, mono, BLOCK, out,
                                          &cl, &cr);
                        double dt = now_ms() - t0;
                        total += dt;
                        if (dt / BLOCK > worst) worst = dt / BLOCK;
                    }
                    frame[mode][0] = total / ((double)REP * BLOCK);
                    frame[mode][1] = worst;
                    printf("frame_ms %-7s avg=%.5f worst=%.5f\n",
                           mode == 0 ? "interp" : "nearest",
                           frame[mode][0], frame[mode][1]);
                    check(mode == 0 ? "frame_budget_interp" : "frame_budget_nearest",
                          frame[mode][0] < 3.0 && frame[mode][1] < 3.0);
                }
                /* the headline budget is the interpolated path */
                check("frame_budget_under_3ms",
                      frame[0][0] < 3.0 && frame[0][1] < 3.0);
                check("budget_output_finite", isfinite(out[0]));
            }
        }
    }

    /* a grid that does not contain 0 must still wrap: the same physical
       direction expressed three ways must give one HRIR, and the 340->20
       seam must be as continuous as any other cell edge */
    {
        const float waz[] = { 20, 40, 60, 80, 100, 120, 140, 160, 180, 200,
                              220, 240, 260, 280, 300, 320, 340 };
        const float wel[] = { 0 };
        int wok = write_dataset(SR, TAPS, waz, 17, wel, 1) == 0 &&
                  hrtf_load(&g_bad, T_DATASET, fc) == 0;
        check("load_grid_without_zero", wok);
        if (wok) {
            float a[TAPS], b[TAPS], c[TAPS], d[TAPS];
            hrtf_hrir_bilinear(&g_bad, 10.0f, 0.0f, a, b);
            hrtf_hrir_bilinear(&g_bad, 370.0f, 0.0f, c, d);
            double num = 0.0, den = 0.0;
            for (int k = 0; k < TAPS; k++) {
                double dl = (double)a[k] - c[k], dr = (double)b[k] - d[k];
                num += dl * dl + dr * dr;
                den += (double)a[k] * a[k] + (double)b[k] * b[k];
            }
            check("wrap_370_same_as_10", sqrt(2.0 * num / den) < 1e-7);
            hrtf_hrir_bilinear(&g_bad, -350.0f, 0.0f, c, d);
            num = den = 0.0;
            for (int k = 0; k < TAPS; k++) {
                double dl = (double)a[k] - c[k], dr = (double)b[k] - d[k];
                num += dl * dl + dr * dr;
                den += (double)a[k] * a[k] + (double)b[k] * b[k];
            }
            check("wrap_negative_same_as_10", sqrt(2.0 * num / den) < 1e-7);
            check("wrap_cell_edge_continuous",
                  boundary_jump(&g_bad, 1, 0.0f, 0.0f, 1, 1e-3f) < 5e-4f);
        }
    }

    /* loader rejects malformed input instead of guessing; expected
       diagnostics are suppressed and the case is named in the check */
    g_quiet = 1;
    {
        FILE* f;
        f = fopen(T_DATASET, "w");
        if (f) {
            fprintf(f, "@hrtf 1\n@rate %d\n@taps 8\n", SR);
            fprintf(f, "0 0 1 0 0 0 0 0 0 0\n0 0 1 0 0 0 0 0 0 0\n");
            fprintf(f, "45 0 1 0 0 0 0 0 0 0\n");
            fclose(f);
        }
        check("reject_dangling_ear", hrtf_load(&g_bad, T_DATASET, 0.0f) != 0);
    }
    {
        FILE* f = fopen(T_DATASET, "w");
        if (f) {
            fprintf(f, "@hrtf 1\n@rate %d\n@taps 8\n", SR);
            fprintf(f, "0 0 1 0 0 0 0 0 0 0\n0 0 1 0 0 0 0 0 0 0\n");
            fprintf(f, "45 0 1 0 0 0 0 0\n45 0 1 0 0 0 0 0\n");
            fclose(f);
        }
        check("reject_short_line", hrtf_load(&g_bad, T_DATASET, 0.0f) != 0);
    }
    {
        FILE* f = fopen(T_DATASET, "w");
        if (f) {
            fprintf(f, "@hrtf 1\n@rate %d\n@taps 8\n", SR);
            fprintf(f, "0 0 1 0 0 0 0 0 0 0\n0 0 1 0 0 0 0 0 0 0\n");
            fprintf(f, "45 0 1 0 0 0 0 0 0 0\n90 0 1 0 0 0 0 0 0 0\n");
            fclose(f);
        }
        check("reject_mismatched_pair", hrtf_load(&g_bad, T_DATASET, 0.0f) != 0);
    }
    {
        FILE* f = fopen(T_DATASET, "w");
        if (f) {
            fprintf(f, "@hrtf 1\n@rate %d\n@taps 8\n@elev 0\n@az 0 45 90\n", SR);
            fprintf(f, "0 0 1 0 0 0 0 0 0 0\n0 0 1 0 0 0 0 0 0 0\n");
            fprintf(f, "45 0 1 0 0 0 0 0 0 0\n45 0 1 0 0 0 0 0 0 0\n");
            fclose(f);
        }
        check("reject_missing_node", hrtf_load(&g_bad, T_DATASET, 0.0f) != 0);
    }
    {
        FILE* f = fopen(T_DATASET, "w");
        if (f) {
            fprintf(f, "@hrtf 1\n@rate %d\n@taps 8\n", SR);
            fprintf(f, "0 0 1 0 0 0 0 0 0 0\n0 0 1 0 0 0 0 0 0 0\n");
            fprintf(f, "0 0 1 0 0 0 0 0 0 0\n0 0 1 0 0 0 0 0 0 0\n");
            fclose(f);
        }
        check("reject_duplicate", hrtf_load(&g_bad, T_DATASET, 0.0f) != 0);
    }
    {
        FILE* f = fopen(T_DATASET, "w");
        if (f) {
            fprintf(f, "@hrtf 1\n@rate %d\n@taps 8\n@az 0 90\n", SR);
            fprintf(f, "0 0 1 0 0 0 0 0 0 0\n0 0 1 0 0 0 0 0 0 0\n");
            fprintf(f, "45 0 1 0 0 0 0 0 0 0\n45 0 1 0 0 0 0 0 0 0\n");
            fclose(f);
        }
        check("reject_offgrid_az", hrtf_load(&g_bad, T_DATASET, 0.0f) != 0);
    }

    g_quiet = 0;
    remove(T_DATASET);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
