
/* haptics: low-frequency drive vibration intensity calibration.

   The low-frequency drive channel showed vibration intensity mismatching
   the drive energy. This module characterizes the channel end to end:
     1. fit a power law A = c * E^p from measured (energy, amplitude)
        pairs and report a 95% confidence interval for the exponent p;
     2. derive the per-frame jitter upper bound of the amplitude
        sequence under that exponent (maximized over the interval);
     3. fix the generation parameters from 1+2 and validate the
        generated amplitude sequence against the reference.

   Acceptance rules enforced by the self tests:
     - an exponent outside the confidence interval must be rejected,
       using it anyway counts as fail;
     - any frame-to-frame jitter exceeding the bound is reported with
       its exact time segment. */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define N_MEAS 24
#define FRAMES 64
#define FPS 60.0f
#define QUANT (1.0f / 255.0f)   /* device amplitude resolution */

/* measured characterization data: drive energy -> vibration amplitude */
static const float MEAS_E[N_MEAS] = {
    0.0200f, 0.0237f, 0.0281f, 0.0333f, 0.0395f, 0.0468f, 0.0555f, 0.0658f,
    0.0780f, 0.0924f, 0.1096f, 0.1299f, 0.1540f, 0.1825f, 0.2164f, 0.2565f,
    0.3040f, 0.3604f, 0.4272f, 0.5064f, 0.6003f, 0.7116f, 0.8436f, 1.0000f
};
static const float MEAS_A[N_MEAS] = {
    0.1295f, 0.1424f, 0.1525f, 0.1604f, 0.1828f, 0.1891f, 0.2173f, 0.2290f,
    0.2458f, 0.2766f, 0.2890f, 0.3227f, 0.3475f, 0.3740f, 0.4275f, 0.4488f,
    0.4951f, 0.5337f, 0.6067f, 0.6520f, 0.6754f, 0.7764f, 0.8415f, 0.9303f
};
/* channel truth used to sanity-check the fit: A = 0.9 * E^0.5 */
#define TRUE_P 0.5f

typedef struct {
    float p;        /* fitted exponent */
    float c;        /* fitted gain */
    float p_lo;     /* 95% confidence interval for p */
    float p_hi;
} Fit;

/* two-sided 95% t critical values, df 1..30 then 40/60/120/inf */
static float t_crit_95(int df) {
    static const float T[] = {
        12.706f, 4.303f, 3.182f, 2.776f, 2.571f, 2.447f, 2.365f, 2.306f,
        2.262f, 2.228f, 2.201f, 2.179f, 2.160f, 2.145f, 2.131f, 2.120f,
        2.110f, 2.101f, 2.093f, 2.086f, 2.080f, 2.074f, 2.069f, 2.064f,
        2.060f, 2.056f, 2.052f, 2.048f, 2.045f, 2.042f
    };
    if (df >= 1 && df <= 30) return T[df - 1];
    if (df <= 40) return 2.021f;
    if (df <= 60) return 2.000f;
    if (df <= 120) return 1.980f;
    return 1.960f;
}

/* least squares on (ln E, ln A): slope p, intercept ln c */
static Fit fit_power_law(const float* E, const float* A, int n) {
    float xm = 0.0f, ym = 0.0f;
    for (int i = 0; i < n; i++) { xm += logf(E[i]); ym += logf(A[i]); }
    xm /= (float)n; ym /= (float)n;

    float sxx = 0.0f, sxy = 0.0f;
    for (int i = 0; i < n; i++) {
        float dx = logf(E[i]) - xm;
        sxx += dx * dx;
        sxy += dx * (logf(A[i]) - ym);
    }
    Fit f;
    f.p = sxy / sxx;
    float b = ym - f.p * xm;
    f.c = expf(b);

    float sse = 0.0f;
    for (int i = 0; i < n; i++) {
        float r = logf(A[i]) - (b + f.p * logf(E[i]));
        sse += r * r;
    }
    float sp = sqrtf(sse / (float)(n - 2) / sxx);
    float t = t_crit_95(n - 2);
    f.p_lo = f.p - t * sp;
    f.p_hi = f.p + t * sp;
    return f;
}

/* an exponent is usable for generation only inside the interval */
static int exponent_usable(const Fit* f, float p) {
    return p >= f->p_lo && p <= f->p_hi;
}

/* drive energy envelope: idle, ramp up, holds, drop, ramp, decay */
static float envelope(int i) {
    if (i < 8)  return 0.02f;
    if (i < 16) return 0.02f + (0.50f - 0.02f) * (float)(i - 8) / 7.0f;
    if (i < 24) return 0.50f;
    if (i < 32) return 0.65f;
    if (i < 40) return 0.15f;
    if (i < 48) return 0.15f;
    if (i < 56) return 0.15f + (0.90f - 0.15f) * (float)(i - 48) / 7.0f;
    return 0.90f + (0.10f - 0.90f) * (float)(i - 56) / 7.0f;
}

/* exact max of |a^p - b^p| over p in [lo, hi]: f' has at most one zero
   (an interior maximum), so check both endpoints plus that point */
static float pow_diff_max(float a, float b, float lo, float hi) {
    if (a == b) return 0.0f;
    float best = fabsf(powf(a, lo) - powf(b, lo));
    float v = fabsf(powf(a, hi) - powf(b, hi));
    if (v > best) best = v;
    if (a < 1.0f && b > 0.0f) {   /* interior critical point exists only here */
        float pc = logf(logf(b) / logf(a)) / logf(a / b);
        if (pc > lo && pc < hi) {
            v = fabsf(powf(a, pc) - powf(b, pc));
            if (v > best) best = v;
        }
    }
    return best;
}

/* jitter upper bound for pair (e0, e1): c * max_p |e1^p - e0^p| over the CI */
static float jitter_bound(const Fit* f, float e0, float e1) {
    float a = e0 > e1 ? e0 : e1;
    float b = e0 > e1 ? e1 : e0;
    return f->c * pow_diff_max(a, b, f->p_lo, f->p_hi);
}

static float quantize(float a) {
    return floorf(a * 255.0f + 0.5f) / 255.0f;
}

/* find runs of pairs where |seq[i+1]-seq[i]| > bound[i] + allow.
   Segments are reported in frame indices [start, end] (inclusive). */
static int find_violations(const float* seq, const float* bound, int n,
                           float allow, int* seg_lo, int* seg_hi, int max_seg) {
    int count = 0, open = 0;
    for (int i = 0; i < n - 1; i++) {
        float j = fabsf(seq[i + 1] - seq[i]);
        if (j > bound[i] + allow) {
            if (!open && count < max_seg) seg_lo[count] = i;
            open = 1;
        } else if (open) {
            seg_hi[count++] = i;   /* pair i-1..i was the last violation */
            open = 0;
        }
    }
    if (open) seg_hi[count++] = n - 1;
    return count;
}

static void report_violations(const float* seq, const float* bound, int n,
                              float allow) {
    int lo[FRAMES], hi[FRAMES];
    int k = find_violations(seq, bound, n, allow, lo, hi, FRAMES);
    if (k == 0) {
        printf("jitter_violations=none\n");
        return;
    }
    for (int s = 0; s < k; s++)
        printf("jitter_violation segment t=[%.3f,%.3f]s frames=[%d,%d]\n",
               lo[s] / FPS, hi[s] / FPS, lo[s], hi[s]);
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Fit fit = fit_power_law(MEAS_E, MEAS_A, N_MEAS);

    printf("== fit: energy->amplitude power law ==\n");
    printf("exponent=%.4f ci95=[%.4f, %.4f] c=%.4f n=%d\n",
           fit.p, fit.p_lo, fit.p_hi, fit.c, N_MEAS);

    /* generation parameters fixed from the fit and the jitter bound */
    float p_gen = fit.p;
    float E[FRAMES], ref[FRAMES], gen[FRAMES], bound[FRAMES - 1];
    float slew_max = 0.0f;
    for (int i = 0; i < FRAMES; i++) {
        E[i] = envelope(i);
        ref[i] = fit.c * powf(E[i], p_gen);
        gen[i] = quantize(ref[i]);
    }
    for (int i = 0; i < FRAMES - 1; i++) {
        bound[i] = jitter_bound(&fit, E[i], E[i + 1]);
        if (bound[i] > slew_max) slew_max = bound[i];
    }

    printf("== generation parameters ==\n");
    printf("p_gen=%.4f (interval [%0.4f, %0.4f]: %s) c=%.4f quant=%.5f slew_max=%.4f\n",
           p_gen, fit.p_lo, fit.p_hi,
           exponent_usable(&fit, p_gen) ? "in" : "OUT", fit.c, QUANT, slew_max);

    printf("== jitter upper bound curve ==\n");
    for (int i = 0; i < FRAMES - 1; i++) {
        int bars = slew_max > 0.0f ? (int)(bound[i] / slew_max * 40.0f + 0.5f) : 0;
        printf("i=%2d t=%.3f E=%.3f J=%.4f |", i, i / FPS, E[i], bound[i]);
        for (int b = 0; b < bars; b++) putchar('#');
        putchar('\n');
    }

    printf("== sequences and deviation table ==\n");
    printf("flag: '!' jitter over bound+1LSB (violation), '~' over bound within 1 LSB\n");
    printf("frame t E ref gen dev jitter bound flag\n");
    float max_dev = 0.0f;
    for (int i = 0; i < FRAMES; i++) {
        float dev = gen[i] - ref[i];
        if (fabsf(dev) > max_dev) max_dev = fabsf(dev);
        float j = i > 0 ? fabsf(gen[i] - gen[i - 1]) : 0.0f;
        float b = i > 0 ? bound[i - 1] : 0.0f;
        char flag = ' ';
        if (j > b + QUANT + 1e-6f) flag = '!';
        else if (j > b + 1e-6f) flag = '~';
        printf("%2d %.3f %.3f %.4f %.4f %+.4f %.4f %.4f %c\n",
               i, i / FPS, E[i], ref[i], gen[i], dev, j, b, flag);
    }
    printf("max_dev=%.5f\n", max_dev);

    printf("== jitter violation report (generated, effective bound = curve + 1 LSB) ==\n");
    report_violations(gen, bound, FRAMES, QUANT + 1e-6f);

    printf("== self tests ==\n");
    check("fit_recovers_true_exponent",
          TRUE_P >= fit.p_lo && TRUE_P <= fit.p_hi);
    check("gen_exponent_in_interval", exponent_usable(&fit, p_gen));
    check("out_of_interval_exponent_rejected",
          !exponent_usable(&fit, fit.p_hi + 0.05f) &&
          !exponent_usable(&fit, fit.p_lo - 0.05f));
    check("deviation_within_quantization", max_dev <= QUANT * 0.5f + 1e-6f);

    int lo[FRAMES], hi[FRAMES];
    check("ref_jitter_within_bound",
          find_violations(ref, bound, FRAMES, 1e-6f, lo, hi, FRAMES) == 0);
    check("gen_jitter_within_bound",
          find_violations(gen, bound, FRAMES, QUANT + 1e-6f, lo, hi, FRAMES) == 0);

    /* tampered sequence: +0.2 spike at frame 30 must be reported as
       exactly one segment covering frames [29,31] */
    float bad[FRAMES];
    memcpy(bad, gen, sizeof(bad));
    bad[30] += 0.2f;
    int k = find_violations(bad, bound, FRAMES, QUANT + 1e-6f, lo, hi, FRAMES);
    printf("tampered_report: ");
    report_violations(bad, bound, FRAMES, QUANT + 1e-6f);
    check("jitter_violation_segment_reported",
          k == 1 && lo[0] == 29 && hi[0] == 31);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
