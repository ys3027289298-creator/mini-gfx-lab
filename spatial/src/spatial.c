
/* mini-spatial-audio: binaural positioning of mono sources.
   ITD by azimuth (max 0.6 ms), IID by azimuth, inverse-distance attenuation,
   soft-clip mixing of N sources into an interleaved stereo buffer.
   Early reflections: shoebox-room image-source method — six walls with
   per-wall absorption, reflection-order limit, each image delayed by
   distance / C_SOUND and mixed binaurally like the direct path.
   room == NULL or room.enabled == 0 renders the dry path only, bit-identical
   to the pre-reflection version. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SR 48000
#define MAX_FRAMES 4096
#define MAX_SRC 8
#define MAX_IMAGES 1024
#define MAX_AXIS_IMG 64
#define C_SOUND 343.0f
#define WALL_UNSET -1.0f

typedef struct {
    float x, y, z;      /* world position, meters; listener at origin facing -z */
    float gain;
} Source;

typedef struct {
    float yaw;          /* radians, 0 = facing -z */
} Listener;

/* wall indices: negative/positive face per axis */
enum { WALL_XN = 0, WALL_XP, WALL_YN, WALL_YP, WALL_ZN, WALL_ZP };

typedef struct {
    float min[3], max[3];   /* room bounds, meters */
    float absorption[6];    /* per wall, 0..1; WALL_UNSET = material missing */
    int order;              /* max reflection order; negative is invalid */
    int enabled;            /* 0 = dry render, bit-identical to no room */
} Room;

typedef struct {
    float x, y, z;
    float gain;             /* source gain * wall attenuation * distance gain */
    int delay;              /* propagation delay in samples */
} ImageSource;

static void room_init(Room* r, const float mn[3], const float mx[3], int order) {
    for (int a = 0; a < 3; a++) { r->min[a] = mn[a]; r->max[a] = mx[a]; }
    for (int w = 0; w < 6; w++) r->absorption[w] = WALL_UNSET;
    r->order = order;
    r->enabled = 1;
}

static int room_set_wall(Room* r, int wall, float absorption) {
    if (wall < 0 || wall > 5) return -1;
    if (!(absorption >= 0.0f && absorption <= 1.0f)) return -1;
    r->absorption[wall] = absorption;
    return 0;
}

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

/* image positions along one axis, walls at lo/hi, reflection coeffs rlo/rhi */
typedef struct {
    float pos;
    float atten;    /* rlo^n_lo * rhi^n_hi for this axis */
    int order;      /* reflections along this axis */
} AxisImg;

static int axis_images(float s, float lo, float hi, float rlo, float rhi,
                       int max_order, AxisImg* out, int cap) {
    float L = hi - lo;
    int n = 0;
    for (int k = -max_order; k <= max_order; k++) {
        int ak = k < 0 ? -k : k;
        int even = 2 * ak;              /* family: s + 2kL, |k| crossings per wall */
        if (even <= max_order) {
            if (n >= cap) return -1;
            out[n].pos = s + 2.0f * k * L;
            out[n].order = even;
            out[n].atten = powf(rlo * rhi, (float)ak);
            n++;
        }
        int odd = abs(2 * k - 1);       /* family: 2lo - s + 2kL */
        if (odd <= max_order) {
            if (n >= cap) return -1;
            int nlo, nhi;
            if (k > 0) { nhi = k;     nlo = k - 1; }   /* e.g. k=1: across hi once */
            else       { nlo = 1 - k; nhi = -k;    }   /* e.g. k=0: across lo once */
            out[n].pos = 2.0f * lo - s + 2.0f * k * L;
            out[n].order = odd;
            out[n].atten = powf(rlo, (float)nlo) * powf(rhi, (float)nhi);
            n++;
        }
    }
    return n;
}

/* enumerate image sources up to room->order; returns count or -1 on error */
static int room_images(const Room* r, const Source* s, ImageSource* out, int cap,
                       char* err, int errsz) {
    if (r->order < 0) {
        snprintf(err, (size_t)errsz, "reflection order %d is negative", r->order);
        return -1;
    }
    for (int a = 0; a < 3; a++) {
        if (!(r->min[a] < r->max[a])) {
            snprintf(err, (size_t)errsz, "axis %d: min %.3f >= max %.3f",
                     a, r->min[a], r->max[a]);
            return -1;
        }
    }
    float refl[6];
    for (int w = 0; w < 6; w++) {
        if (r->absorption[w] == WALL_UNSET) {
            snprintf(err, (size_t)errsz, "wall %d has no material", w);
            return -1;
        }
        refl[w] = 1.0f - r->absorption[w];
    }
    if (r->order == 0) return 0;
    float sp[3] = { s->x, s->y, s->z };
    static AxisImg ai[3][MAX_AXIS_IMG];
    int na[3];
    for (int a = 0; a < 3; a++) {
        na[a] = axis_images(sp[a], r->min[a], r->max[a],
                            refl[2 * a], refl[2 * a + 1], r->order,
                            ai[a], MAX_AXIS_IMG);
        if (na[a] < 0) {
            snprintf(err, (size_t)errsz, "order %d too large", r->order);
            return -1;
        }
    }
    int n = 0;
    for (int ix = 0; ix < na[0]; ix++)
    for (int iy = 0; iy < na[1]; iy++)
    for (int iz = 0; iz < na[2]; iz++) {
        int order = ai[0][ix].order + ai[1][iy].order + ai[2][iz].order;
        if (order == 0 || order > r->order) continue;   /* 0 = direct path */
        if (n >= cap) {
            snprintf(err, (size_t)errsz, "image capacity %d exceeded", cap);
            return -1;
        }
        float x = ai[0][ix].pos, y = ai[1][iy].pos, z = ai[2][iz].pos;
        float d = sqrtf(x * x + y * y + z * z);
        float atten = ai[0][ix].atten * ai[1][iy].atten * ai[2][iz].atten;
        out[n].x = x; out[n].y = y; out[n].z = z;
        out[n].gain = s->gain * atten / (1.0f + d);
        out[n].delay = (int)lroundf(d / C_SOUND * SR);
        n++;
    }
    return n;
}

/* returns 0 on success, -1 if the room config was rejected (dry path still
   rendered so the output buffer is always defined) */
static int render(const Source* srcs, int n, const Listener* l,
                  const float* mono, int frames, float* out, /* interleaved */
                  const Room* room) {
    int rc = 0;
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
        if (!room || !room->enabled) continue;
        static ImageSource imgs[MAX_IMAGES];
        char err[128];
        int m = room_images(room, &srcs[k], imgs, MAX_IMAGES, err, (int)sizeof err);
        if (m < 0) { fprintf(stderr, "room: %s\n", err); rc = -1; continue; }
        for (int q = 0; q < m; q++) {
            Source img = { imgs[q].x, imgs[q].y, imgs[q].z, 1.0f };
            float iaz = azimuth_of(&img, l);
            int idelay = itd_samples(iaz) + imgs[q].delay;
            float igl, igr; iid_gains(iaz, &igl, &igr);
            float ig = imgs[q].gain;
            for (int i = 0; i < frames; i++) {
                int j = i - idelay;
                float s = (j >= 0 && j < frames) ? mono[j] : 0.0f;
                out[2*i + 0] += s * ig * igl;
                out[2*i + 1] += s * ig * igr;
            }
        }
    }
    for (int i = 0; i < frames * 2; i++) out[i] = soft_clip(out[i]);
    return rc;
}

static float peak(const float* buf, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(buf[i]) > m) m = fabsf(buf[i]);
    return m;
}

static int failures = 0;
static void check(const char* name, int ok) {
    if (!ok) { printf("FAIL %s\n", name); failures++; }
}

/* --- independent per-face manual computation, used to cross-check the
      lattice enumeration in room_images --- */

/* fold an image coordinate back into [lo, hi] by mirroring at the walls;
   a correct image of source s folds back onto s */
static float fold_into(float xi, float lo, float hi) {
    float L = hi - lo;
    float t = fmodf(xi - lo, 2.0f * L);
    if (t < 0) t += 2.0f * L;
    if (t > L) t = 2.0f * L - t;
    return lo + t;
}

/* count wall crossings between the source cell and the image along one axis */
static void count_crossings(float xi, float lo, float hi, int* n_lo, int* n_hi) {
    float L = hi - lo;
    int K;
    if (xi >= lo) {
        /* boundaries lo + m*L (m = 1..K) lie below xi; odd m = hi face */
        K = (int)floorf((xi - lo) / L - 1e-4f);
        *n_hi = (K + 1) / 2;
        *n_lo = K / 2;
    } else {
        /* boundaries lo - m*L (m = 0..K-1) lie above xi; even m = lo face */
        K = (int)floorf((lo - xi) / L - 1e-4f) + 1;
        *n_lo = (K + 1) / 2;
        *n_hi = K / 2;
    }
}

int main(void) {
    float mono[MAX_FRAMES];
    float out[MAX_FRAMES * 2];
    for (int i = 0; i < 256; i++) mono[i] = (i == 64) ? 1.0f : 0.0f;  /* impulse at 64 */

    Listener l = { 0.0f };
    Source right = { 3.0f, 0.0f, 0.0f, 1.0f };
    render(&right, 1, &l, mono, 256, out, NULL);
    int best_l = 0, best_r = 0;
    for (int i = 1; i < 256; i++) {
        if (fabsf(out[2*i+0]) > fabsf(out[2*best_l+0])) best_l = i;
        if (fabsf(out[2*i+1]) > fabsf(out[2*best_r+1])) best_r = i;
    }
    printf("right_source peakL=%d peakR=%d (impulse@64)\n", best_l, best_r);

    Source left = { -3.0f, 0.0f, 0.0f, 1.0f };
    render(&left, 1, &l, mono, 256, out, NULL);
    int bl = 0, br = 0;
    for (int i = 1; i < 256; i++) {
        if (fabsf(out[2*i+0]) > fabsf(out[2*best_l+0])) bl = i;
        if (fabsf(out[2*i+1]) > fabsf(out[2*best_r+1])) br = i;
    }
    printf("left_source peakL=%d peakR=%d (impulse@64)\n", bl, br);

    Source near = { 0.0f, 0.0f, -1.0f, 1.0f };
    Source far  = { 0.0f, 0.0f, -9.0f, 1.0f };
    render(&near, 1, &l, mono, 256, out, NULL); float pn = peak(out, 256*2);
    render(&far, 1, &l, mono, 256, out, NULL);  float pf = peak(out, 256*2);
    printf("attenuation near=%.4f far=%.4f ratio=%.3f\n", pn, pf, pf / pn);

    Source many[4] = { {1,0,0,1}, {-1,0,0,1}, {0,0,-1,1}, {0,0,1,1} };
    float loud[MAX_FRAMES];
    for (int i = 0; i < 256; i++) loud[i] = 0.9f;
    render(many, 4, &l, loud, 256, out, NULL);
    printf("mix_peak=%.4f (soft clip 上限 1.0)\n", peak(out, 256*2));

    /* --- early reflections --- */
    float mn[3] = { -2.0f, -1.5f, -6.0f }, mx[3] = { 4.0f, 2.5f, 3.0f };
    Source src = { 1.0f, 0.5f, -1.0f, 1.0f };
    const float abs6[6] = { 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f };  /* per wall */
    float refl[6];
    for (int w = 0; w < 6; w++) refl[w] = 1.0f - abs6[w];

    Room room; room_init(&room, mn, mx, 4);
    for (int w = 0; w < 6; w++) room_set_wall(&room, w, abs6[w]);

    static ImageSource imgs[MAX_IMAGES];
    char err[128];
    int m = room_images(&room, &src, imgs, MAX_IMAGES, err, (int)sizeof err);
    check("images_order4_count", m == 128);   /* sum_{n=1..4} (4n^2+2) */

    /* cross-check every image against the manual per-face computation:
       fold the image back onto the source, count face crossings for the
       attenuation, recompute gain and delay independently */
    int fold_ok = 1;
    for (int q = 0; q < m && fold_ok; q++) {
        float pos[3] = { imgs[q].x, imgs[q].y, imgs[q].z };
        float sp[3] = { src.x, src.y, src.z };
        float atten = 1.0f;
        for (int a = 0; a < 3; a++) {
            if (fabsf(fold_into(pos[a], mn[a], mx[a]) - sp[a]) > 1e-3f) {
                fold_ok = 0; break;
            }
            int nlo, nhi;
            count_crossings(pos[a], mn[a], mx[a], &nlo, &nhi);
            atten *= powf(refl[2*a], (float)nlo) * powf(refl[2*a+1], (float)nhi);
        }
        if (!fold_ok) break;
        float d = sqrtf(pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]);
        float egain = src.gain * atten / (1.0f + d);
        int edelay = (int)lroundf(d / C_SOUND * SR);
        if (fabsf(imgs[q].gain - egain) > 1e-5f) fold_ok = 0;
        if (imgs[q].delay != edelay) fold_ok = 0;
    }
    check("images_match_manual_fold", fold_ok);

    /* literal hand-computed first-order images: reflect src across each wall */
    const float expect[6][3] = {
        { -5.0f,  0.5f,  -1.0f },   /* x = -2 wall */
        {  7.0f,  0.5f,  -1.0f },   /* x =  4 wall */
        {  1.0f, -3.5f,  -1.0f },   /* y = -1.5 wall */
        {  1.0f,  4.5f,  -1.0f },   /* y =  2.5 wall */
        {  1.0f,  0.5f, -11.0f },   /* z = -6 wall */
        {  1.0f,  0.5f,   7.0f },   /* z =  3 wall */
    };
    int found_all = 1;
    for (int e = 0; e < 6; e++) {
        int found = 0;
        for (int q = 0; q < m; q++)
            if (fabsf(imgs[q].x - expect[e][0]) < 1e-3f &&
                fabsf(imgs[q].y - expect[e][1]) < 1e-3f &&
                fabsf(imgs[q].z - expect[e][2]) < 1e-3f) found = 1;
        if (!found) found_all = 0;
    }
    check("first_order_positions", found_all);

    /* total reflected energy must converge as the order grows (absorbing
       walls: each order costs a factor of refl^2 per bounce) */
    float energy[7] = { 0.0f };
    Room eroom; room_init(&eroom, mn, mx, 0);
    for (int w = 0; w < 6; w++) room_set_wall(&eroom, w, 0.5f);
    int enum_ok = 1;
    for (int o = 1; o <= 6; o++) {
        eroom.order = o;
        int mm = room_images(&eroom, &src, imgs, MAX_IMAGES, err, (int)sizeof err);
        if (mm <= 0) { enum_ok = 0; break; }
        for (int q = 0; q < mm; q++) energy[o] += imgs[q].gain * imgs[q].gain;
    }
    int converges = enum_ok;
    for (int o = 2; o <= 6; o++) {
        float inc = energy[o] - energy[o-1];
        float prev = energy[o-1] - energy[o-2];
        if (!(inc > 0.0f && inc < prev)) converges = 0;
    }
    printf("reflected energy by order: %.5f %.5f %.5f %.5f %.5f %.5f\n",
           energy[1], energy[2], energy[3], energy[4], energy[5], energy[6]);
    check("energy_converges", converges);

    /* config errors */
    Room bad; room_init(&bad, mn, mx, 4);
    for (int w = 0; w < 5; w++) room_set_wall(&bad, w, 0.3f);  /* wall 5 missing */
    check("missing_material_error",
          room_images(&bad, &src, imgs, MAX_IMAGES, err, (int)sizeof err) == -1);

    Room neg; room_init(&neg, mn, mx, -1);
    for (int w = 0; w < 6; w++) room_set_wall(&neg, w, 0.3f);
    check("negative_order_error",
          room_images(&neg, &src, imgs, MAX_IMAGES, err, (int)sizeof err) == -1);

    check("bad_absorption_rejected",
          room_set_wall(&bad, 0, 1.5f) == -1 &&
          room_set_wall(&bad, 0, -0.2f) == -1 &&
          room_set_wall(&bad, 6, 0.5f) == -1);

    /* reflections off -> sample-identical to the dry render */
    float mono2[MAX_FRAMES];
    for (int i = 0; i < MAX_FRAMES; i++) mono2[i] = (i == 64) ? 1.0f : 0.0f;
    float out_a[MAX_FRAMES * 2], out_b[MAX_FRAMES * 2];
    Room off = room; off.enabled = 0;
    render(&src, 1, &l, mono2, 2048, out_a, &off);
    render(&src, 1, &l, mono2, 2048, out_b, NULL);
    check("off_is_sample_identical",
          memcmp(out_a, out_b, sizeof(float) * 2048 * 2) == 0);

    Room zero = room; zero.order = 0;
    render(&src, 1, &l, mono2, 2048, out_a, &zero);
    check("order0_is_dry", memcmp(out_a, out_b, sizeof(float) * 2048 * 2) == 0);

    /* a first-order reflection lands at its arrival time in the wet render */
    Room one = room; one.order = 1;
    render(&src, 1, &l, mono2, 2048, out_a, &one);
    Source img_xp = { 7.0f, 0.5f, -1.0f, 1.0f };   /* across x = 4 wall */
    float d7 = sqrtf(50.25f);
    int arrive = 64 + (int)lroundf(d7 / C_SOUND * SR)
                    + itd_samples(azimuth_of(&img_xp, &l));
    check("reflection_arrives_on_time",
          fabsf(out_a[2*arrive + 1]) > 1e-3f && out_b[2*arrive + 1] == 0.0f);

    check("render_flags_room_error",
          render(&src, 1, &l, mono2, 2048, out_a, &bad) == -1);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
