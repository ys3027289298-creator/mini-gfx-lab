
/* aniso-spec: anisotropic GGX highlight for brushed metal. The highlight
   stretches along the tangent direction, roughness is given per axis, and a
   toggle falls back to the original isotropic path bit-exactly. The self-test
   compares both paths against a double-precision per-direction analytic
   reference and bounds the brightness error. */
#include <stdio.h>
#include <math.h>

#define PI_F 3.14159265358979323846
#define MIN_ALPHA 1e-3f   /* clamp so D can never explode */

typedef struct { float x, y, z; } Vec3;
static Vec3 v3(float x, float y, float z) { Vec3 r = {x, y, z}; return r; }
static float dot3(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static Vec3 add3(Vec3 a, Vec3 b) { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static Vec3 sub3(Vec3 a, Vec3 b) { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static Vec3 scale3(Vec3 a, float s) { return v3(a.x*s, a.y*s, a.z*s); }
static Vec3 norm3(Vec3 a) { float l = sqrtf(dot3(a, a)); return scale3(a, 1.0f/l); }
static Vec3 cross3(Vec3 a, Vec3 b) {
    return v3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

typedef struct {
    float at, ab;      /* per-axis alpha = roughness^2; larger at stretches along T */
    float roughness;   /* scalar roughness for the isotropic fallback */
    int aniso_enabled;
} Aniso;

/* reject zero / out-of-range parameters instead of silently clamping them */
static int aniso_init(Aniso* a, float roughness, float anisotropy, int enabled) {
    if (!isfinite(roughness) || !isfinite(anisotropy)) return -1;
    if (roughness <= 0.0f || roughness > 1.0f) return -1;
    if (anisotropy < 0.0f || anisotropy >= 1.0f) return -1;
    float alpha = roughness * roughness;
    a->at = fmaxf(alpha * (1.0f + anisotropy), MIN_ALPHA);
    a->ab = fmaxf(alpha * (1.0f - anisotropy), MIN_ALPHA);
    a->roughness = roughness;
    a->aniso_enabled = enabled;
    return 0;
}

static float fresnel_schlick(float vdoth) {
    float m = 1.0f - vdoth;
    float m5 = m*m*m*m*m;
    return 0.04f + 0.96f * m5;
}

static float smith_g1_iso(float ndotw, float alpha) {
    float s = sqrtf(ndotw*ndotw + alpha*alpha * (1.0f - ndotw*ndotw));
    return 2.0f * ndotw / (ndotw + s);
}

/* the original isotropic GGX: exact Smith masking, frame-free */
float iso_specular(Vec3 n, Vec3 v, Vec3 l, float roughness) {
    float ndotv = dot3(n, v), ndotl = dot3(n, l);
    if (ndotv <= 0.0f || ndotl <= 0.0f) return 0.0f;
    Vec3 h = norm3(add3(v, l));
    float ndoth = dot3(n, h);
    float alpha = roughness * roughness;
    float a2 = alpha * alpha;
    float dd = ndoth*ndoth * (a2 - 1.0f) + 1.0f;
    float D = a2 / ((float)PI_F * dd * dd);
    float G = smith_g1_iso(ndotv, alpha) * smith_g1_iso(ndotl, alpha);
    float F = fresnel_schlick(dot3(v, h));
    return D * G * F / (4.0f * ndotv * ndotl);
}

static float smith_g1_aniso(const Aniso* a, Vec3 t, Vec3 b, Vec3 n, Vec3 w) {
    float wz = dot3(w, n);
    float wt = dot3(w, t) * a->at;
    float wb = dot3(w, b) * a->ab;
    float s = sqrtf(wz*wz + wt*wt + wb*wb);
    return 2.0f * wz / (wz + s);
}

/* Heitz anisotropic GGX; t need not be exactly perpendicular to n */
float aniso_specular(const Aniso* a, Vec3 n, Vec3 t, Vec3 v, Vec3 l) {
    if (!a->aniso_enabled)
        return iso_specular(n, v, l, a->roughness);   /* 与改动前完全一致 */
    Vec3 tn = norm3(sub3(t, scale3(n, dot3(n, t))));
    Vec3 b = cross3(n, tn);
    float ndotv = dot3(n, v), ndotl = dot3(n, l);
    if (ndotv <= 0.0f || ndotl <= 0.0f) return 0.0f;
    Vec3 h = norm3(add3(v, l));
    float ht = dot3(h, tn) / a->at;
    float hb = dot3(h, b) / a->ab;
    float hn = dot3(h, n);
    float den = ht*ht + hb*hb + hn*hn;
    float D = 1.0f / ((float)PI_F * a->at * a->ab * den * den);
    float G = smith_g1_aniso(a, tn, b, n, v) * smith_g1_aniso(a, tn, b, n, l);
    float F = fresnel_schlick(dot3(v, h));
    return D * G * F / (4.0f * ndotv * ndotl);
}

/* double-precision analytic reference, evaluated per light direction */
static double ref_specular_d(Vec3 n, Vec3 t, Vec3 v, Vec3 l, double at, double ab) {
    double nx = n.x, ny = n.y, nz = n.z;
    double dnt = n.x*t.x + n.y*t.y + n.z*t.z;
    double tx = t.x - nx*dnt, ty = t.y - ny*dnt, tz = t.z - nz*dnt;
    double tl = sqrt(tx*tx + ty*ty + tz*tz);
    tx /= tl; ty /= tl; tz /= tl;
    double bx = ny*tz - nz*ty, by = nz*tx - nx*tz, bz = nx*ty - ny*tx;
    double ndotv = nx*v.x + ny*v.y + nz*v.z;
    double ndotl = nx*l.x + ny*l.y + nz*l.z;
    if (ndotv <= 0.0 || ndotl <= 0.0) return 0.0;
    double hx = v.x + l.x, hy = v.y + l.y, hz = v.z + l.z;
    double hl = sqrt(hx*hx + hy*hy + hz*hz);
    hx /= hl; hy /= hl; hz /= hl;
    double ht = (hx*tx + hy*ty + hz*tz) / at;
    double hb = (hx*bx + hy*by + hz*bz) / ab;
    double hn = hx*nx + hy*ny + hz*nz;
    double den = ht*ht + hb*hb + hn*hn;
    double D = 1.0 / (PI_F * at * ab * den * den);
    double G = 1.0;
    for (int i = 0; i < 2; i++) {
        Vec3 w = i == 0 ? v : l;
        double wz = nx*w.x + ny*w.y + nz*w.z;
        double wt = (w.x*tx + w.y*ty + w.z*tz) * at;
        double wb = (w.x*bx + w.y*by + w.z*bz) * ab;
        G *= 2.0 * wz / (wz + sqrt(wz*wz + wt*wt + wb*wb));
    }
    double vdoth = v.x*hx + v.y*hy + v.z*hz;
    double m = 1.0 - vdoth;
    double F = 0.04 + 0.96 * m*m*m*m*m;
    return D * G * F / (4.0 * ndotv * ndotl);
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Vec3 n = v3(0.0f, 0.0f, 1.0f);
    Vec3 t = v3(1.0f, 0.0f, 0.0f);
    Vec3 v = norm3(v3(0.3f, 0.1f, 1.0f));
    Aniso a;

    /* 参数校验：粗糙度为零或越界必须拒绝 */
    check("reject_zero_roughness", aniso_init(&a, 0.0f, 0.5f, 1) != 0);
    check("reject_negative_roughness", aniso_init(&a, -0.2f, 0.5f, 1) != 0);
    check("reject_roughness_above_one", aniso_init(&a, 1.5f, 0.5f, 1) != 0);
    check("reject_nan", aniso_init(&a, NAN, 0.5f, 1) != 0);
    check("reject_full_anisotropy", aniso_init(&a, 0.5f, 1.0f, 1) != 0);
    check("accept_valid", aniso_init(&a, 0.4f, 0.8f, 1) == 0);

    /* 高光应沿切线方向拉长：同样的倾角，T 平面内比 B 平面内更亮 */
    Vec3 vn = v3(0.0f, 0.0f, 1.0f);
    float tilt = 0.35f;
    Vec3 lt = norm3(v3(sinf(tilt), 0.0f, cosf(tilt)));
    Vec3 lb = norm3(v3(0.0f, sinf(tilt), cosf(tilt)));
    float spec_t = aniso_specular(&a, n, t, vn, lt);
    float spec_b = aniso_specular(&a, n, t, vn, lb);
    check("stretch_along_tangent", spec_t > spec_b);
    check("iso_stays_round",
          iso_specular(n, vn, lt, 0.4f) == iso_specular(n, vn, lb, 0.4f));

    /* 关闭各向异性时必须与原版逐位一致 */
    Aniso off;
    aniso_init(&off, 0.4f, 0.8f, 0);
    float maxdiff = 0.0f;
    for (int deg = 0; deg < 360; deg += 7) {
        float ph = (float)deg * (float)PI_F / 180.0f, th = 0.9f;
        Vec3 l = v3(sinf(th)*cosf(ph), sinf(th)*sinf(ph), cosf(th));
        float d = fabsf(aniso_specular(&off, n, t, v, l) - iso_specular(n, v, l, 0.4f));
        if (d > maxdiff) maxdiff = d;
    }
    check("toggle_off_bitexact", maxdiff == 0.0f);

    /* anisotropy = 0 时应退化为各向同性 */
    Aniso zero;
    aniso_init(&zero, 0.4f, 0.0f, 1);
    float maxrel_zero = 0.0f;
    for (int deg = 0; deg < 360; deg += 7) {
        float ph = (float)deg * (float)PI_F / 180.0f, th = 0.9f;
        Vec3 l = v3(sinf(th)*cosf(ph), sinf(th)*sinf(ph), cosf(th));
        float fa = aniso_specular(&zero, n, t, v, l);
        float fi = iso_specular(n, v, l, 0.4f);
        float rel = fabsf(fa - fi) / fmaxf(fi, 1e-3f);
        if (rel > maxrel_zero) maxrel_zero = rel;
    }
    check("zero_aniso_matches_iso", maxrel_zero < 1e-4f);

    /* 与逐方向解析参考（双精度）比对，给出亮度误差上界 */
    const double bound = 2e-3;
    double maxrel_aniso = 0.0, maxrel_iso = 0.0;
    double alpha = 0.4 * 0.4;
    for (int dt = 5; dt <= 85; dt += 5) {
        for (int dp = 0; dp < 360; dp += 15) {
            double th = dt * PI_F / 180.0, ph = dp * PI_F / 180.0;
            Vec3 l = v3((float)(sin(th)*cos(ph)), (float)(sin(th)*sin(ph)), (float)cos(th));
            double ra = ref_specular_d(n, t, v, l, a.at, a.ab);
            double fa = aniso_specular(&a, n, t, v, l);
            double rel = fabs(fa - ra) / fmax(ra, 1e-3);
            if (rel > maxrel_aniso) maxrel_aniso = rel;
            double ri = ref_specular_d(n, t, v, l, alpha, alpha);
            double fi = iso_specular(n, v, l, 0.4f);
            rel = fabs(fi - ri) / fmax(ri, 1e-3);
            if (rel > maxrel_iso) maxrel_iso = rel;
        }
    }
    printf("aniso_max_rel_err=%.3g iso_max_rel_err=%.3g bound=%.3g\n",
           maxrel_aniso, maxrel_iso, bound);
    check("aniso_error_bounded", maxrel_aniso <= bound);
    check("iso_error_bounded", maxrel_iso <= bound);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
