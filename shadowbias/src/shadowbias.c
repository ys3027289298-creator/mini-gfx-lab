
/* shadowbias: angle-adaptive shadow-map bias.
   Software shadow-map simulation over a thin-wall scene.  Two orthographic
   depth maps are rendered per frame -- ground-only and box(occluder)-only --
   so self-shadow acne (quantization / discretization of the receiver's own
   surface) is measured separately from true occlusion and cannot be polluted
   by two surfaces sharing one texel.

   For every depth-map texel a per-axis central-difference depth slope
   |grad z| is derived together with tan(theta) from the stored surface
   normal and the light direction, and the bias is picked adaptively in
   depth quanta:

       bias = clamp(BASE_Q + slope_scale * max(|grad z|, tan theta), ..., cap)

   slope_scale is calibrated from the shadow-map dimensions rather than hand
   tuned.  Head-on surfaces have zero slope and get the constant BASE_Q;
   grazing surfaces get bias scaled by the slope.  When the slope cannot be
   derived (edge texel, empty neighbours on both axes, depth discontinuity,
   non-finite value) a fixed worst-case bias is used and the fallback counted.

   Each camera pixel is classified against a ray-cast ground truth for the
   hand-tuned fixed bias and the adaptive bias.  A pixel misread as shadowed
   is a streak (self-shadow acne / over-biased overlap), a pixel misread as
   lit is a light leak / detached shadow (peter-panning). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

#define PI_F 3.14159265358979323846f

/* ---- shadow map and frame setup ----------------------------------------- */
#define SM        64        /* shadow map resolution per axis            */
#define QBITS     12
#define QMAX      ((1 << QBITS) - 1)
#define EXT       12.0f     /* orthographic light frustum half extent     */
#define ZN        (-16.0f)  /* light-space depth range                    */
#define ZF        (14.0f)
#define IMG_W     120
#define IMG_H     90
#define FRAMES    6
#define NBOX      4
#define EPS       1e-4f
#define FAR       60.0f

/* bias parameters, in depth quanta.
   The slope scale is derived from the shadow-map geometry instead of being
   hand-tuned: one texel of receiver-depth variation at slope m is
       qdepth * (texel_world_size / 2) * m  quanta,
   so SLOPE_K = 1 is exactly the bias that covers quantization acne across a
   whole texel at the measured slope. */
#define BASE_Q        2       /* constant part on a head-on surface         */
#define SLOPE_K       1.0f    /* multiplier on the derived one-texel scale  */
#define FALLBACK_DEG  78.5f   /* worst-case grazing angle for an unknown    */
                              /* slope: thin-wall texels are all silhouettes*/
#define CAP_DEG       85.0f   /* slope beyond this angle is clamped         */
#define JUMP_QUANTA   16      /* neighbour depth jump, in depth quanta,     */
                              /* past which a neighbour is a silhouette     */

static float g_qdepth;         /* depth quanta per world unit               */
static float g_texel;          /* shadow-map texel size in world units      */
static float g_slope_scale;    /* quanta per unit of effective slope        */
static int   g_fallback_q;     /* fixed bias used when slope is undecidable */
static int   g_cap_q;

static void bias_calibrate(void) {
    g_qdepth = (float)QMAX / (ZF - ZN);
    g_texel = 2.0f * EXT / SM;
    g_slope_scale = g_qdepth * g_texel * 0.5f * SLOPE_K;
    float fb_rad = FALLBACK_DEG * PI_F / 180.0f;
    float cap_rad = CAP_DEG * PI_F / 180.0f;
    g_fallback_q = BASE_Q + (int)lroundf(g_slope_scale * tanf(fb_rad));
    g_cap_q = BASE_Q + (int)lroundf(g_slope_scale * tanf(cap_rad));
}

typedef struct { float x, y, z; } Vec;
typedef struct { Vec min, max; int id; } Box;

typedef struct {
    int   id;               /* 0 = ground plane, 1..NBOX = box, -1 = miss */
    Vec   p, n;
} Hit;

static Vec vec(float x, float y, float z) { Vec v = {x, y, z}; return v; }
static float dot(Vec a, Vec b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static Vec sub(Vec a, Vec b) { return vec(a.x-b.x, a.y-b.y, a.z-b.z); }
static Vec add(Vec a, Vec b) { return vec(a.x+b.x, a.y+b.y, a.z+b.z); }
static Vec mul(Vec a, float k) { return vec(a.x*k, a.y*k, a.z*k); }
static Vec cross(Vec a, Vec b) {
    return vec(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
static Vec norm(Vec a) {
    float l = sqrtf(dot(a, a));
    return l > 0 ? mul(a, 1.0f / l) : a;
}

/* ---- scene: ground plane plus thin walls and a floating box ------------- */
static Box boxes[NBOX];

static void scene_init(void) {
    /* wall A/B: 0.06-thin in x (aligned with the light's sweep plane) */
    boxes[0].min = vec(-2.03f, -1.5f, 0.0f); boxes[0].max = vec(-1.97f, 1.5f, 1.5f);
    boxes[1].min = vec( 1.17f, -2.2f, 0.0f); boxes[1].max = vec( 1.23f, 0.2f, 1.2f);
    /* wall C: 0.06-thin in y, drives silhouette / undersampling artefacts */
    boxes[2].min = vec( 0.3f,  1.37f, 0.0f); boxes[2].max = vec( 2.1f, 1.43f, 0.9f);
    /* floating occluder: real contact shadow, peter-panning probe */
    boxes[3].min = vec(-0.25f, -0.05f, 1.95f); boxes[3].max = vec( 0.85f, 1.05f, 2.65f);
    for (int i = 0; i < NBOX; i++) boxes[i].id = i + 1;
}

/* nearest ray intersection.  layer: 1 = ground only, 2 = boxes only.
   skip ignores one primitive id (used by ground-truth shadow rays). */
static int cast_ray(Vec o, Vec d, int layer, int skip, Hit* h) {
    float best = FAR;
    h->id = -1;
    if (layer != 2 && d.z < -EPS && skip != 0) {
        float t = -o.z / d.z;                    /* ground plane z = 0 */
        if (t > EPS && t < best) {
            best = t; h->id = 0; h->p = add(o, mul(d, t)); h->n = vec(0, 0, 1);
        }
    }
    if (layer == 1) return h->id;
    for (int b = 0; b < NBOX; b++) {
        if (boxes[b].id == skip) continue;
        float t0 = 0.0f, t1 = FAR; int axis = -1, sign = 0;
        const float* oa = &o.x; const float* da = &d.x;
        const float* lo = &boxes[b].min.x; const float* hi = &boxes[b].max.x;
        for (int k = 0; k < 3; k++) {
            if (fabsf(da[k]) < EPS) { if (oa[k] < lo[k] || oa[k] > hi[k]) { t0 = 1; t1 = 0; break; } }
            else {
                float ta = (lo[k] - oa[k]) / da[k];
                float tb = (hi[k] - oa[k]) / da[k];
                int sg = da[k] < 0;
                if (ta > tb) { float tmp = ta; ta = tb; tb = tmp; sg = !sg; }
                if (ta > t0) { t0 = ta; axis = k; sign = sg; }
                if (tb < t1) t1 = tb;
            }
        }
        if (t0 < t1 && t0 > EPS && t0 < best) {
            best = t0; h->id = boxes[b].id; h->p = add(o, mul(d, t0));
            Vec n = {0, 0, 0}; (&n.x)[axis] = sign ? 1.0f : -1.0f; h->n = n;
        }
    }
    return h->id;
}

/* ---- per-frame light basis ---------------------------------------------- */
typedef struct {
    Vec   L;             /* unit vector pointing *toward* the light      */
    Vec   r, u;          /* light frustum right / up, depth axis is -L   */
} Light;

static Light light_make(float elev_deg) {
    float el = elev_deg * PI_F / 180.0f;
    float s = sinf(el), c = cosf(el);
    Light g;
    g.L = vec(-s, 0, c);                   /* toward light, c > 0        */
    g.r = vec(0, 1, 0);
    g.u = vec(-c, 0, -s);                 /* perpendicular to L and r   */
    return g;
}

static float light_depth(const Light* g, Vec p) { return -dot(g->L, p); }

/* ---- shadow map ---------------------------------------------------------- */
typedef struct {
    float   z[SM * SM];      /* float depth, inf for empty texels        */
    int32_t q[SM * SM];      /* quantized depth, QMAX when empty         */
    int     empty[SM * SM];
    int     ncode[SM * SM];  /* stored surface normal axis code 0..5     */
} ShadowMap;

/* normal codes: +x -x +y -y +z -z */
static int normal_code(Vec n) {
    const Vec tab[6] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    int best = 0;
    for (int k = 1; k < 6; k++) if (dot(n, tab[k]) > dot(n, tab[best])) best = k;
    return best;
}
static Vec code_normal(int code) {
    const Vec tab[6] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    return tab[code];
}

static int quantize(float z) {
    int q = (int)((z - ZN) / (ZF - ZN) * QMAX + 0.5f);
    if (q < 0) q = 0;
    if (q > QMAX) q = QMAX;
    return q;
}

static void sm_render(const Light* g, int layer, ShadowMap* sm) {
    float ds = 2.0f * EXT / SM;
    for (int j = 0; j < SM; j++) {
        for (int i = 0; i < SM; i++) {
            int idx = j * SM + i;
            float lx = -EXT + (i + 0.5f) * ds;
            float ly = -EXT + (j + 0.5f) * ds;
            Vec o = add(add(mul(g->r, lx), mul(g->u, ly)), mul(g->L, -ZN));
            Vec d = mul(g->L, -1.0f);
            Hit h;
            if (cast_ray(o, d, layer, -1, &h) < 0) {
                sm->empty[idx] = 1; sm->z[idx] = INFINITY; sm->q[idx] = QMAX;
                sm->ncode[idx] = 4;
            } else {
                sm->empty[idx] = 0;
                sm->z[idx] = light_depth(g, h.p);
                sm->q[idx] = quantize(sm->z[idx]);
                sm->ncode[idx] = normal_code(h.n);
            }
        }
    }
}

/* ---- slope grid and adaptive bias --------------------------------------- */
typedef struct {
    int   valid;           /* slope can be derived for this texel         */
    float slope;           /* |dz/d(light xy)| from neighbour depths      */
    float tan_inc;         /* tan of light incidence angle, stored normal */
} Cell;

static void slope_build(const Light* g, const ShadowMap* sm, Cell* cell) {
    float ds = 2.0f * EXT / SM;
    float jump_world = (float)JUMP_QUANTA * (ZF - ZN) / (float)QMAX;
    for (int j = 0; j < SM; j++) {
        for (int i = 0; i < SM; i++) {
            int idx = j * SM + i;
            Vec n = code_normal(sm->ncode[idx]);
            float cos_t = dot(n, g->L);
            if (cos_t < EPS) cos_t = EPS;
            cell[idx].tan_inc = sqrtf(fmaxf(0.0f, 1.0f - cos_t * cos_t)) / cos_t;
            cell[idx].valid = 0;
            cell[idx].slope = 0.0f;
            if (sm->empty[idx]) continue;
            /* per-axis central difference: an axis is dropped when either
               neighbour is empty or crosses a silhouette, but the other
               axis still yields a usable slope */
            float dzdx = 0.0f, dzdy = 0.0f; int axes = 0;
            if (i > 0 && i < SM - 1) {
                int xl = idx - 1, xr = idx + 1;
                if (!sm->empty[xl] && !sm->empty[xr] &&
                    fabsf(sm->z[xr] - sm->z[xl]) <= jump_world) {
                    dzdx = (sm->z[xr] - sm->z[xl]) / (2.0f * ds); axes++;
                }
            }
            if (j > 0 && j < SM - 1) {
                int yd = idx - SM, yu = idx + SM;
                if (!sm->empty[yd] && !sm->empty[yu] &&
                    fabsf(sm->z[yu] - sm->z[yd]) <= jump_world) {
                    dzdy = (sm->z[yu] - sm->z[yd]) / (2.0f * ds); axes++;
                }
            }
            if (axes == 0 || !isfinite(dzdx) || !isfinite(dzdy)) continue;
            cell[idx].slope = sqrtf(dzdx * dzdx + dzdy * dzdy);
            cell[idx].valid = 1;
        }
    }
}

/* adaptive bias in quanta; counts every fallback to the fixed value */
static int adaptive_bias(const Cell* c, int* fallbacks) {
    if (!c->valid) { (*fallbacks)++; return g_fallback_q; }
    float eff = fmaxf(c->slope, c->tan_inc);   /* head-on -> 0, grazing -> large */
    int b = BASE_Q + (int)lroundf(g_slope_scale * eff);
    if (b > g_cap_q) b = g_cap_q;
    return b;
}

static int sm_texel(const Light* g, Vec p, int* ti, int* tj) {
    float lx = dot(g->r, p), ly = dot(g->u, p);
    if (lx < -EXT || lx > EXT || ly < -EXT || ly > EXT) return 0;
    *ti = (int)((lx + EXT) / (2.0f * EXT) * SM);
    *tj = (int)((ly + EXT) / (2.0f * EXT) * SM);
    if (*ti < 0) *ti = 0;
    if (*ti >= SM) *ti = SM - 1;
    if (*tj < 0) *tj = 0;
    if (*tj >= SM) *tj = SM - 1;
    return 1;
}

/* map lookup against one depth map; 1 = shadowed.
   mode 0: fixed bias in quanta; mode 1: adaptive.
   Bias exists to cancel the receiver's own quantization acne, so callers
   pass bias_occl = 0 for a depth map that only stores occluders (the box
   map queried from the ground): biasing an occluder depth just detaches the
   contact shadow and buys nothing. */
static int shadow_test(const Light* g, const ShadowMap* sm, const Cell* cell,
                       Vec p, int mode, int fixed_q, int bias_occl,
                       int* fallbacks) {
    int ti, tj;
    if (!sm_texel(g, p, &ti, &tj)) return 0;
    int idx = tj * SM + ti;
    if (sm->empty[idx]) return 0;
    int qrec = quantize(light_depth(g, p));
    int bias;
    if (bias_occl) bias = 0;
    else bias = mode ? adaptive_bias(&cell[idx], fallbacks) : fixed_q;
    return qrec > sm->q[idx] + bias;
}

/* ---- camera -------------------------------------------------------------- */
typedef struct { Vec eye, f, r, u; float half_w, half_h; } Camera;

static Camera camera_make(void) {
    Camera c;
    c.eye = vec(-4.0f, -5.0f, 3.4f);
    Vec tgt = vec(-0.4f, 0.0f, 0.8f);
    c.f = norm(sub(tgt, c.eye));
    c.r = norm(cross(c.f, vec(0, 0, 1)));
    c.u = cross(c.r, c.f);
    float fov = 55.0f * PI_F / 180.0f;
    c.half_h = tanf(fov * 0.5f);
    c.half_w = c.half_h * ((float)IMG_W / IMG_H);
    return c;
}

/* ground-truth occlusion: shadow ray toward the light, skipping the
   primitive the receiver point lies on */
static int truth_shadow(const Light* g, Vec p, int self_id) {
    Hit h;
    return cast_ray(add(p, mul(g->L, EPS * 10.0f)), g->L, 0, self_id, &h) >= 0;
}

/* ---- per-frame pixel results --------------------------------------------- */
#define CLASS_SKY     0
#define CLASS_LIT     1
#define CLASS_SHADOW  2

typedef struct {
    unsigned char truth[IMG_H * IMG_W];
    unsigned char fixed[IMG_H * IMG_W];
    unsigned char adapt[IMG_H * IMG_W];
    int tested;
    int gf_streak, gf_leak;      /* ground receiver, fixed bias            */
    int ga_streak, ga_leak;      /* ground receiver, adaptive bias         */
    int wf_streak, wf_leak;      /* wall / box receiver, fixed bias        */
    int wa_streak, wa_leak;      /* wall / box receiver, adaptive bias     */
    int f_streak, f_leak;        /* fixed bias, both classes               */
    int a_streak, a_leak;        /* adaptive bias, both classes            */
    int fallbacks;
    int slope_valid, slope_used; /* filled texels with/without usable slope */
} FrameResult;

static void render_frame(const Light* g, const ShadowMap* smg, const Cell* cg,
                         const ShadowMap* smb, const Cell* cb,
                         const Camera* cam, int fixed_q, FrameResult* fr) {
    memset(fr, 0, sizeof(*fr));
    for (int t = 0; t < SM * SM; t++) {
        if (!smb->empty[t]) { fr->slope_used++; if (cb[t].valid) fr->slope_valid++; }
    }
    for (int y = 0; y < IMG_H; y++) {
        for (int x = 0; x < IMG_W; x++) {
            int px = y * IMG_W + x;
            float u = (x + 0.5f) / IMG_W * 2.0f - 1.0f;
            float v = 1.0f - (y + 0.5f) / IMG_H * 2.0f;
            Vec d = norm(add(add(cam->f, mul(cam->r, u * cam->half_w)),
                             mul(cam->u, v * cam->half_h)));
            Hit h;
            if (cast_ray(cam->eye, d, 0, -1, &h) < 0) {
                fr->truth[px] = fr->fixed[px] = fr->adapt[px] = CLASS_SKY;
                continue;
            }
            fr->tested++;
            int ts = truth_shadow(g, h.p, h.id);
            fr->truth[px] = ts ? CLASS_SHADOW : CLASS_LIT;

            int fs, as;
            if (h.id == 0) {
                /* ground receiver: the ground map is the receiver's own
                   surface -> adaptive/fixed bias for acne; the box map is a
                   pure occluder -> no bias, else the contact shadow detaches */
                int f_self = shadow_test(g, smg, cg, h.p, 0, fixed_q, 0, &fr->fallbacks);
                int a_self = shadow_test(g, smg, cg, h.p, 1, fixed_q, 0, &fr->fallbacks);
                int f_box  = shadow_test(g, smb, cb, h.p, 0, fixed_q, 1, &fr->fallbacks);
                int a_box  = shadow_test(g, smb, cb, h.p, 1, fixed_q, 1, &fr->fallbacks);
                fs = f_self || f_box;
                as = a_self || a_box;
            } else {
                /* wall/box receiver: box map is its own surface -> bias */
                fs = shadow_test(g, smb, cb, h.p, 0, fixed_q, 0, &fr->fallbacks);
                as = shadow_test(g, smb, cb, h.p, 1, fixed_q, 0, &fr->fallbacks);
            }
            fr->fixed[px] = fs ? CLASS_SHADOW : CLASS_LIT;
            fr->adapt[px] = as ? CLASS_SHADOW : CLASS_LIT;

            if (fs && !ts) {
                fr->f_streak++;
                if (h.id == 0) fr->gf_streak++; else fr->wf_streak++;
            }
            if (!fs && ts) {
                fr->f_leak++;
                if (h.id == 0) fr->gf_leak++; else fr->wf_leak++;
            }
            if (as && !ts) {
                fr->a_streak++;
                if (h.id == 0) fr->ga_streak++; else fr->wa_streak++;
            }
            if (!as && ts) {
                fr->a_leak++;
                if (h.id == 0) fr->ga_leak++; else fr->wa_leak++;
            }
        }
    }
}

/* ---- PPM montage: truth / fixed / adaptive / diff rows ------------------- */
static void write_ppm(const char* path, const FrameResult* fr) {
    FILE* fp = fopen(path, "wb");
    if (!fp) { perror(path); return; }
    fprintf(fp, "P6\n%d %d\n255\n", IMG_W, IMG_H * 4);
    const unsigned char pal_lit[3] = {235, 235, 230};
    const unsigned char pal_shd[3] = {60, 60, 72};
    const unsigned char pal_sky[3] = {15, 15, 20};
    const unsigned char pal_agree_lit[3] = {240, 240, 240};
    const unsigned char pal_agree_shd[3] = {90, 90, 96};
    const unsigned char pal_streak[3] = {220, 60, 50};   /* red: false shadow */
    const unsigned char pal_leak[3]   = {240, 170, 40};  /* orange: false lit  */
    for (int row = 0; row < 4; row++) {
        for (int px = 0; px < IMG_W * IMG_H; px++) {
            const unsigned char* col;
            if (row < 3) {
                unsigned char c = row == 0 ? fr->truth[px]
                                 : row == 1 ? fr->fixed[px] : fr->adapt[px];
                col = c == CLASS_SKY ? pal_sky : c == CLASS_LIT ? pal_lit : pal_shd;
            } else if (fr->truth[px] == CLASS_SKY) {
                col = pal_sky;
            } else if (fr->adapt[px] == fr->truth[px]) {
                col = fr->truth[px] == CLASS_LIT ? pal_agree_lit : pal_agree_shd;
            } else {
                col = fr->adapt[px] == CLASS_SHADOW ? pal_streak : pal_leak;
            }
            fwrite(col, 1, 3, fp);
        }
    }
    fclose(fp);
}

/* ---- test scaffolding ---------------------------------------------------- */
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(int argc, char** argv) {
    int dump = argc > 1 && strcmp(argv[1], "dump") == 0;
    scene_init();
    bias_calibrate();
    Camera cam = camera_make();
    const float elevs[FRAMES] = {-60, -36, -12, 12, 36, 60};

    FrameResult frames[FRAMES];
    const int sweep[] = {0, 2, 4, 8, 16, 32, 64, 128};
    const int NSW = (int)(sizeof(sweep) / sizeof(sweep[0]));
    int sweep_err[8], sweep_str[8], sweep_leak[8];
    memset(sweep_err, 0, sizeof(sweep_err));
    memset(sweep_str, 0, sizeof(sweep_str));
    memset(sweep_leak, 0, sizeof(sweep_leak));

    printf("shadowmap=%dx%d depth=%dbit image=%dx%d frames=%d\n",
           SM, SM, QBITS, IMG_W, IMG_H, FRAMES);
    printf("calibrated slope_scale=%.2f quanta/slope fallback=%d cap=%d quanta\n",
           g_slope_scale, g_fallback_q, g_cap_q);

    /* pass 1: fixed-bias sweep to pick the hand-tuned reference */
    for (int k = 0; k < NSW; k++) {
        for (int f = 0; f < FRAMES; f++) {
            Light g = light_make(elevs[f]);
            ShadowMap smg, smb; Cell cg[SM * SM], cb[SM * SM];
            sm_render(&g, 1, &smg); slope_build(&g, &smg, cg);
            sm_render(&g, 2, &smb); slope_build(&g, &smb, cb);
            FrameResult fr;
            render_frame(&g, &smg, cg, &smb, cb, &cam, sweep[k], &fr);
            sweep_err[k] += fr.f_streak + fr.f_leak;
            sweep_str[k] += fr.f_streak;
            sweep_leak[k] += fr.f_leak;
        }
    }
    printf("fixed_bias_sweep quanta:streak:leak:total\n");
    int hand = 0;
    for (int k = 0; k < NSW; k++) {
        printf("  b=%3d streak=%4d leak=%4d total=%4d\n",
               sweep[k], sweep_str[k], sweep_leak[k], sweep_err[k]);
        if (sweep_err[k] < sweep_err[hand]) hand = k;   /* tie keeps smaller bias */
    }
    int hand_q = sweep[hand];
    printf("hand_picked_bias_quanta=%d\n", hand_q);

    /* pass 2: hand-tuned fixed vs adaptive, per frame */
    int tot_tested = 0, tot_fs = 0, tot_fl = 0, tot_as = 0, tot_al = 0, tot_fb = 0;
    int gf_s = 0, gf_l = 0, ga_s = 0, ga_l = 0;
    int wf_s = 0, wf_l = 0, wa_s = 0, wa_l = 0;
    ShadowMap det_g, det_b;
    for (int f = 0; f < FRAMES; f++) {
        Light g = light_make(elevs[f]);
        ShadowMap smg, smb; Cell cg[SM * SM], cb[SM * SM];
        sm_render(&g, 1, &smg); slope_build(&g, &smg, cg);
        sm_render(&g, 2, &smb); slope_build(&g, &smb, cb);
        if (f == 0) { det_g = smg; det_b = smb; }
        render_frame(&g, &smg, cg, &smb, cb, &cam, hand_q, &frames[f]);
        FrameResult* fr = &frames[f];
        printf("frame=%d elev=%+.0f tested=%d | fixed streak=%d leak=%d | "
               "adapt streak=%d leak=%d fallback=%d\n",
               f, elevs[f], fr->tested, fr->f_streak, fr->f_leak,
               fr->a_streak, fr->a_leak, fr->fallbacks);
        printf("         boxmap slope_valid=%d/%d texels (%.0f%%)\n",
               fr->slope_valid, fr->slope_used,
               100.0f * fr->slope_valid / (fr->slope_used ? fr->slope_used : 1));
        printf("         ground fixed(s=%d l=%d) adapt(s=%d l=%d) | "
               "wall fixed(s=%d l=%d) adapt(s=%d l=%d)\n",
               fr->gf_streak, fr->gf_leak, fr->ga_streak, fr->ga_leak,
               fr->wf_streak, fr->wf_leak, fr->wa_streak, fr->wa_leak);
        tot_tested += fr->tested;
        tot_fs += fr->f_streak; tot_fl += fr->f_leak;
        tot_as += fr->a_streak; tot_al += fr->a_leak; tot_fb += fr->fallbacks;
        gf_s += fr->gf_streak; gf_l += fr->gf_leak;
        ga_s += fr->ga_streak; ga_l += fr->ga_leak;
        wf_s += fr->wf_streak; wf_l += fr->wf_leak;
        wa_s += fr->wa_streak; wa_l += fr->wa_leak;
    }
    printf("totals tested=%d\n", tot_tested);
    printf("  fixed streak=%d leak=%d errors=%d\n", tot_fs, tot_fl, tot_fs + tot_fl);
    printf("  adapt streak=%d leak=%d errors=%d fallback=%d\n",
           tot_as, tot_al, tot_as + tot_al, tot_fb);
    printf("  ground: fixed(s=%d l=%d) adapt(s=%d l=%d)\n", gf_s, gf_l, ga_s, ga_l);
    printf("  wall:   fixed(s=%d l=%d) adapt(s=%d l=%d)\n", wf_s, wf_l, wa_s, wa_l);

    /* determinism: re-render the first frame, depth grids must match */
    Light g0 = light_make(elevs[0]);
    ShadowMap rg, rb;
    sm_render(&g0, 1, &rg);
    sm_render(&g0, 2, &rb);
    check("deterministic_render",
          memcmp(rg.q, det_g.q, sizeof(rg.q)) == 0 &&
          memcmp(rb.q, det_b.q, sizeof(rb.q)) == 0);

    /* bias unit tests */
    int fb = 0;
    Cell bad = {0, 9.0f, 9.0f};
    check("fallback_value", adaptive_bias(&bad, &fb) == g_fallback_q);
    check("fallback_counted", fb == 1);
    Cell flat = {1, 0.0f, 0.0f};
    int b_head = adaptive_bias(&flat, &fb);
    Cell graze = {1, 1.732f, 1.732f};
    int b_graze = adaptive_bias(&graze, &fb);
    Cell mid = {1, 0.8f, 0.8f};
    int b_mid = adaptive_bias(&mid, &fb);
    check("head_on_is_base", b_head == BASE_Q);
    check("grazing_above_head_on", b_graze > b_head);
    check("bias_monotonic", b_head <= b_mid && b_mid <= b_graze);
    Cell cap_cell = {1, 1000.0f, 1000.0f};
    check("bias_capped", adaptive_bias(&cap_cell, &fb) == g_cap_q);
    check("no_fallback_when_valid", fb == 1);
    check("fallback_above_base", g_fallback_q > BASE_Q);
    check("slope_scale_derived",
          fabsf(g_slope_scale - g_qdepth * g_texel * 0.5f * SLOPE_K) < 1e-3f);

    /* outcome requirements */
    check("adapt_fewer_streaks", tot_as <= tot_fs);
    check("adapt_fewer_leaks", tot_al <= tot_fl);
    check("adapt_wins_total", tot_as + tot_al < tot_fs + tot_fl);
    check("fallback_observed", tot_fb > 0);
    int argmin = 0;
    for (int k = 1; k < NSW; k++)
        if (sweep_err[k] < sweep_err[argmin]) argmin = k;
    check("hand_pick_is_argmin", hand == argmin && hand_q == sweep[argmin]);

    if (dump) {
        if (mkdir("out", 0755) != 0 && errno != EEXIST) { perror("mkdir out"); return 1; }
        for (int f = 0; f < FRAMES; f++) {
            char path[64];
            snprintf(path, sizeof(path), "out/frame%d.ppm", f);
            write_ppm(path, &frames[f]);
        }
        printf("ppm_written=%d (rows: truth fixed adaptive diff)\n", FRAMES);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
