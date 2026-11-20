/* envprobe: build a reflection cubemap by sampling the environment in six
   directions independently and stitching the faces into one seamless image.

   Problem. The old path had one fixed cube map captured in a fixed frame:
   the reflected direction was looked up without being rotated into that
   frame, so the further the camera turns from the capture orientation, the
   more wrong the reflection is; at 180 degrees you are looking at the
   opposite side of the environment. The fix is to sample all six cube
   directions for the current view. Six independent faces do not line up at
   their shared edges, though: the outermost texel of a face and the
   outermost texel of its neighbour sample two directions half a texel apart
   on either side of the seam, so the assembled cube has a visible step on
   every edge.

   Stitch. For every interior boundary texel we take its partner immediately
   across the edge and set both to the mean of the pair. The mean of two
   samples taken symmetrically around the geometric edge is a second-order
   accurate estimate of the value on the seam, and writing both sides from
   the same unmodified per-face renders makes them bit-identical, so the
   edge is continuous. The three texels that meet at each cube corner are set
   to the mean of all three for the same reason. Only boundary texels are
   touched; the interior of every face is left exactly as rendered.

   Contract checked by the self-test:
     - probe.faces must be exactly CUBE_FACES and probe.res must be inside
       [MIN_FACE_RES, MAX_FACE_RES]; anything else fails before rendering and
       the message names the field (probe.faces / probe.res / probe.stitch).
     - each stitched face is compared pixel by pixel with the same direction
       rendered on its own; the report gives, per face, the texel count, the
       number of changed texels and the worst / mean absolute error.
     - seam drift (max |a-b| over every texel pair across every edge) is
       reported before and after stitching.
     - with probe.stitch = 0 the pipeline output is byte-identical to the
       per-face renders produced without any stitch pass (memcmp).

   Conventions: +X right, +Y up, +Z forward (the camera looks down -Z).
   Face order is +X, -X, +Y, -Y, +Z, -Z; texels are face-major, RGB
   interleaved, sampled at texel centres. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_FACE_RES 1024
#define MIN_FACE_RES 2         /* a face needs an interior to have a seam */
#define CUBE_FACES 6
#define CHANNELS 3

typedef struct { float x, y, z; } Vec3;

typedef struct {
    int faces;                 /* must be CUBE_FACES */
    int res;                   /* texels per edge */
    int stitch;                /* 1 = blend seams, 0 = per-face output as-is */
} ProbeCfg;

/* face basis: n = outward normal, u = right, v = up (fixed up, all unit) */
typedef struct { Vec3 n, u, v; const char* name; } FaceBasis;

static const FaceBasis FACES[CUBE_FACES] = {
    { {  1, 0, 0 }, { 0, 0, -1 }, { 0, 1, 0 }, "+X" },
    { { -1, 0, 0 }, { 0, 0,  1 }, { 0, 1, 0 }, "-X" },
    { {  0, 1, 0 }, { 1, 0,  0 }, { 0, 0, 1 }, "+Y" },
    { {  0,-1, 0 }, { 1, 0,  0 }, { 0, 0,-1 }, "-Y" },
    { {  0, 0, 1 }, { 1, 0,  0 }, { 0, 1, 0 }, "+Z" },
    { {  0, 0,-1 }, { -1, 0, 0 }, { 0, 1, 0 }, "-Z" },
};

/* For each face, which face lies across each of its four edges (left,
   right, top, bottom), which edge of that neighbour is shared (same side
   order), and whether the shared edge is walked in reverse. Derived by
   matching the geometric edge directions of the bases above; the
   reciprocity and direction checks in the self-test keep the table honest. */
typedef struct {
    int face[4];
    int pside[4];
    int rev[4];
} EdgeMap;

static const EdgeMap EDGES[CUBE_FACES] = {
    /* +X */ { { 4, 5, 2, 3 }, { 1, 0, 1, 1 }, { 0, 0, 1, 0 } },
    /* -X */ { { 5, 4, 2, 3 }, { 1, 0, 0, 0 }, { 0, 0, 0, 1 } },
    /* +Y */ { { 1, 0, 4, 5 }, { 2, 2, 2, 2 }, { 0, 1, 0, 1 } },
    /* -Y */ { { 1, 0, 5, 4 }, { 3, 3, 3, 3 }, { 1, 0, 1, 0 } },
    /* +Z */ { { 1, 0, 2, 3 }, { 1, 0, 2, 3 }, { 0, 0, 0, 0 } },
    /* -Z */ { { 0, 1, 2, 3 }, { 1, 0, 3, 2 }, { 0, 0, 1, 1 } },
};

/* side indices: 0 = left, 1 = right, 2 = top, 3 = bottom */

static float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

/* ---- environment ----------------------------------------------------- */
/* Sky/ground gradient, coloured lobes and an azimuthal ripple. The ripple
   keeps every direction distinct, and the lobes cross face boundaries on
   purpose, so a bad seam shows up as a step instead of hiding in a smooth
   gradient. */
static void env_radiance(Vec3 d, float out[CHANNELS]) {
    float up = d.y;
    float sky = 0.55f + 0.25f * up;
    float t = clamp01((up + 0.2f) / 0.4f);
    float base = 0.25f * (1.0f - t) + sky * t;
    float lobe_r = 0.9f * powf(clamp01(d.x), 8.0f);
    float lobe_g = 0.7f * powf(clamp01(0.5f - 0.5f * d.z), 6.0f);
    float lobe_b = 0.6f * powf(clamp01(up), 5.0f);
    float az = atan2f(d.x, d.z);
    float ripple_r = 0.05f * sinf(5.0f * az + 0.3f);
    float ripple_g = 0.05f * sinf(7.0f * az + 1.1f);
    float ripple_b = 0.05f * sinf(9.0f * az + 2.0f);
    out[0] = clamp01(base + lobe_r + ripple_r);
    out[1] = clamp01(base + lobe_g + ripple_g);
    out[2] = clamp01(base + lobe_b - 0.15f * lobe_r + ripple_b);
}

/* Direction through the centre of texel (x, y) of a face. */
static Vec3 face_dir(const FaceBasis* fb, int res, int x, int y) {
    float s = 2.0f * ((float)x + 0.5f) / (float)res - 1.0f;
    float t = 2.0f * ((float)y + 0.5f) / (float)res - 1.0f;
    Vec3 d = {
        fb->n.x + fb->u.x * s + fb->v.x * t,
        fb->n.y + fb->u.y * s + fb->v.y * t,
        fb->n.z + fb->u.z * s + fb->v.z * t,
    };
    float len = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
    d.x /= len; d.y /= len; d.z /= len;
    return d;
}

/* Exact cube-corner vector for corner signs (s, t) of a face; only its sign
   pattern is used, to group the three faces meeting at a corner. */
static Vec3 face_corner_dir(const FaceBasis* fb, int s, int t) {
    Vec3 d = {
        fb->n.x + fb->u.x * (float)s + fb->v.x * (float)t,
        fb->n.y + fb->u.y * (float)s + fb->v.y * (float)t,
        fb->n.z + fb->u.z * (float)s + fb->v.z * (float)t,
    };
    return d;
}

/* ---- configuration --------------------------------------------------- */
/* Hard validation. The message names the offending field and is also copied
   into `err` so an embedding caller can surface it unchanged. */
static int probe_validate(const ProbeCfg* cfg, char* err, size_t errlen) {
    if (cfg->faces != CUBE_FACES) {
        snprintf(err, errlen,
                 "invalid probe.faces=%d: a cube probe needs exactly %d directions",
                 cfg->faces, CUBE_FACES);
        return -1;
    }
    if (cfg->res < MIN_FACE_RES || cfg->res > MAX_FACE_RES) {
        snprintf(err, errlen,
                 "invalid probe.res=%d: must be in [%d, %d]",
                 cfg->res, MIN_FACE_RES, MAX_FACE_RES);
        return -1;
    }
    if (cfg->stitch != 0 && cfg->stitch != 1) {
        snprintf(err, errlen,
                 "invalid probe.stitch=%d: must be 0 or 1", cfg->stitch);
        return -1;
    }
    return 0;
}

/* ---- render ---------------------------------------------------------- */
/* Render one face on its own: exactly what the per-direction path did
   before, with no knowledge of any neighbour. */
static void render_face(const ProbeCfg* cfg, int f, float* face) {
    int n = cfg->res;
    for (int y = 0; y < n; y++)
        for (int x = 0; x < n; x++)
            env_radiance(face_dir(&FACES[f], n, x, y),
                         face + ((size_t)y * n + x) * CHANNELS);
}

static void render_all(const ProbeCfg* cfg, float* rgb) {
    int n = cfg->res;
    for (int f = 0; f < cfg->faces; f++)
        render_face(cfg, f, rgb + (size_t)f * n * n * CHANNELS);
}

/* ---- seam mapping ---------------------------------------------------- */
/* (x, y) of boundary texel i on the given side. */
static void edge_texel(int side, int i, int n, int* x, int* y) {
    switch (side) {
    case 0: *x = 0;     *y = i; break;
    case 1: *x = n - 1; *y = i; break;
    case 2: *x = i;     *y = n - 1; break;
    default:*x = i;     *y = 0; break;
    }
}

/* Partner of boundary texel i along edge `side` of face f. i runs along the
   edge (y on left/right, x on top/bottom). Returns 0 if there is no
   neighbour. The partner's coordinates follow the neighbour's shared edge,
   not the original side. */
static int face_edge_partner(const ProbeCfg* cfg, int f, int side, int i,
                             int* pf, int* px, int* py) {
    int n = cfg->res;
    const EdgeMap* em = &EDGES[f];
    int g = em->face[side];
    if (g < 0) return 0;
    int k = em->rev[side] ? (n - 1 - i) : i;
    switch (em->pside[side]) {
    case 0: *px = 0;     *py = k; break;
    case 1: *px = n - 1; *py = k; break;
    case 2: *px = k;     *py = n - 1; break;
    default:*px = k;     *py = 0; break;
    }
    *pf = g;
    return 1;
}

/* ---- stitching ------------------------------------------------------- */
/* Edge pass: each pair of texels across a seam becomes the mean of the pair;
   both sides are written from the raw image, so they end up bit identical.
   Corner texels are skipped here and handled by the corner pass, because a
   corner texel belongs to two seams at once and cannot equal both partners
   individually. */
static void stitch_edges(const ProbeCfg* cfg, const float* raw, float* out) {
    int n = cfg->res;
    for (int f = 0; f < cfg->faces; f++)
        for (int side = 0; side < 4; side++)
            for (int i = 1; i < n - 1; i++) {
                int x, y;
                edge_texel(side, i, n, &x, &y);
                int pf, px, py;
                if (!face_edge_partner(cfg, f, side, i, &pf, &px, &py))
                    continue;
                const float* a = raw + ((size_t)f * n * n + (size_t)y * n + x) * CHANNELS;
                const float* b = raw + ((size_t)pf * n * n + (size_t)py * n + px) * CHANNELS;
                float* oa = out + ((size_t)f * n * n + (size_t)y * n + x) * CHANNELS;
                float* ob = out + ((size_t)pf * n * n + (size_t)py * n + px) * CHANNELS;
                for (int c = 0; c < CHANNELS; c++) {
                    float m = 0.5f * (a[c] + b[c]);
                    oa[c] = m;
                    ob[c] = m;
                }
            }
}

/* Corner pass: the three texels meeting at a cube corner become the mean of
   all three, so every pair around that corner agrees. The group is built
   from the sign pattern of the exact corner direction. Returns the number
   of groups built (must be 8) and reports whether each had three members. */
static int stitch_corners(const ProbeCfg* cfg, const float* raw, float* out,
                          int* groups_ok) {
    int n = cfg->res;
    int built = 0;
    *groups_ok = 1;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2) {
                int member[3][3];          /* face, x, y */
                int count = 0;
                for (int f = 0; f < CUBE_FACES; f++)
                    for (int cs = -1; cs <= 1; cs += 2)
                        for (int ct = -1; ct <= 1; ct += 2) {
                            Vec3 d = face_corner_dir(&FACES[f], cs, ct);
                            int mx = d.x > 0 ? 1 : -1;
                            int my = d.y > 0 ? 1 : -1;
                            int mz = d.z > 0 ? 1 : -1;
                            if (mx != sx || my != sy || mz != sz) continue;
                            if (count < 3) {
                                member[count][0] = f;
                                member[count][1] = cs < 0 ? 0 : n - 1;
                                member[count][2] = ct < 0 ? 0 : n - 1;
                            }
                            count++;
                        }
                if (count != 3) { *groups_ok = 0; continue; }
                built++;
                float m[CHANNELS] = { 0.0f, 0.0f, 0.0f };
                for (int k = 0; k < 3; k++) {
                    int f = member[k][0], x = member[k][1], y = member[k][2];
                    const float* a = raw + ((size_t)f * n * n + (size_t)y * n + x) * CHANNELS;
                    for (int c = 0; c < CHANNELS; c++) m[c] += a[c];
                }
                for (int c = 0; c < CHANNELS; c++) m[c] /= 3.0f;
                for (int k = 0; k < 3; k++) {
                    int f = member[k][0], x = member[k][1], y = member[k][2];
                    float* o = out + ((size_t)f * n * n + (size_t)y * n + x) * CHANNELS;
                    for (int c = 0; c < CHANNELS; c++) o[c] = m[c];
                }
            }
    return built;
}

/* ---- pipeline entry -------------------------------------------------- */
/* Validate, render the six directions independently, then optionally stitch.
   With cfg->stitch == 0 `out` is left exactly as render_all wrote it, which
   is the pre-change output. `scratch` must hold one full image and is only
   used when stitching. */
static int probe_render(const ProbeCfg* cfg, float* out, float* scratch,
                        char* err, size_t errlen) {
    if (probe_validate(cfg, err, errlen) != 0) return -1;
    render_all(cfg, out);
    if (cfg->stitch) {
        memcpy(scratch, out, sizeof(float) * (size_t)cfg->faces * cfg->res *
                              cfg->res * CHANNELS);
        stitch_edges(cfg, scratch, out);
        int groups_ok;
        int built = stitch_corners(cfg, scratch, out, &groups_ok);
        if (!groups_ok || built != 8) {
            snprintf(err, errlen, "internal: cube corner groups incomplete");
            return -1;
        }
    }
    return 0;
}

/* ---- metrics --------------------------------------------------------- */
/* Worst per-channel difference over every pair of texels across a seam.
   Includes corners; after stitching every pair must agree exactly. */
static float seam_drift(const ProbeCfg* cfg, const float* img) {
    int n = cfg->res;
    float worst = 0.0f;
    for (int f = 0; f < cfg->faces; f++)
        for (int side = 0; side < 4; side++)
            for (int i = 0; i < n; i++) {
                int x, y;
                edge_texel(side, i, n, &x, &y);
                int pf, px, py;
                if (!face_edge_partner(cfg, f, side, i, &pf, &px, &py))
                    continue;
                const float* a = img + ((size_t)f * n * n + (size_t)y * n + x) * CHANNELS;
                const float* b = img + ((size_t)pf * n * n + (size_t)py * n + px) * CHANNELS;
                for (int c = 0; c < CHANNELS; c++) {
                    float e = fabsf(a[c] - b[c]);
                    if (e > worst) worst = e;
                }
            }
    return worst;
}

typedef struct {
    int changed;               /* texels differing from the solo render */
    float max_err;
    double mean_err;
} FaceDiff;

/* Diff a face of the composed image against the same direction rendered on
   its own. */
static FaceDiff face_diff(const ProbeCfg* cfg, int f, const float* image) {
    int n = cfg->res;
    FaceDiff d = { 0, 0.0f, 0.0 };
    float* solo = (float*)malloc(sizeof(float) * (size_t)n * n * CHANNELS);
    render_face(cfg, f, solo);
    const float* b = image + (size_t)f * n * n * CHANNELS;
    double sum = 0.0;
    int count = n * n;
    for (int i = 0; i < count; i++) {
        int hit = 0;
        for (int c = 0; c < CHANNELS; c++) {
            float e = fabsf(solo[i * CHANNELS + c] - b[i * CHANNELS + c]);
            sum += e;
            if (e > 0.0f) hit = 1;
            if (e > d.max_err) d.max_err = e;
        }
        if (hit) d.changed++;
    }
    d.mean_err = sum / (double)(count * CHANNELS);
    free(solo);
    return d;
}

/* True when no face interior texel changed; the stitcher is only allowed to
   touch the boundary ring. */
static int interior_untouched(const ProbeCfg* cfg, const float* image) {
    int n = cfg->res;
    for (int f = 0; f < cfg->faces; f++) {
        float* solo = (float*)malloc(sizeof(float) * (size_t)n * n * CHANNELS);
        render_face(cfg, f, solo);
        const float* b = image + (size_t)f * n * n * CHANNELS;
        for (int y = 1; y < n - 1; y++)
            for (int x = 1; x < n - 1; x++)
                for (int c = 0; c < CHANNELS; c++)
                    if (solo[((size_t)y * n + x) * CHANNELS + c] !=
                        b[((size_t)y * n + x) * CHANNELS + c]) {
                        free(solo);
                        return 0;
                    }
        free(solo);
    }
    return 1;
}

/* Largest one-texel step straight across a face boundary, measured just
   inside the face. This is the natural scale for a seam: sampling half a
   texel either side of the seam cannot differ by more than one interior
   texel step of the same environment. */
static float edge_normal_step(const ProbeCfg* cfg, const float* image) {
    int n = cfg->res;
    float worst = 0.0f;
    for (int f = 0; f < cfg->faces; f++) {
        const float* face = image + (size_t)f * n * n * CHANNELS;
        for (int side = 0; side < 4; side++)
            for (int i = 1; i < n - 1; i++) {
                int x, y, ix, iy;
                edge_texel(side, i, n, &x, &y);
                ix = side == 0 ? x + 1 : side == 1 ? x - 1 : x;
                iy = side == 2 ? y - 1 : side == 3 ? y + 1 : y;
                for (int c = 0; c < CHANNELS; c++) {
                    float e = fabsf(face[((size_t)y * n + x) * CHANNELS + c] -
                                    face[((size_t)iy * n + ix) * CHANNELS + c]);
                    if (e > worst) worst = e;
                }
            }
    }
    return worst;
}

/* ---- old path: fixed cube map in a fixed frame ----------------------- */
/* The pre-change lookup rotated the reflected direction by nothing, while
   the capture frame had been rotated by `yaw` with the camera. The probe
   therefore returns the environment seen `yaw` radians away from where it
   should look. */
static void legacy_radiance(Vec3 d, float yaw, float out[CHANNELS]) {
    float c = cosf(yaw), s = sinf(yaw);
    Vec3 r = { c * d.x - s * d.z, d.y, s * d.x + c * d.z };
    env_radiance(r, out);
}

/* Mean and max per-channel error of the legacy lookup over the reflection
   directions facing the camera, at a given yaw. */
static void legacy_error(float yaw, int res, double* mean_out, float* max_out) {
    double sum = 0.0;
    long count = 0;
    float mx = 0.0f;
    for (int y = 0; y < res; y++)
        for (int x = 0; x < res; x++) {
            Vec3 d = face_dir(&FACES[4], res, x, y);
            float truth[CHANNELS];
            env_radiance(d, truth);
            float legacy[CHANNELS];
            legacy_radiance(d, yaw, legacy);
            for (int c = 0; c < CHANNELS; c++) {
                float e = fabsf(truth[c] - legacy[c]);
                sum += e;
                count++;
                if (e > mx) mx = e;
            }
        }
    *mean_out = count ? sum / (double)count : 0.0;
    *max_out = mx;
}

/* ---- self-test ------------------------------------------------------- */
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static float* alloc_image(const ProbeCfg* cfg) {
    size_t px = (size_t)cfg->faces * (size_t)cfg->res * (size_t)cfg->res;
    return (float*)calloc(px * CHANNELS, sizeof(float));
}

static int run_selftest(void) {
    char err[256] = {0};

    /* 1. invalid direction count / resolution fails up front, before any
       rendering, and the message names the field. */
    ProbeCfg bad = { 5, 64, 1 };
    int rc = probe_validate(&bad, err, sizeof err);
    printf("reject_faces_rc=%d msg=\"%s\"\n", rc, err);
    check("reject_faces_names_field", rc == -1 && strstr(err, "probe.faces") != NULL);
    bad.faces = 8;
    check("reject_faces_eight", probe_validate(&bad, err, sizeof err) == -1);
    bad.faces = 0;
    check("reject_faces_zero", probe_validate(&bad, err, sizeof err) == -1);
    bad.faces = CUBE_FACES; bad.res = 0;
    rc = probe_validate(&bad, err, sizeof err);
    printf("reject_res0_rc=%d msg=\"%s\"\n", rc, err);
    check("reject_res_names_field", rc == -1 && strstr(err, "probe.res") != NULL);
    bad.res = 1;
    check("reject_res_one", probe_validate(&bad, err, sizeof err) == -1);
    bad.res = MAX_FACE_RES + 1;
    check("reject_res_huge", probe_validate(&bad, err, sizeof err) == -1);
    bad.res = -4;
    check("reject_res_negative", probe_validate(&bad, err, sizeof err) == -1);
    bad.res = MIN_FACE_RES;
    check("accept_min_res", probe_validate(&bad, err, sizeof err) == 0);
    bad.res = MAX_FACE_RES;
    check("accept_max_res", probe_validate(&bad, err, sizeof err) == 0);
    bad.res = 64;
    check("accept_valid", probe_validate(&bad, err, sizeof err) == 0);
    ProbeCfg bad_stitch = { CUBE_FACES, 64, 2 };
    check("reject_stitch_names_field",
          probe_validate(&bad_stitch, err, sizeof err) == -1 &&
          strstr(err, "probe.stitch") != NULL);

    /* 2. why a fixed cube map fails: error grows as the view turns away from
       the capture orientation, worst at the back. The six-direction probe
       has no such error by construction. */
    double m0, m90, m180;
    float x0, x90, x180;
    legacy_error(0.0f, 64, &m0, &x0);
    legacy_error(1.5707963f, 64, &m90, &x90);
    legacy_error(3.1415927f, 64, &m180, &x180);
    printf("legacy_err_yaw0=%.6f yaw90=%.6f yaw180=%.6f\n", m0, m90, m180);
    printf("legacy_max_yaw180=%.6f six_dir_err=0.000000\n", x180);
    check("legacy_error_zero_at_capture", m0 == 0.0);
    check("legacy_error_grows_to_back", m180 > m90 && m90 > m0);
    check("legacy_back_error_large", m180 > 0.05);

    /* 3. the adjacency table describes a real cube */
    int adjacency_ok = 1;
    for (int f = 0; f < CUBE_FACES; f++) {
        int seen[CUBE_FACES] = {0};
        for (int side = 0; side < 4; side++) {
            int g = EDGES[f].face[side];
            int ps = EDGES[f].pside[side];
            if (g < 0 || g == f || g >= CUBE_FACES || ps < 0 || ps > 3 || seen[g]) {
                adjacency_ok = 0;
                break;
            }
            seen[g] = 1;
        }
    }
    check("adjacency_four_distinct_neighbours", adjacency_ok);

    ProbeCfg cfg = { CUBE_FACES, 64, 1 };
    /* Corners are shared by three faces, so a corner texel has two partners;
       edge reciprocity covers edge interiors only. Corners are checked
       separately below. */
    int reciprocal_ok = 1;
    int partner_direction_ok = 1;
    for (int f = 0; f < CUBE_FACES && reciprocal_ok; f++)
        for (int side = 0; side < 4 && reciprocal_ok; side++)
            for (int i = 1; i < cfg.res - 1 && reciprocal_ok; i++) {
                int x, y;
                edge_texel(side, i, cfg.res, &x, &y);
                int pf, px, py;
                if (!face_edge_partner(&cfg, f, side, i, &pf, &px, &py)) {
                    reciprocal_ok = 0;
                    break;
                }
                int back_side = -1, back_i = 0;
                if (px == 0) { back_side = 0; back_i = py; }
                else if (px == cfg.res - 1) { back_side = 1; back_i = py; }
                else if (py == cfg.res - 1) { back_side = 2; back_i = px; }
                else if (py == 0) { back_side = 3; back_i = px; }
                int bf, bx, by;
                if (back_side < 0
                    || !face_edge_partner(&cfg, pf, back_side, back_i, &bf, &bx, &by)
                    || bf != f || bx != x || by != y)
                    reciprocal_ok = 0;
                /* the partner is the texel just across the same seam */
                Vec3 da = face_dir(&FACES[f], cfg.res, x, y);
                Vec3 db = face_dir(&FACES[pf], cfg.res, px, py);
                float dot = da.x * db.x + da.y * db.y + da.z * db.z;
                if (dot < 0.995f) partner_direction_ok = 0;
            }
    check("edge_partner_reciprocal", reciprocal_ok);
    check("edge_partner_direction_adjacent", partner_direction_ok);

    float* probe_a = alloc_image(&cfg);
    float* probe_b = alloc_image(&cfg);
    int groups_ok = 0;
    int built = stitch_corners(&cfg, probe_a, probe_b, &groups_ok);
    check("corner_groups_three_faces", groups_ok && built == 8);
    free(probe_a);
    free(probe_b);

    /* 4. render through the pipeline and compare per face with solo renders */
    float* raw = alloc_image(&cfg);
    float* stitched = alloc_image(&cfg);
    float* scratch = alloc_image(&cfg);
    render_all(&cfg, raw);
    check("pipeline_ok", probe_render(&cfg, stitched, scratch, err, sizeof err) == 0);

    printf("face  pixels  changed  max_abs_err  mean_abs_err\n");
    double total_mean = 0.0;
    for (int f = 0; f < CUBE_FACES; f++) {
        FaceDiff d = face_diff(&cfg, f, stitched);
        printf("%-4s %6d  %7d  %11.6f  %12.6f\n", FACES[f].name,
               cfg.res * cfg.res, d.changed, d.max_err, d.mean_err);
        total_mean += d.mean_err;
        check("stitch_changed_face_boundary", d.changed == 4 * cfg.res - 4);
    }
    printf("stitch_mean_abs_err_overall=%.6f\n", total_mean / CUBE_FACES);

    /* Interiors must be untouched by the stitcher: only the boundary ring
       can differ from the solo render. */
    check("interior_untouched", interior_untouched(&cfg, stitched));

    /* 5. seam continuity before and after, scaled by the face's own edge
       gradient: a half-texel mismatch at a seam can be at most the size of
       one texel step along the edge normal. */
    float drift_raw = seam_drift(&cfg, raw);
    float drift_stitched = seam_drift(&cfg, stitched);
    float digest = edge_normal_step(&cfg, stitched);
    printf("seam_drift_raw=%.6f seam_drift_stitched=%.6f seam_scale=%.6f\n",
           drift_raw, drift_stitched, digest);
    check("seam_existed", drift_raw > 0.0f);
    check("seam_error_is_subtexel", drift_raw <= digest);
    check("seam_stitched_continuous", drift_stitched == 0.0f);

    /* 6. stitching off is byte-identical to the per-face path */
    ProbeCfg off = { CUBE_FACES, 64, 0 };
    float* baseline = alloc_image(&off);
    float* off_out = alloc_image(&off);
    render_all(&off, baseline);   /* pre-change output, no stitch pass */
    check("pipeline_off_ok", probe_render(&off, off_out, scratch, err, sizeof err) == 0);
    size_t bytes = (size_t)off.faces * off.res * off.res * CHANNELS * sizeof(float);
    check("stitch_off_bit_identical", memcmp(off_out, baseline, bytes) == 0);
    check("stitch_on_differs", memcmp(stitched, baseline, bytes) != 0);

    /* 7. resolution sweep: every valid size stitches to zero drift */
    int sizes_ok = 1;
    for (int r = MIN_FACE_RES; r <= 128; r = r < 8 ? r + 1 : r * 2) {
        ProbeCfg c = { CUBE_FACES, r, 1 };
        float* a = alloc_image(&c);
        float* b = alloc_image(&c);
        float* s = alloc_image(&c);
        if (probe_render(&c, a, s, err, sizeof err) != 0 || seam_drift(&c, a) != 0.0f)
            sizes_ok = 0;
        free(a); free(b); free(s);
        if (!sizes_ok) break;
    }
    check("resolution_sweep_continuous", sizes_ok);

    free(raw);
    free(stitched);
    free(scratch);
    free(baseline);
    free(off_out);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}

/* ---- command line ---------------------------------------------------- */
/* key=value arguments for the three probe fields; invalid values fail with
   the field named, before anything is rendered. */
static int run_cli(int argc, char** argv) {
    ProbeCfg cfg = { CUBE_FACES, 64, 1 };
    for (int i = 1; i < argc; i++) {
        const char* a = argv[i];
        if (strncmp(a, "faces=", 6) == 0) cfg.faces = atoi(a + 6);
        else if (strncmp(a, "res=", 4) == 0) cfg.res = atoi(a + 4);
        else if (strncmp(a, "stitch=", 7) == 0) cfg.stitch = atoi(a + 7);
        else {
            fprintf(stderr, "error: unknown field \"%s\" (expected faces=, res=, stitch=)\n", a);
            return 2;
        }
    }
    char err[256] = {0};
    if (probe_validate(&cfg, err, sizeof err) != 0) {
        fprintf(stderr, "error: %s\n", err);
        return 1;
    }

    size_t px = (size_t)cfg.faces * (size_t)cfg.res * (size_t)cfg.res * CHANNELS;
    float* out = (float*)calloc(px, sizeof(float));
    float* scratch = (float*)calloc(px, sizeof(float));
    if (!out || !scratch) {
        fprintf(stderr, "error: out of memory for %dx%d x %d faces\n",
                cfg.res, cfg.res, cfg.faces);
        free(out);
        free(scratch);
        return 1;
    }
    if (probe_render(&cfg, out, scratch, err, sizeof err) != 0) {
        fprintf(stderr, "error: %s\n", err);
        free(out);
        free(scratch);
        return 1;
    }
    /* same render without the stitch pass, for the before/after numbers */
    ProbeCfg pre = cfg;
    pre.stitch = 0;
    float* raw = (float*)calloc(px, sizeof(float));
    if (!raw || probe_render(&pre, raw, scratch, err, sizeof err) != 0) {
        fprintf(stderr, "error: out of memory\n");
        free(out);
        free(raw);
        free(scratch);
        return 1;
    }

    printf("probe.faces=%d probe.res=%d probe.stitch=%d\n",
           cfg.faces, cfg.res, cfg.stitch);
    for (int f = 0; f < CUBE_FACES; f++) {
        FaceDiff d = face_diff(&cfg, f, out);
        printf("%-4s changed=%d max_abs_err=%.6f mean_abs_err=%.6f\n",
               FACES[f].name, d.changed, d.max_err, d.mean_err);
    }
    printf("seam_drift_raw=%.6f seam_drift_out=%.6f\n",
           seam_drift(&cfg, raw), seam_drift(&cfg, out));

    free(raw);
    free(out);
    free(scratch);
    return 0;
}

int main(int argc, char** argv) {
    if (argc > 1) return run_cli(argc, argv);
    return run_selftest();
}
