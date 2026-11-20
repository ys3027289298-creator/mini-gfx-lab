/* terrain: chunked heightfield with screen-space-error LOD selection.

   The heightfield is cut into square chunks.  For each chunk a quadtree of
   vertex grids is implied; level L draws the chunk with 2^L-sample cells.
   The chunk caches the geometric error of every level: the largest vertical
   distance between the real heightfield and the two triangles actually drawn
   for a cell at that level.  Because the number is measured against the drawn
   triangles, it bounds the surface error including the cell diagonal.

   Every frame:
     - frustum-cull each chunk (yaw-only camera over a heightfield)
     - pick the coarsest level whose projected error is under one pixel
     - refine levels so drawn neighbours differ by at most one level
     - on a border with a finer neighbour, split every border cell edge once
       and fan the cell from its centre, so both sides emit the same vertices
       along the shared edge (no T-junctions, no cracks, no skirts)

   The self-test renders the same view at full precision and with LOD and
   compares the skyline against the full-precision image. */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdint.h>

#define HF_W 513
#define HF_H 513
#define SAMPLE_M 4.0f                     /* world metres between height samples */
#define CHUNK 64                          /* samples per chunk edge */
#define NCHUNK ((HF_W - 1) / CHUNK)       /* 8 x 8 chunks over 2 km */
#define MAX_LEVEL 6                       /* level L cell = 2^L samples */
#define NLEV (MAX_LEVEL + 1)
#define SCREEN_W 960
#define SCREEN_H 540
#define NEAR_Z 0.5f
#define FAR_Z 3000.0f
#define TOL_PX 1.0f                       /* LOD error budget, one pixel */
#define SUPER 4                           /* vertical supersample for the skyline check */

typedef struct {
    float h[HF_W][HF_H];
} Heightfield;

typedef struct {
    float x0, z0, x1, z1;                 /* footprint in world metres */
    float err[NLEV];                      /* geometric error if drawn at level L */
} Chunk;

typedef struct {
    float px, py, pz;
    float yaw;                            /* radians; 0 looks along +z */
    float fov_y;
} Camera;

typedef struct {
    int h;                                /* rows; SCREEN_H for the normal view */
    float vscale;                         /* h / SCREEN_H, vertical supersample factor */
    unsigned char* cov;
    float* depth;
    int top[SCREEN_W];                    /* topmost covered row per column, -1 = sky */
} Image;

typedef struct {
    int visible, cells, tris;
} RenderInfo;

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* ------------------------------------------------------------- heightfield --- */

static unsigned int hash2(int x, int y, unsigned int seed) {
    unsigned int h = (unsigned int)x * 374761393u + (unsigned int)y * 668265263u + seed * 2246822519u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}

static float value_noise(float x, float y, unsigned int seed) {
    int xi = (int)floorf(x), yi = (int)floorf(y);
    float fx = x - (float)xi, fy = y - (float)yi;
    float ux = fx * fx * (3.0f - 2.0f * fx);
    float uy = fy * fy * (3.0f - 2.0f * fy);
    float a = (float)(hash2(xi, yi, seed) & 0xffff) / 65535.0f;
    float b = (float)(hash2(xi + 1, yi, seed) & 0xffff) / 65535.0f;
    float c = (float)(hash2(xi, yi + 1, seed) & 0xffff) / 65535.0f;
    float d = (float)(hash2(xi + 1, yi + 1, seed) & 0xffff) / 65535.0f;
    float ab = a + (b - a) * ux, cd = c + (d - c) * ux;
    return ab + (cd - ab) * uy;
}

static float terrain_height(int x, int y) {
    float fx = (float)x, fy = (float)y;
    float v = 0.0f;
    v += 34.0f * (value_noise(fx / 97.0f, fy / 97.0f, 11u) * 2.0f - 1.0f);
    v += 12.0f * (value_noise(fx / 31.0f, fy / 31.0f, 23u) * 2.0f - 1.0f);
    v += 3.5f * (value_noise(fx / 11.0f, fy / 11.0f, 37u) * 2.0f - 1.0f);
    v += 0.9f * (value_noise(fx / 4.3f, fy / 4.3f, 51u) * 2.0f - 1.0f);
    v += 0.22f * (value_noise(fx / 1.9f, fy / 1.9f, 67u) * 2.0f - 1.0f);
    return v;
}

static void heightfield_init(Heightfield* hf) {
    for (int z = 0; z < HF_H; z++)
        for (int x = 0; x < HF_W; x++)
            hf->h[z][x] = terrain_height(x, z);
}

/* -------------------------------------------------------------- chunk LOD --- */

/* Worst vertical error inside a grid cell of `step` samples.  The check uses
   the exact two triangles the renderer emits, so the result bounds the drawn
   surface, diagonal included. */
static float node_error(const Heightfield* hf, int x0, int z0, int step) {
    float worst = 0.0f;
    for (int gz = 0; gz < CHUNK; gz += step) {
        for (int gx = 0; gx < CHUNK; gx += step) {
            int ax = x0 + gx, az = z0 + gz;
            float h00 = hf->h[az][ax];
            float h10 = hf->h[az][ax + step];
            float h01 = hf->h[az + step][ax];
            float h11 = hf->h[az + step][ax + step];
            for (int sz = 0; sz <= step; sz++) {
                for (int sx = 0; sx <= step; sx++) {
                    if ((sx == 0 || sx == step) && (sz == 0 || sz == step)) continue;
                    float u = (float)sx / (float)step;
                    float v = (float)sz / (float)step;
                    float approx = (u >= v)
                        ? h00 * (1.0f - u) + h10 * (u - v) + h11 * v
                        : h00 * (1.0f - v) + h01 * (v - u) + h11 * u;
                    float d = fabsf(hf->h[az + sz][ax + sx] - approx);
                    if (d > worst) worst = d;
                }
            }
        }
    }
    return worst;
}

static void chunk_init(Chunk* chunks, const Heightfield* hf) {
    for (int cz = 0; cz < NCHUNK; cz++) {
        for (int cx = 0; cx < NCHUNK; cx++) {
            Chunk* c = &chunks[cz * NCHUNK + cx];
            int x0 = cx * CHUNK, z0 = cz * CHUNK;
            c->x0 = (float)x0 * SAMPLE_M;
            c->z0 = (float)z0 * SAMPLE_M;
            c->x1 = (float)(x0 + CHUNK) * SAMPLE_M;
            c->z1 = (float)(z0 + CHUNK) * SAMPLE_M;
            c->err[0] = 0.0f;
            for (int L = 1; L <= MAX_LEVEL; L++)
                c->err[L] = node_error(hf, x0, z0, 1 << L);
            /* A coarser grid is not guaranteed to be worse at every node, but
               selection needs a monotone error: keep the running maximum. */
            for (int L = 1; L <= MAX_LEVEL; L++)
                if (c->err[L] < c->err[L - 1]) c->err[L] = c->err[L - 1];
        }
    }
}

static float chunk_distance(const Chunk* c, const Camera* cam) {
    float dx = 0.0f, dz = 0.0f;
    if (cam->px < c->x0) dx = c->x0 - cam->px;
    else if (cam->px > c->x1) dx = cam->px - c->x1;
    if (cam->pz < c->z0) dz = c->z0 - cam->pz;
    else if (cam->pz > c->z1) dz = cam->pz - c->z1;
    float d = sqrtf(dx * dx + dz * dz);
    return d < NEAR_Z ? NEAR_Z : d;
}

static float focal_length(const Camera* cam) {
    return (float)SCREEN_H * 0.5f / tanf(cam->fov_y * 0.5f);
}

/* Height error `geom_error` at `distance` projects to this many pixels. */
static float projected_error(float geom_error, float distance, const Camera* cam) {
    return geom_error * focal_length(cam) / distance;
}

static float screen_error(const Chunk* c, int level, const Camera* cam) {
    return projected_error(c->err[level], chunk_distance(c, cam), cam);
}

/* coarsest level whose error stays inside the pixel budget */
static int select_level(const Chunk* c, const Camera* cam, float tol_px) {
    for (int L = MAX_LEVEL; L >= 1; L--)
        if (screen_error(c, L, cam) <= tol_px) return L;
    return 0;
}

/* Yaw-only frustum test on the convex chunk footprint: exact half-plane test
   on its four corners, vertical extent deliberately left conservative. */
static int chunk_visible(const Chunk* c, const Camera* cam) {
    float s = sinf(cam->yaw), co = cosf(cam->yaw);
    float tx = (float)SCREEN_W * 0.5f / focal_length(cam);
    int all_near = 1, all_far = 1, all_left = 1, all_right = 1;
    for (int i = 0; i < 4; i++) {
        float wx = (i & 1) ? c->x1 : c->x0;
        float wz = (i & 2) ? c->z1 : c->z0;
        float rx = wx - cam->px, rz = wz - cam->pz;
        float vx = rx * co - rz * s;
        float vz = rx * s + rz * co;
        if (vz > -NEAR_Z) all_near = 0;
        if (vz < FAR_Z) all_far = 0;
        if (!(vz > 0.0f && vx < -vz * tx)) all_left = 0;
        if (!(vz > 0.0f && vx > vz * tx)) all_right = 0;
    }
    return !(all_near || all_far || all_left || all_right);
}

static const int NX[4] = { -1, 1, 0, 0 };
static const int NZ[4] = { 0, 0, -1, 1 };

static int neighbor_index(int cx, int cz, int dir) {
    int nx = cx + NX[dir], nz = cz + NZ[dir];
    if (nx < 0 || nx >= NCHUNK || nz < 0 || nz >= NCHUNK) return -1;
    return nz * NCHUNK + nx;
}

/* Clamp levels so drawn neighbours differ by at most one level.  Only drawn
   neighbours matter: an edge shared with a culled chunk is not rendered and
   cannot crack. */
static int balance_levels(const int* active, int* levels) {
    int refined = 0;
    for (int pass = 0; pass < MAX_LEVEL; pass++) {
        int changed = 0;
        for (int cz = 0; cz < NCHUNK; cz++) {
            for (int cx = 0; cx < NCHUNK; cx++) {
                int i = cz * NCHUNK + cx;
                if (!active[i]) continue;
                for (int d = 0; d < 4; d++) {
                    int j = neighbor_index(cx, cz, d);
                    if (j < 0 || !active[j]) continue;
                    if (levels[i] > levels[j] + 1) {
                        levels[i] = levels[j] + 1;
                        refined++;
                        changed = 1;
                    }
                }
            }
        }
        if (!changed) break;
    }
    return refined;
}

/* ----------------------------------------------------------------- raster --- */

typedef struct { float x, y, z; } CamV;

static CamV to_cam(const Camera* cam, float wx, float wy, float wz) {
    float rx = wx - cam->px, rz = wz - cam->pz;
    float s = sinf(cam->yaw), co = cosf(cam->yaw);
    CamV v;
    v.x = rx * co - rz * s;
    v.y = wy - cam->py;
    v.z = rx * s + rz * co;
    return v;
}

static void raster(float focal, Image* img, CamV a, CamV b, CamV c) {
    float sx[3], sy[3], iz[3];
    const CamV* v[3] = { &a, &b, &c };
    for (int k = 0; k < 3; k++) {
        sx[k] = (float)SCREEN_W * 0.5f + focal * v[k]->x / v[k]->z;
        sy[k] = (float)img->h * 0.5f - focal * img->vscale * v[k]->y / v[k]->z;
        iz[k] = 1.0f / v[k]->z;
    }
    int minx = (int)floorf(fminf(sx[0], fminf(sx[1], sx[2])));
    int maxx = (int)ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2])));
    int miny = (int)floorf(fminf(sy[0], fminf(sy[1], sy[2])));
    int maxy = (int)ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2])));
    if (maxx < 0 || maxy < 0 || minx >= SCREEN_W || miny >= img->h) return;
    if (minx < 0) minx = 0;
    if (miny < 0) miny = 0;
    if (maxx > SCREEN_W - 1) maxx = SCREEN_W - 1;
    if (maxy > img->h - 1) maxy = img->h - 1;

    float area = (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
    if (fabsf(area) < 1e-9f) return;
    float inv = 1.0f / area;
    for (int y = miny; y <= maxy; y++) {
        float py = (float)y + 0.5f;
        for (int x = minx; x <= maxx; x++) {
            float px = (float)x + 0.5f;
            float w0 = ((sx[1] - px) * (sy[2] - py) - (sx[2] - px) * (sy[1] - py)) * inv;
            float w1 = ((sx[2] - px) * (sy[0] - py) - (sx[0] - px) * (sy[2] - py)) * inv;
            float w2 = 1.0f - w0 - w1;
            if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) continue;
            float z = 1.0f / (w0 * iz[0] + w1 * iz[1] + w2 * iz[2]);
            int o = y * SCREEN_W + x;
            if (z < img->depth[o]) {
                img->depth[o] = z;
                img->cov[o] = 1;
            }
        }
    }
}

/* clip against the near plane, then fan the resulting polygon */
static void draw_tri(float focal, Image* img, CamV a, CamV b, CamV c) {
    CamV in[3] = { a, b, c };
    CamV poly[4];
    int n = 0;
    for (int i = 0; i < 3; i++) {
        CamV p = in[i], q = in[(i + 1) % 3];
        float dp = p.z - NEAR_Z, dq = q.z - NEAR_Z;
        if (dp >= 0.0f) poly[n++] = p;
        if ((dp >= 0.0f) != (dq >= 0.0f)) {
            float t = dp / (dp - dq);
            CamV r;
            r.x = p.x + (q.x - p.x) * t;
            r.y = p.y + (q.y - p.y) * t;
            r.z = NEAR_Z;
            poly[n++] = r;
        }
    }
    for (int i = 1; i + 1 < n; i++) raster(focal, img, poly[0], poly[i], poly[i + 1]);
}

static void image_alloc(Image* img, int h) {
    img->h = h;
    img->vscale = (float)h / (float)SCREEN_H;
    img->cov = (unsigned char*)calloc((size_t)SCREEN_W * h, 1);
    img->depth = (float*)malloc(sizeof(float) * (size_t)SCREEN_W * h);
}

static void image_free(Image* img) {
    free(img->cov);
    free(img->depth);
}

static void image_reset(Image* img) {
    size_t n = (size_t)SCREEN_W * img->h;
    memset(img->cov, 0, n);
    for (size_t i = 0; i < n; i++) img->depth[i] = 1e30f;
    for (int x = 0; x < SCREEN_W; x++) img->top[x] = -1;
}

static void image_finish(Image* img) {
    for (int y = 0; y < img->h; y++)
        for (int x = 0; x < SCREEN_W; x++)
            if (img->cov[(size_t)y * SCREEN_W + x] && img->top[x] < 0) img->top[x] = y;
}

/* Triangulate one chunk.  A border cell edge is split once when the neighbour
   across that border is finer; the split cell is fanned from its centre, so
   the split edge stays a single straight run of vertices. */
static void draw_chunk(const Heightfield* hf, const Camera* cam, const int* active,
                       const int* levels, int cx, int cz, float focal, Image* img,
                       RenderInfo* info) {
    int i = cz * NCHUNK + cx;
    int L = levels[i];
    int step = 1 << L;
    int x0 = cx * CHUNK, z0 = cz * CHUNK;
    int finer[4];
    for (int d = 0; d < 4; d++) {
        int j = neighbor_index(cx, cz, d);
        finer[d] = (j >= 0 && active[j] && levels[j] < L);
    }

    for (int gz = 0; gz < CHUNK; gz += step) {
        for (int gx = 0; gx < CHUNK; gx += step) {
            int ax = x0 + gx, az = z0 + gz;
            float h00 = hf->h[az][ax];
            float h10 = hf->h[az][ax + step];
            float h01 = hf->h[az + step][ax];
            float h11 = hf->h[az + step][ax + step];
            info->cells++;

            int split_w = finer[0] && gx == 0;
            int split_e = finer[1] && gx + step == CHUNK;
            int split_s = finer[2] && gz == 0;
            int split_n = finer[3] && gz + step == CHUNK;

            if (!(split_w | split_e | split_s | split_n)) {
                CamV a = to_cam(cam, (float)ax * SAMPLE_M, h00, (float)az * SAMPLE_M);
                CamV b = to_cam(cam, (float)(ax + step) * SAMPLE_M, h10, (float)az * SAMPLE_M);
                CamV c = to_cam(cam, (float)(ax + step) * SAMPLE_M, h11, (float)(az + step) * SAMPLE_M);
                CamV d = to_cam(cam, (float)ax * SAMPLE_M, h01, (float)(az + step) * SAMPLE_M);
                draw_tri(focal, img, a, b, c);
                draw_tri(focal, img, a, c, d);
                info->tris += 2;
                continue;
            }

            float sx[8], sy[8], sh[8];
            int np = 0;
            sx[np] = (float)ax;                 sy[np] = (float)az;                 sh[np] = h00; np++;
            if (split_s) { sx[np] = (float)(ax + step / 2); sy[np] = (float)az;                 sh[np] = hf->h[az][ax + step / 2]; np++; }
            sx[np] = (float)(ax + step);        sy[np] = (float)az;                 sh[np] = h10; np++;
            if (split_e) { sx[np] = (float)(ax + step);      sy[np] = (float)(az + step / 2);   sh[np] = hf->h[az + step / 2][ax + step]; np++; }
            sx[np] = (float)(ax + step);        sy[np] = (float)(az + step);        sh[np] = h11; np++;
            if (split_n) { sx[np] = (float)(ax + step / 2); sy[np] = (float)(az + step);       sh[np] = hf->h[az + step][ax + step / 2]; np++; }
            sx[np] = (float)ax;                 sy[np] = (float)(az + step);        sh[np] = h01; np++;
            if (split_w) { sx[np] = (float)ax;               sy[np] = (float)(az + step / 2);   sh[np] = hf->h[az + step / 2][ax]; np++; }

            CamV cc = to_cam(cam, ((float)ax + (float)step * 0.5f) * SAMPLE_M,
                             0.25f * (h00 + h10 + h01 + h11),
                             ((float)az + (float)step * 0.5f) * SAMPLE_M);
            CamV prev = to_cam(cam, sx[0] * SAMPLE_M, sh[0], sy[0] * SAMPLE_M);
            for (int k = 1; k < np; k++) {
                CamV cur = to_cam(cam, sx[k] * SAMPLE_M, sh[k], sy[k] * SAMPLE_M);
                draw_tri(focal, img, cc, prev, cur);
                info->tris++;
                prev = cur;
            }
        }
    }
}

static void render(const Heightfield* hf, const int* active, const int* levels,
                   const Camera* cam, Image* img, RenderInfo* info) {
    image_reset(img);
    float focal = focal_length(cam);
    memset(info, 0, sizeof(*info));
    for (int cz = 0; cz < NCHUNK; cz++) {
        for (int cx = 0; cx < NCHUNK; cx++) {
            if (!active[cz * NCHUNK + cx]) continue;
            info->visible++;
            draw_chunk(hf, cam, active, levels, cx, cz, focal, img, info);
        }
    }
    image_finish(img);
}

/* ------------------------------------------------------------------ checks --- */

/* Vertex spacing a chunk emits along a border.  A level-L cell edge is 2^L
   samples; when the neighbour is finer the coarse side adds one midpoint, so
   both sides land on the same set of vertices. */
static int edge_spacing(int self_level, int neighbor_level) {
    int spacing = 1 << self_level;
    if (neighbor_level < self_level) spacing /= 2;
    return spacing;
}

static uint64_t edge_digest_v(const Heightfield* hf, int x, int z0, int z1, int spacing) {
    uint64_t h = 1469598103934665603ull;
    for (int z = z0; z <= z1; z += spacing) {
        h ^= (uint64_t)(int64_t)(hf->h[z][x] * 1024.0f);
        h *= 1099511628211ull;
    }
    return h;
}

static uint64_t edge_digest_h(const Heightfield* hf, int z, int x0, int x1, int spacing) {
    uint64_t h = 1469598103934665603ull;
    for (int x = x0; x <= x1; x += spacing) {
        h ^= (uint64_t)(int64_t)(hf->h[z][x] * 1024.0f);
        h *= 1099511628211ull;
    }
    return h;
}

static int seam_spacing(const int* levels, int self, int neighbor) {
    return edge_spacing(levels[self], levels[neighbor]);
}

/* Digest of the height samples one branch exposes along border `dir` of chunk
   (cx, cz).  Sampling at `spacing` models the vertex list of that side. */
static uint64_t branch_digest(const Heightfield* hf, int cx, int cz, int dir, int spacing) {
    int x0 = cx * CHUNK, z0 = cz * CHUNK;
    switch (dir) {
    case 0: return edge_digest_v(hf, x0, z0, z0 + CHUNK, spacing);
    case 1: return edge_digest_v(hf, x0 + CHUNK, z0, z0 + CHUNK, spacing);
    case 2: return edge_digest_h(hf, z0, x0, x0 + CHUNK, spacing);
    default: return edge_digest_h(hf, z0 + CHUNK, x0, x0 + CHUNK, spacing);
    }
}

/* Largest sub-pixel offset between the two skylines, in screen pixels.  The
   caller renders at `vscale` rows per pixel, so one pixel is `vscale` rows. */
static double skyline_shift_px(const Image* a, const Image* b, float vscale, int* columns) {
    int worst = 0;
    *columns = 0;
    for (int x = 0; x < SCREEN_W; x++) {
        if (a->top[x] < 0 || b->top[x] < 0) continue;
        (*columns)++;
        int d = a->top[x] - b->top[x];
        if (d < 0) d = -d;
        if (d > worst) worst = d;
    }
    return (double)worst / (double)vscale;
}

/* Pinholes: pixels the full-precision image covers, the LOD image leaves
   uncovered, and the LOD image covers on all eight neighbours.  An enclosed
   gap like that is a mesh crack; a skyline shift is not. */
static int interior_holes(const Image* full, const Image* lod) {
    int holes = 0;
    for (int y = 1; y < full->h - 1; y++) {
        for (int x = 1; x < SCREEN_W - 1; x++) {
            int o = y * SCREEN_W + x;
            if (!full->cov[o] || lod->cov[o]) continue;
            int enclosed = 1;
            for (int dy = -1; dy <= 1 && enclosed; dy++)
                for (int dx = -1; dx <= 1; dx++)
                    if (!lod->cov[o + dy * SCREEN_W + dx]) { enclosed = 0; break; }
            if (enclosed) holes++;
        }
    }
    return holes;
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e3 + (double)ts.tv_nsec * 1e-6;
}

/* -------------------------------------------------------------------- main --- */

int main(void) {
    Heightfield* hf = (Heightfield*)malloc(sizeof(Heightfield));
    Chunk* chunks = (Chunk*)calloc(NCHUNK * NCHUNK, sizeof(Chunk));
    int* active = (int*)calloc(NCHUNK * NCHUNK, sizeof(int));
    int* levels = (int*)calloc(NCHUNK * NCHUNK, sizeof(int));
    int* full_levels = (int*)calloc(NCHUNK * NCHUNK, sizeof(int));
    Image full, lod;
    image_alloc(&full, SCREEN_H * SUPER);
    image_alloc(&lod, SCREEN_H * SUPER);
    if (!hf || !chunks || !active || !levels || !full_levels || !full.cov || !full.depth ||
        !lod.cov || !lod.depth) {
        printf("alloc_failed=1\n");
        return 2;
    }

    heightfield_init(hf);
    chunk_init(chunks, hf);

    int zero_ok = 1, mono_ok = 1;
    float max_err = 0.0f;
    for (int i = 0; i < NCHUNK * NCHUNK; i++) {
        if (chunks[i].err[0] != 0.0f) zero_ok = 0;
        for (int L = 1; L < NLEV; L++) {
            if (chunks[i].err[L] < chunks[i].err[L - 1]) mono_ok = 0;
            if (chunks[i].err[L] > max_err) max_err = chunks[i].err[L];
        }
    }
    check("level0_exact", zero_ok);
    check("error_monotone", mono_ok);
    check("error_nonzero", max_err > 0.0f);

    Camera cam = { 1024.0f, 70.0f, 10.0f, 0.18f, 1.0f };
    int visible = 0;
    for (int cz = 0; cz < NCHUNK; cz++) {
        for (int cx = 0; cx < NCHUNK; cx++) {
            int i = cz * NCHUNK + cx;
            active[i] = chunk_visible(&chunks[i], &cam);
            levels[i] = active[i] ? select_level(&chunks[i], &cam, TOL_PX) : 0;
            full_levels[i] = 0;
            if (active[i]) visible++;
        }
    }
    int refined = balance_levels(active, levels);

    int max_diff = 0;
    for (int cz = 0; cz < NCHUNK; cz++)
        for (int cx = 0; cx < NCHUNK; cx++) {
            int i = cz * NCHUNK + cx;
            if (!active[i]) continue;
            for (int d = 0; d < 4; d++) {
                int j = neighbor_index(cx, cz, d);
                if (j < 0 || !active[j]) continue;
                int diff = levels[i] - levels[j];
                if (diff < 0) diff = -diff;
                if (diff > max_diff) max_diff = diff;
            }
        }
    check("frustum_culls", visible > 0 && visible < NCHUNK * NCHUNK);
    check("levels_balanced", max_diff <= 1);

    /* Balance pass has teeth: a lone 3-beside-1 pair is pulled to 2. */
    {
        int* test_active = (int*)calloc(NCHUNK * NCHUNK, sizeof(int));
        int* test_levels = (int*)calloc(NCHUNK * NCHUNK, sizeof(int));
        test_active[0] = test_active[1] = 1;
        test_levels[0] = 3;
        test_levels[1] = 1;
        int n = balance_levels(test_active, test_levels);
        check("balance_refines", n > 0 && test_levels[0] == 2 && test_levels[1] == 1);
        test_active[0] = 0; test_active[1] = 0;    /* a culled neighbour does not count */
        test_levels[0] = 3; test_levels[1] = 0;
        n = balance_levels(test_active, test_levels);
        check("balance_skips_culled", n == 0 && test_levels[0] == 3);
        free(test_active);
        free(test_levels);
    }

    int near_i = 0 * NCHUNK + 4;
    int far_i = (NCHUNK - 1) * NCHUNK + 4;
    check("near_full_precision", levels[near_i] == 0);
    check("far_uses_lod", levels[far_i] > levels[near_i]);

    RenderInfo info_full, info_lod;
    render(hf, active, full_levels, &cam, &full, &info_full);
    render(hf, active, levels, &cam, &lod, &info_lod);

    /* every drawn chunk border must expose identical vertices from both sides */
    int seams = 0, seam_bad = 0, ring_borders = 0;
    for (int cz = 0; cz < NCHUNK; cz++)
        for (int cx = 0; cx < NCHUNK; cx++) {
            int i = cz * NCHUNK + cx;
            if (!active[i]) continue;
            for (int d = 0; d < 4; d++) {
                int j = neighbor_index(cx, cz, d);
                if (j < 0 || !active[j]) continue;
                seams++;
                if (levels[i] != levels[j]) ring_borders++;
                if (seam_spacing(levels, i, j) != seam_spacing(levels, j, i) ||
                    branch_digest(hf, cx, cz, d, seam_spacing(levels, i, j)) !=
                    branch_digest(hf, cx, cz, d, seam_spacing(levels, j, i))) seam_bad++;
            }
        }
    printf("seams=borders=%d lod_rings=%d mismatched=%d\n", seams, ring_borders, seam_bad);
    check("seam_vertices_match", seams > 0 && ring_borders > 0 && seam_bad == 0);

    float worst_px = 0.0f;
    for (int i = 0; i < NCHUNK * NCHUNK; i++) {
        if (!active[i]) continue;
        float e = screen_error(&chunks[i], levels[i], &cam);
        if (e > worst_px) worst_px = e;
    }

    int columns = 0;
    double skyline_px = skyline_shift_px(&full, &lod, (float)SUPER, &columns);
    int holes = interior_holes(&full, &lod);

    int hist[NLEV] = { 0 };
    for (int i = 0; i < NCHUNK * NCHUNK; i++) if (active[i]) hist[levels[i]]++;

    printf("chunks_total=%d chunks_visible=%d chunks_refined=%d\n",
           NCHUNK * NCHUNK, visible, refined);
    printf("levels=%d,%d,%d,%d,%d,%d,%d\n",
           hist[0], hist[1], hist[2], hist[3], hist[4], hist[5], hist[6]);
    printf("cells_full=%d cells_lod=%d tris_full=%d tris_lod=%d\n",
           info_full.cells, info_lod.cells, info_full.tris, info_lod.tris);
    printf("max_screen_error_px=%.3f tolerance_px=%.2f\n", worst_px, TOL_PX);
    printf("skyline_px_max=%.2f skyline_columns=%d/%d supersample=%dx\n",
           skyline_px, columns, SCREEN_W, SUPER);
    printf("crack_pixels=%d\n", holes);

    /* timing: same camera and culling, only the level choice differs */
    int iters = 5;
    double t0 = now_ms();
    for (int k = 0; k < iters; k++) render(hf, active, full_levels, &cam, &full, &info_full);
    double ms_full = (now_ms() - t0) / iters;
    t0 = now_ms();
    for (int k = 0; k < iters; k++) render(hf, active, levels, &cam, &lod, &info_lod);
    double ms_lod = (now_ms() - t0) / iters;
    printf("draw_ms_full=%.2f draw_ms_lod=%.2f speedup=%.2fx\n",
           ms_full, ms_lod, ms_lod > 0.0 ? ms_full / ms_lod : 0.0);

    check("lod_uses_fewer_cells", info_lod.cells < info_full.cells);
    check("lod_draws_less", ms_lod < ms_full);
    check("screen_error_under_one_px", worst_px < 1.0f);
    check("skyline_within_one_px", skyline_px <= 1.0);
    check("no_cracks", holes == 0);

    /* A second, deterministic arrangement for the seam path: bands at levels
       0..3 put a one-level ring on every band border regardless of the camera
       sweep, so the stitching rule is exercised even on views where the error
       budget happens to give all chunks the same level. */
    int* banded = (int*)calloc(NCHUNK * NCHUNK, sizeof(int));
    int* gap = (int*)calloc(NCHUNK * NCHUNK, sizeof(int));
    for (int cz = 0; cz < NCHUNK; cz++)
        for (int cx = 0; cx < NCHUNK; cx++) {
            banded[cz * NCHUNK + cx] = cz < 4 ? cz : 3;
            gap[cz * NCHUNK + cx] = cz < 2 ? 0 : (cz < 3 ? 2 : 3);   /* jump of two */
        }

    Image band_img, gap_img;
    image_alloc(&band_img, SCREEN_H * SUPER);
    image_alloc(&gap_img, SCREEN_H * SUPER);
    RenderInfo info_band, info_gap;
    render(hf, active, banded, &cam, &band_img, &info_band);
    render(hf, active, gap, &cam, &gap_img, &info_gap);

    int band_bad = 0, band_rings = 0, band_vertices = 0;
    int gap_bad = 0;
    for (int cz = 0; cz < NCHUNK; cz++)
        for (int cx = 0; cx < NCHUNK; cx++) {
            int i = cz * NCHUNK + cx;
            if (!active[i]) continue;
            for (int d = 0; d < 4; d++) {
                int j = neighbor_index(cx, cz, d);
                if (j < 0 || !active[j]) continue;
                int sa = seam_spacing(banded, i, j);
                int sb = seam_spacing(banded, j, i);
                if (banded[i] != banded[j] && (d == 1 || d == 3)) {
                    int fine = sa < sb ? sa : sb;
                    band_rings++;
                    band_vertices += CHUNK / fine + 1;
                }
                if (branch_digest(hf, cx, cz, d, sa) != branch_digest(hf, cx, cz, d, sb)) band_bad++;
                if (branch_digest(hf, cx, cz, d, seam_spacing(gap, i, j)) !=
                    branch_digest(hf, cx, cz, d, seam_spacing(gap, j, i))) gap_bad++;
            }
        }
    int band_cracks = interior_holes(&full, &band_img);
    printf("band_rings=borders=%d edge_vertices=%d mismatched_seams=%d crack_pixels=%d\n",
           band_rings, band_vertices, band_bad, band_cracks);
    check("band_ring_borders_present", band_rings > 0);
    check("band_ring_seams_watertight", band_bad == 0 && band_cracks == 0);
    /* the digest must be able to see a T-junction, or the check above is empty:
       a two-level jump is not stitched by a single midpoint and must show. */
    printf("two_level_jump=mismatched_seams=%d\n", gap_bad);
    check("seam_check_detects_gap", gap_bad > 0);
    image_free(&band_img);
    image_free(&gap_img);
    free(banded);
    free(gap);

    free(hf); free(chunks); free(active); free(levels); free(full_levels);
    image_free(&full);
    image_free(&lod);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
