
/* occlusion: 层次遮挡剔除。
   - 保守层次包围盒(BVH):父节点包围盒为子节点并集,子树剔除永不误杀
   - 按视锥逐层裁剪:节点越界整棵子树跳过
   - 遮挡查询:软件深度缓冲 + 包围盒屏幕矩形逐纹素判定;
     查询不可答(投影退化 / 遮挡系统关闭)时回退为"可见"或纯视锥裁剪
   - 自检:层次剔除可见集 == 逐物体逐像素判定的可见集;
     被剔除物体绘制调用下降;层次构建 < 1s */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define MAX_OBJ 24000
#define LEAF_CAP 4
#define DW 320              /* 遮挡深度缓冲分辨率 */
#define DH 180
#define ZNEAR 0.5f
#define ZFAR  300.0f

typedef struct { float x, y, z; } Vec3;
typedef struct { Vec3 mn, mx; } AABB;
typedef struct { AABB b; int occluder; } Object;

typedef struct {
    AABB b;
    int left, right;    /* 子节点下标,叶子为 -1 */
    int start, count;   /* 叶子:order[] 中的区间 */
} Node;

static Object objs[MAX_OBJ];
static int nobj;
static int order[MAX_OBJ];
static Node nodes[2 * MAX_OBJ];
static int nnodes;

static float depth_buf[DH][DW];
static int occ_available = 1;   /* 遮挡系统开关:关闭时整体回退为纯视锥裁剪 */
static int force_unknown = 0;   /* 测试用:模拟查询不可答,走 UNKNOWN 回退 */

/* 相机位于原点、朝 +z,故 AABB 的屏幕投影恰为矩形 */
static const float FOVY = 60.0f * 3.14159265f / 180.0f;
static const float ASPECT = 16.0f / 9.0f;

/* ---------- 视锥 ---------- */

typedef struct { float a, b, c, d; } Plane;
static Plane frustum[6];

static void frustum_init(void) {
    float hy = FOVY * 0.5f;
    float hx = atanf(tanf(hy) * ASPECT);
    float cx = cosf(hx), sx = sinf(hx);
    float cy = cosf(hy), sy = sinf(hy);
    frustum[0] = (Plane){ 0, 0, 1, -ZNEAR };   /* near  */
    frustum[1] = (Plane){ 0, 0,-1,  ZFAR  };   /* far   */
    frustum[2] = (Plane){ cx, 0, sx, 0 };      /* left  */
    frustum[3] = (Plane){-cx, 0, sx, 0 };      /* right */
    frustum[4] = (Plane){ 0, cy, sy, 0 };      /* down  */
    frustum[5] = (Plane){ 0,-cy, sy, 0 };      /* up    */
}

/* 保守:仅当盒子整体位于某平面负侧才判出界 */
static int box_outside_frustum(const AABB* b) {
    for (int i = 0; i < 6; i++) {
        const Plane* p = &frustum[i];
        float px = p->a > 0 ? b->mx.x : b->mn.x;
        float py = p->b > 0 ? b->mx.y : b->mn.y;
        float pz = p->c > 0 ? b->mx.z : b->mn.z;
        if (p->a * px + p->b * py + p->c * pz + p->d < 0.0f) return 1;
    }
    return 0;
}

/* ---------- 遮挡查询 ---------- */

enum { Q_VISIBLE, Q_OCCLUDED, Q_UNKNOWN };

/* 投影包围盒到屏幕矩形,输出纹素范围与最近/最远深度 */
static void project_box(const AABB* b, float* x0, float* y0, float* x1, float* y1,
                        float* znear, float* zfar) {
    float tx = tanf(FOVY * 0.5f) * ASPECT;
    float ty = tanf(FOVY * 0.5f);
    float ax0 = 1e30f, ay0 = 1e30f, ax1 = -1e30f, ay1 = -1e30f;
    float zn = 1e30f, zf = -1e30f;
    for (int i = 0; i < 8; i++) {
        float x = (i & 1) ? b->mx.x : b->mn.x;
        float y = (i & 2) ? b->mx.y : b->mn.y;
        float z = (i & 4) ? b->mx.z : b->mn.z;
        if (z < ZNEAR) z = ZNEAR;            /* 压到近平面,保守 */
        float sx = (x / z / tx * 0.5f + 0.5f) * DW;
        float sy = (0.5f - y / z / ty * 0.5f) * DH;
        if (sx < ax0) ax0 = sx;
        if (sx > ax1) ax1 = sx;
        if (sy < ay0) ay0 = sy;
        if (sy > ay1) ay1 = sy;
        if (z < zn) zn = z;
        if (z > zf) zf = z;
    }
    *x0 = ax0; *y0 = ay0; *x1 = ax1; *y1 = ay1; *znear = zn; *zfar = zf;
}

/* 遮挡物写入深度缓冲:用盒子的最远深度填充(保守,不会高估遮挡) */
static void raster_occluder(const AABB* b) {
    float x0, y0, x1, y1, zn, zf;
    project_box(b, &x0, &y0, &x1, &y1, &zn, &zf);
    int ix0 = (int)floorf(x0), ix1 = (int)ceilf(x1);
    int iy0 = (int)floorf(y0), iy1 = (int)ceilf(y1);
    if (ix0 < 0) ix0 = 0;
    if (iy0 < 0) iy0 = 0;
    if (ix1 > DW) ix1 = DW;
    if (iy1 > DH) iy1 = DH;
    for (int y = iy0; y < iy1; y++)
        for (int x = ix0; x < ix1; x++)
            if (zf < depth_buf[y][x]) depth_buf[y][x] = zf;
}

/* 查询:盒子覆盖的所有纹素深度都小于其最近深度 => 完全遮挡。
   不足一纹素时也采样其覆盖的纹素,保证与逐物体判定严格一致:
   节点遮挡 => 子节点矩形被包含且最近深度更大 => 子节点必遮挡。 */
static int query_box(const AABB* b) {
    if (!occ_available || force_unknown) return Q_UNKNOWN;
    float x0, y0, x1, y1, zn, zf;
    project_box(b, &x0, &y0, &x1, &y1, &zn, &zf);
    int ix0 = (int)floorf(x0), ix1 = (int)ceilf(x1);
    int iy0 = (int)floorf(y0), iy1 = (int)ceilf(y1);
    if (ix0 < 0) ix0 = 0;
    if (iy0 < 0) iy0 = 0;
    if (ix1 > DW) ix1 = DW;
    if (iy1 > DH) iy1 = DH;
    if (ix0 >= ix1 || iy0 >= iy1) return Q_UNKNOWN;
    for (int y = iy0; y < iy1; y++)
        for (int x = ix0; x < ix1; x++)
            if (depth_buf[y][x] >= zn - 1e-4f) return Q_VISIBLE;
    return Q_OCCLUDED;
}

/* ---------- BVH 构建 ---------- */

static int cmp_axis;
static int cmp_obj(const void* pa, const void* pb) {
    const Object* a = &objs[*(const int*)pa];
    const Object* b = &objs[*(const int*)pb];
    float ca, cb;
    if (cmp_axis == 0) {
        ca = a->b.mn.x + a->b.mx.x; cb = b->b.mn.x + b->b.mx.x;
    } else if (cmp_axis == 1) {
        ca = a->b.mn.y + a->b.mx.y; cb = b->b.mn.y + b->b.mx.y;
    } else {
        ca = a->b.mn.z + a->b.mx.z; cb = b->b.mn.z + b->b.mx.z;
    }
    return ca < cb ? -1 : ca > cb ? 1 : 0;
}

static void bounds_expand(AABB* d, const AABB* s) {
    if (s->mn.x < d->mn.x) d->mn.x = s->mn.x;
    if (s->mn.y < d->mn.y) d->mn.y = s->mn.y;
    if (s->mn.z < d->mn.z) d->mn.z = s->mn.z;
    if (s->mx.x > d->mx.x) d->mx.x = s->mx.x;
    if (s->mx.y > d->mx.y) d->mx.y = s->mx.y;
    if (s->mx.z > d->mx.z) d->mx.z = s->mx.z;
}

static int build(int start, int count) {
    int ni = nnodes++;
    Node* n = &nodes[ni];
    n->b = (AABB){{ 1e30f, 1e30f, 1e30f }, { -1e30f, -1e30f, -1e30f }};
    for (int i = start; i < start + count; i++)
        bounds_expand(&n->b, &objs[order[i]].b);   /* 父盒 = 子盒并集,保守 */
    if (count <= LEAF_CAP) {
        n->left = n->right = -1;
        n->start = start; n->count = count;
        return ni;
    }
    Vec3 e = { n->b.mx.x - n->b.mn.x, n->b.mx.y - n->b.mn.y, n->b.mx.z - n->b.mn.z };
    cmp_axis = (e.x >= e.y && e.x >= e.z) ? 0 : (e.y >= e.z) ? 1 : 2;
    qsort(order + start, count, sizeof(int), cmp_obj);
    n->start = n->count = 0;
    n->left = build(start, count / 2);
    n->right = build(start + count / 2, count - count / 2);
    return ni;
}

/* ---------- 遍历剔除 ---------- */

static long draws;

static void traverse(int ni, int use_occ, unsigned char* vis) {
    Node* n = &nodes[ni];
    if (box_outside_frustum(&n->b)) return;                 /* 视锥:整层裁掉 */
    if (use_occ && query_box(&n->b) == Q_OCCLUDED) return;  /* 遮挡:整棵子树裁掉 */
    /* Q_UNKNOWN:回退,继续向下细分 */
    if (n->left < 0) {
        for (int i = n->start; i < n->start + n->count; i++) {
            int idx = order[i];
            if (box_outside_frustum(&objs[idx].b)) continue;
            if (use_occ && query_box(&objs[idx].b) == Q_OCCLUDED) continue;
            /* Q_UNKNOWN 回退为可见,保守不误杀 */
            vis[idx] = 1;
            draws++;
        }
    } else {
        traverse(n->left, use_occ, vis);
        traverse(n->right, use_occ, vis);
    }
}

/* 基准:逐物体逐像素判定(不做任何层次加速) */
static void brute_force(int use_occ, unsigned char* vis, long* out_draws) {
    long d = 0;
    for (int i = 0; i < nobj; i++) {
        if (box_outside_frustum(&objs[i].b)) continue;
        if (use_occ && query_box(&objs[i].b) == Q_OCCLUDED) continue;
        vis[i] = 1;
        d++;
    }
    *out_draws = d;
}

/* ---------- 场景 ---------- */

static unsigned rng_state = 12345;
static float frand(float lo, float hi) {
    rng_state = rng_state * 1664525u + 1013904223u;
    return lo + (hi - lo) * (float)(rng_state >> 8) / 16777216.0f;
}

static void add_box(float cx, float cy, float cz, float hx, float hy, float hz, int occ) {
    Object* o = &objs[nobj++];
    o->b.mn = (Vec3){ cx - hx, cy - hy, cz - hz };
    o->b.mx = (Vec3){ cx + hx, cy + hy, cz + hz };
    o->occluder = occ;
}

static void scene_build(int nsmall) {
    nobj = 0;
    /* 前景遮挡墙:z 25~45,铺满视野大部分 */
    for (int i = 0; i < 6; i++)
        add_box(frand(-28, 28), frand(-12, 12), frand(25, 45),
                frand(7, 11), frand(5, 8), 1.5f, 1);
    /* 墙前少量物体:必须保持可见 */
    for (int i = 0; i < 200; i++)
        add_box(frand(-15, 15), frand(-8, 8), frand(4, 20),
                frand(0.3f, 1.0f), frand(0.3f, 1.0f), frand(0.3f, 1.0f), 0);
    /* 墙后大量小物体:大部分应被遮挡剔除 */
    for (int i = 0; i < nsmall; i++)
        add_box(frand(-60, 60), frand(-30, 30), frand(50, 120),
                frand(0.3f, 1.2f), frand(0.3f, 1.2f), frand(0.3f, 1.2f), 0);
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

int main(void) {
    frustum_init();
    scene_build(20000);

    double t0 = now_ms();
    for (int i = 0; i < nobj; i++) order[i] = i;
    nnodes = 0;
    build(0, nobj);
    double build_ms = now_ms() - t0;

    /* 遮挡物写入深度缓冲 */
    for (int y = 0; y < DH; y++)
        for (int x = 0; x < DW; x++)
            depth_buf[y][x] = ZFAR;
    for (int i = 0; i < nobj; i++)
        if (objs[i].occluder && !box_outside_frustum(&objs[i].b))
            raster_occluder(&objs[i].b);

    static unsigned char vis_hier[MAX_OBJ], vis_ref[MAX_OBJ], vis_f[MAX_OBJ];

    /* 1) 纯视锥(遮挡系统不可用时的整体回退路径) */
    draws = 0;
    memset(vis_f, 0, nobj);
    traverse(0, 0, vis_f);
    long draws_frustum = draws;

    /* 2) 视锥 + 层次遮挡 */
    draws = 0;
    memset(vis_hier, 0, nobj);
    traverse(0, 1, vis_hier);
    long draws_occ = draws;

    /* 3) 基准:逐物体逐像素 */
    long draws_ref;
    memset(vis_ref, 0, nobj);
    brute_force(1, vis_ref, &draws_ref);

    int consistent = (draws_occ == draws_ref);
    for (int i = 0; i < nobj && consistent; i++)
        if (vis_hier[i] != vis_ref[i]) consistent = 0;

    /* 4) 回退路径一:遮挡系统关闭 => 退化为纯视锥,集合必须一致 */
    occ_available = 0;
    static unsigned char vis_fb[MAX_OBJ];
    draws = 0;
    memset(vis_fb, 0, nobj);
    traverse(0, 1, vis_fb);
    int fallback_off_ok = (draws == draws_frustum);
    for (int i = 0; i < nobj && fallback_off_ok; i++)
        if (vis_fb[i] != vis_f[i]) fallback_off_ok = 0;
    occ_available = 1;

    /* 5) 回退路径二:查询全部不可答 => 不得误杀任何视锥内物体 */
    force_unknown = 1;
    draws = 0;
    memset(vis_fb, 0, nobj);
    traverse(0, 1, vis_fb);
    int fallback_unknown_ok = (draws == draws_frustum);
    for (int i = 0; i < nobj && fallback_unknown_ok; i++)
        if (vis_fb[i] != vis_f[i]) fallback_unknown_ok = 0;
    force_unknown = 0;

    long culled = draws_frustum - draws_occ;
    printf("objects=%d nodes=%d build_ms=%.1f\n", nobj, nnodes, build_ms);
    printf("draws_frustum=%ld draws_occlusion=%ld culled=%ld (%.0f%%)\n",
           draws_frustum, draws_occ, culled,
           draws_frustum ? 100.0 * culled / draws_frustum : 0.0);
    printf("consistent_with_perpixel=%d\n", consistent);
    printf("fallback_disabled_ok=%d fallback_unknown_ok=%d\n",
           fallback_off_ok, fallback_unknown_ok);
    printf("build_under_1s=%d\n", build_ms < 1000.0);
    printf("culling_effective=%d\n", culled > 0 && draws_occ < draws_frustum);
    return !(consistent && fallback_off_ok && fallback_unknown_ok &&
             build_ms < 1000.0 && culled > 0);
}
