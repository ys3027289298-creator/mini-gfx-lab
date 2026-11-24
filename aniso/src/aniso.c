
/* aniso-filter: 按表面朝向自适应的各向异性贴图过滤。

   系数推导：像素在贴图上的足迹（footprint）随 法线-视线夹角 θ 拉长，
   对平面而言覆盖面积 ∝ 1/cosθ，且拉长全部发生在倾斜方向上，故
     各向异性采样数 N = floor(1/cosθ + 0.75)   （比例超 1.25 才加采样，上限 ANISO_MAX）
     mip 级别取足迹短轴（保持跨轴清晰度），N 个采样沿长轴均布。
   该系数只依赖贴图尺度与像素覆盖面积；足迹雅可比 J(du,dv)/J(dx,dy) 的
   奇异值给出基准值（自检输出的 ratio_jac 列）。注意 1/cosθ 只是
   "表面倾斜"的近似：正对工作平面但离轴时 1/cosθ>1 而 ratio_jac=1，
   雅可比才是 ground truth。

   自检：同一场景渲染正面平面与 45° 斜面（同一贴图、同一贴图尺度），
   在两个面上各取相同贴图尺寸的区域，分别计算清晰度指标
   （Tenengrad / 64 次采样参考渲染的 Tenengrad，归一化后两面可比较）。
   若斜面清晰度反而显著高于平面，说明斜面在欠采样（走样能量被误读为
   清晰），即系数方向弄反了。 */
#include <stdio.h>
#include <math.h>

#define TEX 512
#define LEVELS 10            /* 512..1 */
#define ANISO_MAX 16
#define REF_SAMPLES 64       /* 参考渲染的沿轴采样数 */
#define IMG_W 640
#define IMG_H 360
#define FOCAL_PX 311.7691f   /* (IMG_H/2)/tan(30°) */
#define TEX_SCALE 0.5f       /* uv 单位 / 世界单位，两个面相同 */
#define CROP_HALF 0.3f       /* 裁剪窗口的局部坐标半径（两面相同 => 贴图尺寸相同） */

static unsigned char tex[LEVELS][TEX * TEX];

/* ---------------- 贴图与 mip 链 ---------------- */

static unsigned hash2(int x, int y) {
    unsigned h = (unsigned)x * 374761393u + (unsigned)y * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}

static void build_texture(void) {
    for (int y = 0; y < TEX; y++)
        for (int x = 0; x < TEX; x++) {
            int c8  = ((x >> 3) ^ (y >> 3)) & 1;   /* 8 texel 棋盘 */
            int c32 = ((x >> 5) ^ (y >> 5)) & 1;   /* 32 texel 棋盘 */
            float n = (hash2(x, y) & 255) / 255.0f;
            /* 高频噪声占优：mip 级别选错时清晰度指标才会明显变化 */
            float v = 0.35f + 0.15f * c32 + 0.15f * c8 + 0.50f * (n - 0.5f);
            if (v < 0.f) v = 0.f;
            if (v > 1.f) v = 1.f;
            tex[0][y * TEX + x] = (unsigned char)(v * 255.f);
        }
    for (int l = 1; l < LEVELS; l++) {
        int s = TEX >> l, p = s << 1;
        for (int y = 0; y < s; y++)
            for (int x = 0; x < s; x++) {
                int a = tex[l-1][(2*y  )*p + 2*x  ], b = tex[l-1][(2*y  )*p + 2*x+1];
                int c = tex[l-1][(2*y+1)*p + 2*x  ], d = tex[l-1][(2*y+1)*p + 2*x+1];
                tex[l][y*s + x] = (unsigned char)((a + b + c + d + 2) >> 2);
            }
    }
}

static float sample_level(int l, float u, float v) {
    int s = TEX >> l;
    float x = u * s - 0.5f, y = v * s - 0.5f;
    int x0 = (int)floorf(x), y0 = (int)floorf(y);
    float fx = x - x0, fy = y - y0;
    int x1 = x0 + 1, y1 = y0 + 1;
    x0 = ((x0 % s) + s) % s; x1 = ((x1 % s) + s) % s;
    y0 = ((y0 % s) + s) % s; y1 = ((y1 % s) + s) % s;
    float a = tex[l][y0*s + x0], b = tex[l][y0*s + x1];
    float c = tex[l][y1*s + x0], d = tex[l][y1*s + x1];
    return (a*(1-fx) + b*fx) * (1-fy) + (c*(1-fx) + d*fx) * fy;
}

static float sample_tri(float u, float v, float lod) {
    if (lod <= 0.f) return sample_level(0, u, v);
    if (lod >= LEVELS - 1) return sample_level(LEVELS - 1, u, v);
    int l0 = (int)lod;
    float f = lod - l0;
    return sample_level(l0, u, v) * (1-f) + sample_level(l0+1, u, v) * f;
}

/* ---------------- 场景：正面平面 + 45° 斜面 ---------------- */

typedef struct { float c[3], e1[3], e2[3], n[3], half; } Quad;

static const Quad QUADS[2] = {
    /* 平面：正对相机 */
    { {-1.1f, 0.f, 3.0f}, {1,0,0}, {0,1,0}, {0,0,-1}, 0.8f },
    /* 斜面：绕 x 轴倾斜 45°，法线 (0,-sin45,cos45)；中心深度与平面相同，
       使两面裁剪区的短轴 texel/像素 一致，清晰度指标可比较 */
    { { 1.1f, 0.f, 3.0f}, {1,0,0}, {0, 0.7071068f, 0.7071068f}, {0,-0.7071068f, 0.7071068f}, 0.8f },
};

static float dot3(const float* a, const float* b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }

static void ray_dir(float px, float py, float* d) {
    float x = (px + 0.5f - IMG_W * 0.5f) / FOCAL_PX;
    float y = (IMG_H * 0.5f - py - 0.5f) / FOCAL_PX;
    float il = 1.f / sqrtf(x*x + y*y + 1.f);
    d[0] = x * il; d[1] = y * il; d[2] = il;
}

/* 相机在原点；返回 t 并输出面内局部坐标 (s,t)，未命中返回 -1 */
static float hit_quad(const Quad* q, const float* d, float* out_s, float* out_t) {
    float dn = dot3(d, q->n);
    if (fabsf(dn) < 1e-9f) return -1.f;
    float t = dot3(q->c, q->n) / dn;
    if (t <= 0.f) return -1.f;
    float p[3] = { d[0]*t - q->c[0], d[1]*t - q->c[1], d[2]*t - q->c[2] };
    float s = dot3(p, q->e1), u = dot3(p, q->e2);
    if (fabsf(s) > q->half || fabsf(u) > q->half) return -1.f;
    *out_s = s; *out_t = u;
    return t;
}

/* ---------------- 足迹与自适应过滤 ---------------- */

typedef struct {
    float u, v;          /* 贴图坐标 */
    float lamaj, lamin;  /* 足迹长/短轴，texel/像素 */
    float ax, ay;        /* 长轴方向（uv 空间单位向量） */
    float cos_theta;     /* |法线·视线| */
} Foot;

/* 用光线微分（相邻像素的光线与同一平面求交）求贴图足迹雅可比 */
static int footprint_at(const Quad* q, int px, int py, Foot* f) {
    float d0[3], dx[3], dy[3];
    ray_dir((float)px, (float)py, d0);
    ray_dir((float)px + 1, (float)py, dx);
    ray_dir((float)px, (float)py + 1, dy);
    float s0, t0, sx, tx, sy, ty;
    if (hit_quad(q, d0, &s0, &t0) < 0.f) return -1;
    float k = TEX_SCALE * TEX;   /* 局部坐标 -> texel */
    float dudx, dudy, dvdx, dvdy;
    if (hit_quad(q, dx, &sx, &tx) > 0.f) { dudx = (sx-s0)*k; dvdx = (tx-t0)*k; }
    else { dudx = k * dot3(q->c, q->n) / FOCAL_PX; dvdx = 0.f; }  /* 边缘近似 */
    if (hit_quad(q, dy, &sy, &ty) > 0.f) { dudy = (sy-s0)*k; dvdy = (ty-t0)*k; }
    else { dudy = 0.f; dvdy = k * dot3(q->c, q->n) / FOCAL_PX; }

    float A = dudx*dudx + dudy*dudy;
    float B = dudx*dvdx + dudy*dvdy;
    float C = dvdx*dvdx + dvdy*dvdy;
    float tr = A + C, det = A*C - B*B;
    float disc = sqrtf(fmaxf(tr*tr*0.25f - det, 0.f));
    float l1 = tr*0.5f + disc, l2 = fmaxf(tr*0.5f - disc, 1e-12f);
    f->lamaj = sqrtf(l1); f->lamin = sqrtf(l2);

    /* 屏幕空间特征向量 -> uv 空间长轴方向 */
    float ex = B, ey = l1 - A;
    if (ex*ex + ey*ey < 1e-18f) { ex = l1 - C; ey = B; }
    if (ex*ex + ey*ey < 1e-18f) { ex = 1.f; ey = 0.f; }
    float il = 1.f / sqrtf(ex*ex + ey*ey);
    ex *= il; ey *= il;
    float ax = dudx*ex + dudy*ey, ay = dvdx*ex + dvdy*ey;
    float al = sqrtf(ax*ax + ay*ay);
    if (al < 1e-9f) { ax = 1.f; ay = 0.f; } else { ax /= al; ay /= al; }

    f->u = s0 * TEX_SCALE; f->v = t0 * TEX_SCALE;
    f->ax = ax; f->ay = ay;
    f->cos_theta = fabsf(dot3(q->n, d0));
    return 0;
}

enum { MODE_ISO = 0,   /* 现状：不过滤方向，lod 取最长轴 -> 斜面糊 */
       MODE_ANISO = 1, /* 修正：N 随 1/cosθ 增长，lod 取短轴，沿长轴采样 */
       MODE_REV  = 2 };/* 反向演示：N = ceil(cosθ)，夹角越大越接近各向同性 */

static int aniso_samples(float cos_theta, int mode) {
    if (mode == MODE_ISO) return 1;
    /* 比例超过 1.25 才值得加采样；反向演示用 cosθ（夹角越大采样越少） */
    float coeff = (mode == MODE_REV) ? cos_theta : 1.f / fmaxf(cos_theta, 1e-3f);
    int n = (int)floorf(coeff + 0.75f);
    if (n < 1) n = 1;
    if (n > ANISO_MAX) n = ANISO_MAX;
    return n;
}

static float filter_sample(const Foot* f, float lod, int n) {
    if (n <= 1) return sample_tri(f->u, f->v, lod);
    float acc = 0.f;
    for (int i = 0; i < n; i++) {
        float off = ((i + 0.5f) / n - 0.5f) * f->lamaj / TEX;  /* 沿长轴均布 */
        acc += sample_tri(f->u + off * f->ax, f->v + off * f->ay, lod);
    }
    return acc / n;
}

static float shade(const Quad* q, int px, int py, int mode, int* out_n) {
    Foot f;
    if (footprint_at(q, px, py, &f) < 0) return 40.f;
    int n = aniso_samples(f.cos_theta, mode);
    float lod = (mode == MODE_ISO) ? log2f(f.lamaj) : log2f(f.lamin);
    if (lod < 0.f) lod = 0.f;
    if (lod > LEVELS - 1) lod = LEVELS - 1;
    if (out_n) *out_n = n;
    return filter_sample(&f, lod, n);
}

/* 参考：收敛的各向异性积分（64 次沿轴采样），作为清晰度基准 */
static float shade_ref(const Quad* q, int px, int py) {
    Foot f;
    if (footprint_at(q, px, py, &f) < 0) return 40.f;
    float lod = log2f(f.lamin);
    if (lod < 0.f) lod = 0.f;
    if (lod > LEVELS - 1) lod = LEVELS - 1;
    return filter_sample(&f, lod, REF_SAMPLES);
}

static void render(float* img, int mode) {
    for (int y = 0; y < IMG_H; y++)
        for (int x = 0; x < IMG_W; x++) {
            float d[3]; ray_dir((float)x, (float)y, d);
            const Quad* best = 0; float bt = 1e30f;
            for (int i = 0; i < 2; i++) {
                float s, t, ht = hit_quad(&QUADS[i], d, &s, &t);
                if (ht > 0.f && ht < bt) { bt = ht; best = &QUADS[i]; }
            }
            if (!best) { img[y*IMG_W + x] = 40.f; continue; }
            img[y*IMG_W + x] = (mode < 0) ? shade_ref(best, x, y)
                                          : shade(best, x, y, mode, 0);
        }
}

/* ---------------- 测量 ---------------- */

/* 面内局部窗口 [-w,w]^2 投影到屏幕的包围盒（两面 w 相同 => 贴图尺寸相同） */
static void crop_box(const Quad* q, float w, int* box) {
    float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f;
    for (int i = 0; i < 4; i++) {
        float s = (i & 1) ? w : -w, t = (i & 2) ? w : -w;
        float p[3] = { q->c[0] + q->e1[0]*s + q->e2[0]*t,
                       q->c[1] + q->e1[1]*s + q->e2[1]*t,
                       q->c[2] + q->e1[2]*s + q->e2[2]*t };
        float sx = p[0] / p[2] * FOCAL_PX + IMG_W * 0.5f;
        float sy = IMG_H * 0.5f - p[1] / p[2] * FOCAL_PX;
        if (sx < x0) x0 = sx;
        if (sx > x1) x1 = sx;
        if (sy < y0) y0 = sy;
        if (sy > y1) y1 = sy;
    }
    box[0] = (int)ceilf(x0); box[1] = (int)ceilf(y0);
    box[2] = (int)floorf(x1); box[3] = (int)floorf(y1);
}

/* Tenengrad：梯度能量均值，模糊下降、走样噪声上升 */
static double tenengrad(const float* img, const int* box) {
    double acc = 0.0; long n = 0;
    for (int y = box[1] + 1; y < box[3]; y++)
        for (int x = box[0] + 1; x < box[2]; x++) {
            float gx = img[y*IMG_W + x+1] - img[y*IMG_W + x-1];
            float gy = img[(y+1)*IMG_W + x] - img[(y-1)*IMG_W + x];
            acc += gx*gx + gy*gy; n++;
        }
    return n ? acc / n : 0.0;
}

static int write_pgm(const char* path, const float* img, const int* box) {
    int x0 = box[0], y0 = box[1], w = box[2]-box[0]+1, h = box[3]-box[1]+1;
    FILE* f = fopen(path, "wb");
    if (!f) { perror(path); return -1; }
    fprintf(f, "P5\n%d %d\n255\n", w, h);
    for (int y = y0; y <= box[3]; y++)
        for (int x = x0; x <= box[2]; x++) {
            float v = img[y*IMG_W + x];
            fputc(v < 0.f ? 0 : v > 255.f ? 255 : (int)v, f);
        }
    fclose(f);
    return 0;
}

int main(void) {
    static float img_iso[IMG_W*IMG_H], img_an[IMG_W*IMG_H];
    static float img_rv[IMG_W*IMG_H],  img_ref[IMG_W*IMG_H];
    int box_full[4] = { 0, 0, IMG_W-1, IMG_H-1 };
    build_texture();
    render(img_ref, -1);
    render(img_iso, MODE_ISO);
    render(img_an,  MODE_ANISO);
    render(img_rv,  MODE_REV);

    int ba[4], bb[4];
    crop_box(&QUADS[0], CROP_HALF, ba);
    crop_box(&QUADS[1], CROP_HALF, bb);

    /* 推导量 vs 实际采样（取各面裁剪区中心像素） */
    printf("face   cosθ    cover(tex^2/px)  1/cosθ  ratio_jac  N_actual\n");
    for (int i = 0; i < 2; i++) {
        int* b = i ? bb : ba;
        int cx = (b[0]+b[2])/2, cy = (b[1]+b[3])/2;
        Foot f; int n = 0;
        footprint_at(&QUADS[i], cx, cy, &f);
        shade(&QUADS[i], cx, cy, MODE_ANISO, &n);
        printf("%-5s  %.3f   %13.2f  %6.3f  %9.3f  %8d\n",
               i ? "slope" : "plane", f.cos_theta, f.lamin * f.lamaj,
               1.f / f.cos_theta, f.lamaj / f.lamin, n);
    }

    /* 清晰度指标：各区域 Tenengrad 相对参考渲染的归一化值 */
    double rp = tenengrad(img_ref, ba), rs = tenengrad(img_ref, bb);
    double pi = tenengrad(img_iso, ba) / rp, si = tenengrad(img_iso, bb) / rs;
    double pa = tenengrad(img_an,  ba) / rp, sa = tenengrad(img_an,  bb) / rs;
    double pr = tenengrad(img_rv,  ba) / rp, sr = tenengrad(img_rv,  bb) / rs;
    printf("sharpness(T/T_ref)  plane  slope\n");
    printf("iso                 %5.2f  %5.2f   (现状：斜面糊)\n", pi, si);
    printf("aniso               %5.2f  %5.2f   (修正：N 随 1/cosθ 增长)\n", pa, sa);
    printf("reversed            %5.2f  %5.2f   (反向：夹角越大越各向同性)\n", pr, sr);

    /* 方向自检：斜面清晰度显著高于平面 => 斜面在欠采样，系数方向反了 */
    printf("direction_ok=%d        (斜面清晰度高于平面则系数方向反了)\n", sa <= pa * 1.05);
    printf("recovered=%d           (修正后斜面清晰度应明显回升)\n", sa > si * 1.15);
    printf("reversed_detected=%d   (反向系数应被同一自检判出)\n", sr > pr * 1.05);

    write_pgm("../artifacts/aniso/scene_iso.pgm",      img_iso, box_full);
    write_pgm("../artifacts/aniso/scene_aniso.pgm",    img_an,  box_full);
    write_pgm("../artifacts/aniso/scene_reversed.pgm", img_rv, box_full);
    write_pgm("../artifacts/aniso/scene_ref.pgm",      img_ref, box_full);
    write_pgm("../artifacts/aniso/crop_plane_iso.pgm",   img_iso, ba);
    write_pgm("../artifacts/aniso/crop_slope_iso.pgm",   img_iso, bb);
    write_pgm("../artifacts/aniso/crop_plane_aniso.pgm", img_an,  ba);
    write_pgm("../artifacts/aniso/crop_slope_aniso.pgm", img_an,  bb);
    printf("screenshots -> ../artifacts/aniso/\n");
    return 0;
}
