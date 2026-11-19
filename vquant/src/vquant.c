/* vquant: 顶点量化压缩 —— 位置/法线按量化参数压成 16-bit 短整型。
   - 位置: 每轴映射到网格 AABB 的 [0,65535] (uint16)
   - 法线: [-1,1] 映射到 [-32767,32767] (int16), 解压后重新归一化
   - 解压误差有界: 位置每轴 <= step/2, 法线分量 <= 1/65534
   - 随机访问: decode_vertex() 按索引独立解压, 无需解码邻接顶点
   - 越界值夹取到量化域端点, 不溢出回绕
   自测: 软件光栅化原始浮点网格与量化解压网格, 逐像素差异须小于阈值。 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define POS_QMAX 65535.0f
#define NRM_QMAX 32767
#define PX_THRESHOLD 4 /* 逐像素允许最大差 (0..255) */

typedef struct { float pos[3]; float nrm[3]; } Vertex;       /* 24 B/顶点 */
typedef struct { uint16_t pos[3]; int16_t nrm[3]; } VertexQ; /* 12 B/顶点 */

typedef struct {
    float bmin[3];
    float step[3]; /* 每轴 (bmax-bmin)/65535; 退化轴为 0 */
} QuantParams;

/* ---------- 量化 / 解压 ---------- */

static uint16_t clamp_u16(float t) { /* 夹取, 不溢出 */
    if (!(t > 0.0f)) return 0;       /* 同时挡住 NaN */
    if (t >= POS_QMAX) return 65535;
    return (uint16_t)floorf(t + 0.5f);
}

static int16_t clamp_i16(float t) {
    if (!(t > -(float)NRM_QMAX)) return -NRM_QMAX;
    if (t >= (float)NRM_QMAX) return NRM_QMAX;
    return (int16_t)lroundf(t);
}

static void quant_params_init(QuantParams* q, const Vertex* v, int n) {
    float bmax[3];
    for (int a = 0; a < 3; a++) { q->bmin[a] = v[0].pos[a]; bmax[a] = v[0].pos[a]; }
    for (int i = 1; i < n; i++)
        for (int a = 0; a < 3; a++) {
            if (v[i].pos[a] < q->bmin[a]) q->bmin[a] = v[i].pos[a];
            if (v[i].pos[a] > bmax[a]) bmax[a] = v[i].pos[a];
        }
    for (int a = 0; a < 3; a++) {
        float ext = bmax[a] - q->bmin[a];
        q->step[a] = ext > 0.0f ? ext / POS_QMAX : 0.0f;
    }
}

static void quantize_vertex(const QuantParams* q, const Vertex* in, VertexQ* out) {
    for (int a = 0; a < 3; a++) {
        float t = q->step[a] > 0.0f ? (in->pos[a] - q->bmin[a]) / q->step[a] : 0.0f;
        out->pos[a] = clamp_u16(t);
        out->nrm[a] = clamp_i16(in->nrm[a] * (float)NRM_QMAX);
    }
}

/* 随机访问: 只读 buf[i], 不触碰其他顶点 */
static void decode_vertex(const QuantParams* q, const VertexQ* buf, int i, Vertex* out) {
    float len2 = 0.0f;
    for (int a = 0; a < 3; a++) {
        out->pos[a] = q->bmin[a] + (float)buf[i].pos[a] * q->step[a];
        out->nrm[a] = (float)buf[i].nrm[a] / (float)NRM_QMAX;
        len2 += out->nrm[a] * out->nrm[a];
    }
    if (len2 > 1e-12f) {
        float inv = 1.0f / sqrtf(len2);
        for (int a = 0; a < 3; a++) out->nrm[a] *= inv;
    }
}

/* ---------- 测试用网格: UV 球三角形汤 ---------- */

static Vertex* gen_sphere(int stacks, int slices, int* out_n) {
    int cap = stacks * slices * 2 * 3;
    Vertex* v = malloc((size_t)cap * sizeof(Vertex));
    int n = 0;
    for (int i = 0; i < stacks; i++) {
        for (int j = 0; j < slices; j++) {
            float t0 = (float)M_PI * i / stacks, t1 = (float)M_PI * (i + 1) / stacks;
            float p0 = 2.0f * (float)M_PI * j / slices, p1 = 2.0f * (float)M_PI * (j + 1) / slices;
            float th[4] = {t0, t1, t1, t0}, ph[4] = {p0, p0, p1, p1};
            Vertex c[4];
            for (int k = 0; k < 4; k++) {
                float s = sinf(th[k]);
                c[k].pos[0] = s * cosf(ph[k]);
                c[k].pos[1] = cosf(th[k]);
                c[k].pos[2] = s * sinf(ph[k]);
                memcpy(c[k].nrm, c[k].pos, sizeof c[k].nrm); /* 单位球: 法线=位置 */
            }
            int idx[6] = {0, 1, 2, 0, 2, 3};
            for (int k = 0; k < 6; k++) v[n++] = c[idx[k]];
        }
    }
    *out_n = n;
    return v;
}

/* ---------- 极简软件光栅化(透视校正 + Lambert) ---------- */

#define IMG_W 160
#define IMG_H 120

static void render(const Vertex* v, int n, uint8_t* img) {
    static float zbuf[IMG_W * IMG_H];
    memset(img, 0, IMG_W * IMG_H);
    for (int i = 0; i < IMG_W * IMG_H; i++) zbuf[i] = 1e9f;
    const float tanf2 = 0.5f; /* fov/2 的 tan, 取 0.5 */
    const float L[3] = {0.5773503f, 0.5773503f, 0.5773503f};
    for (int t = 0; t < n; t += 3) {
        float sx[3], sy[3], d[3];
        int behind = 0;
        for (int k = 0; k < 3; k++) {
            const float* p = v[t + k].pos;
            d[k] = 3.0f - p[2]; /* 相机在 (0,0,3) 朝 -z */
            if (d[k] < 0.1f) behind = 1;
            sx[k] = (p[0] / (d[k] * tanf2) * 0.5f + 0.5f) * IMG_W;
            sy[k] = (0.5f - p[1] / (d[k] * tanf2) * 0.5f) * IMG_H;
        }
        if (behind) continue;
        float x0 = sx[0], y0 = sy[0], x1 = sx[0], y1 = sy[0];
        for (int k = 1; k < 3; k++) {
            if (sx[k] < x0) x0 = sx[k];
            if (sx[k] > x1) x1 = sx[k];
            if (sy[k] < y0) y0 = sy[k];
            if (sy[k] > y1) y1 = sy[k];
        }
        int ix0 = (int)floorf(x0) - 1, ix1 = (int)ceilf(x1) + 1;
        int iy0 = (int)floorf(y0) - 1, iy1 = (int)ceilf(y1) + 1;
        if (ix0 < 0) ix0 = 0;
        if (iy0 < 0) iy0 = 0;
        if (ix1 >= IMG_W) ix1 = IMG_W - 1;
        if (iy1 >= IMG_H) iy1 = IMG_H - 1;
        float den = (sy[1] - sy[2]) * (sx[0] - sx[2]) + (sx[2] - sx[1]) * (sy[0] - sy[2]);
        if (fabsf(den) < 1e-9f) continue;
        for (int y = iy0; y <= iy1; y++) {
            for (int x = ix0; x <= ix1; x++) {
                float px = x + 0.5f, py = y + 0.5f;
                float w0 = ((sy[1] - sy[2]) * (px - sx[2]) + (sx[2] - sx[1]) * (py - sy[2])) / den;
                float w1 = ((sy[2] - sy[0]) * (px - sx[2]) + (sx[0] - sx[2]) * (py - sy[2])) / den;
                float w2 = 1.0f - w0 - w1;
                if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) continue;
                float q0 = w0 / d[0], q1 = w1 / d[1], q2 = w2 / d[2];
                float qs = q0 + q1 + q2;
                float z = 1.0f / qs;
                int pi = y * IMG_W + x;
                if (z >= zbuf[pi]) continue;
                float nrm[3] = {0, 0, 0};
                for (int a = 0; a < 3; a++)
                    nrm[a] = (q0 * v[t].nrm[a] + q1 * v[t + 1].nrm[a] + q2 * v[t + 2].nrm[a]) / qs;
                float nl = sqrtf(nrm[0] * nrm[0] + nrm[1] * nrm[1] + nrm[2] * nrm[2]);
                if (nl < 1e-9f) continue;
                float lam = 0.0f;
                for (int a = 0; a < 3; a++) lam += (nrm[a] / nl) * L[a];
                if (lam < 0.0f) lam = 0.0f;
                zbuf[pi] = z;
                img[pi] = (uint8_t)(40.0f + 215.0f * lam);
            }
        }
    }
}

/* ---------- 自测 ---------- */

int main(void) {
    int fails = 0;
    int n = 0;
    Vertex* src = gen_sphere(24, 48, &n);

    QuantParams q;
    quant_params_init(&q, src, n);

    VertexQ* packed = malloc((size_t)n * sizeof(VertexQ));
    for (int i = 0; i < n; i++) quantize_vertex(&q, &src[i], &packed[i]);

    /* 1) 解压误差有界 */
    float max_step = q.step[0];
    for (int a = 1; a < 3; a++) if (q.step[a] > max_step) max_step = q.step[a];
    float pos_bound = 0.5f * max_step + 1e-6f;
    float max_pos_err = 0.0f, max_nrm_ang = 0.0f;
    for (int i = 0; i < n; i++) {
        Vertex d;
        decode_vertex(&q, packed, i, &d);
        for (int a = 0; a < 3; a++) {
            float e = fabsf(d.pos[a] - src[i].pos[a]);
            if (e > max_pos_err) max_pos_err = e;
        }
        /* 小角度下 acos 会放大 float 舍入, 用 atan2(|perp|, dot) 双精度测角 */
        double dot = 0.0, perp2 = 0.0;
        for (int a = 0; a < 3; a++) dot += (double)d.nrm[a] * src[i].nrm[a];
        for (int a = 0; a < 3; a++) {
            double p = (double)d.nrm[a] - dot * src[i].nrm[a];
            perp2 += p * p;
        }
        float ang = (float)(atan2(sqrt(perp2), dot) * 180.0 / M_PI);
        if (ang > max_nrm_ang) max_nrm_ang = ang;
    }
    printf("pos_err=%.3e bound=%.3e ok=%d\n", max_pos_err, pos_bound, max_pos_err <= pos_bound);
    printf("nrm_ang_err=%.5fdeg ok=%d\n", max_nrm_ang, max_nrm_ang < 0.01f);
    fails += !(max_pos_err <= pos_bound) + !(max_nrm_ang < 0.01f);

    /* 2) 随机访问: 乱序索引独立解码, 误差同样有界 */
    unsigned rng = 12345;
    float ra_err = 0.0f;
    for (int k = 0; k < 4096; k++) {
        rng = rng * 1664525u + 1013904223u;
        int i = (int)(rng % (unsigned)n);
        Vertex d;
        decode_vertex(&q, packed, i, &d);
        for (int a = 0; a < 3; a++) {
            float e = fabsf(d.pos[a] - src[i].pos[a]);
            if (e > ra_err) ra_err = e;
        }
    }
    printf("random_access_err=%.3e ok=%d\n", ra_err, ra_err <= pos_bound);
    fails += !(ra_err <= pos_bound);

    /* 3) 越界夹取: 构造 AABB 外顶点, 应夹到端点而非溢出 */
    Vertex wild = {{q.bmin[0] - 100.0f * max_step * POS_QMAX,
                    q.bmin[1] + 1e9f,
                    q.bmin[2] + q.step[2] * POS_QMAX * 0.5f},
                   {5.0f, -5.0f, 0.0f}};
    VertexQ wq;
    quantize_vertex(&q, &wild, &wq);
    int clamp_ok = wq.pos[0] == 0 && wq.pos[1] == 65535 &&
                   wq.nrm[0] == NRM_QMAX && wq.nrm[1] == -NRM_QMAX;
    Vertex wd;
    decode_vertex(&q, &wq, 0, &wd);
    float bmax1 = q.bmin[1] + q.step[1] * POS_QMAX;
    clamp_ok = clamp_ok && wd.pos[1] <= bmax1 + 1e-6f && wd.pos[0] >= q.bmin[0] - 1e-6f;
    printf("clamp_ok=%d\n", clamp_ok);
    fails += !clamp_ok;

    /* 4) 渲染对比: 原始浮点 vs 量化解压, 逐像素差 < 阈值 */
    Vertex* dec = malloc((size_t)n * sizeof(Vertex));
    for (int i = 0; i < n; i++) decode_vertex(&q, packed, i, &dec[i]);
    static uint8_t img_a[IMG_W * IMG_H], img_b[IMG_W * IMG_H];
    render(src, n, img_a);
    render(dec, n, img_b);
    int max_diff = 0, over = 0;
    double mean = 0.0;
    for (int i = 0; i < IMG_W * IMG_H; i++) {
        int df = abs((int)img_a[i] - (int)img_b[i]);
        if (df > max_diff) max_diff = df;
        if (df > PX_THRESHOLD) over++;
        mean += df;
    }
    mean /= IMG_W * IMG_H;
    printf("pixel_max_diff=%d mean=%.4f threshold=%d over=%d ok=%d\n",
           max_diff, mean, PX_THRESHOLD, over, max_diff <= PX_THRESHOLD);
    fails += !(max_diff <= PX_THRESHOLD);

    /* 5) 压缩率 */
    size_t orig = (size_t)n * sizeof(Vertex);
    size_t comp = (size_t)n * sizeof(VertexQ) + sizeof(QuantParams);
    printf("verts=%d orig=%zuB compressed=%zuB ratio=%.2fx\n",
           n, orig, comp, (double)orig / (double)comp);

    printf("%s\n", fails ? "FAIL" : "PASS");
    free(src); free(packed); free(dec);
    return fails ? 1 : 0;
}
