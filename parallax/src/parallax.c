
/* parallax-pom: 视差遮蔽映射（steep parallax + 层间线性过渡），
   层数按入射角与深度范围自适应；关闭自适应后与旧的固定层渲染逐像素一致。 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define W 96
#define H 64
#define REF_LAYERS 4096     /* 高精度参考渲染的层数 */
#define REF_SCALE  0.1f     /* 自适应公式的参考深度范围 */
#define SCENE_SCALE 0.25f   /* 测试场景的深度范围 */
#define TEAR_TOL   1e-4f    /* 与参考深度差超过此值记为一个断层像素 */

typedef struct { float x, y, z; } Vec3;

typedef struct {
    int min_layers;    /* 自适应下限，>0 */
    int max_layers;    /* 自适应上限，>= min_layers */
    int fixed_layers;  /* 关闭自适应时使用的固定层数，>0 */
    float depth_scale; /* 深度范围，必须为正且有限 */
    int adaptive;      /* 0 = 固定层（旧行为） */
} ParallaxCfg;

/* 参数校验：层数为零、上下限颠倒、深度范围非法一律拒绝 */
static int parallax_check(const ParallaxCfg* c) {
    if (c->min_layers <= 0 || c->max_layers <= 0 || c->fixed_layers <= 0) return -1;
    if (c->min_layers > c->max_layers) return -1;
    if (!(c->depth_scale > 0.0f) || !isfinite(c->depth_scale)) return -1;
    return 0;
}

/* 平滑高度场，值域约 [0.05, 0.95] */
static float heightf(float u, float v) {
    return 0.5f
        + 0.20f * sinf(6.2831853f * (1.5f * u + 0.5f * v))
        + 0.15f * cosf(6.2831853f * (1.0f * v - 0.5f * u))
        + 0.10f * sinf(6.2831853f * (2.5f * u + 1.5f * v));
}

/* 沿视线逐层递进采样，命中后在相邻两层间做线性过渡，返回归一化深度 [0,1]。
   内部用 double 累进：高层数参考渲染下 float 累加误差会淹没收敛性。 */
static float pom_depth(float u, float v, Vec3 view, int layers, float depth_scale) {
    double step = 1.0 / layers;
    /* 每层采样偏移：按层数均分总视差位移 */
    double du = (double)view.x / view.z * depth_scale / layers;
    double dv = (double)view.y / view.z * depth_scale / layers;
    double cu = u, cv = v, cd = 0.0;
    double h = heightf((float)cu, (float)cv);
    while (cd < h) {                 /* 逐层递进，直到射线没入表面 */
        cu -= du; cv -= dv; cd += step;
        h = heightf((float)cu, (float)cv);
    }
    /* 层间线性过渡：在跨越表面的两层之间求交 */
    double pu = cu + du, pv = cv + dv;
    double pd = cd - step;
    double after  = cd - h;                     /* 当前层：射线在表面之下 */
    double before = pd - heightf((float)pu, (float)pv); /* 上一层：射线在表面之上 */
    double t = before / (before - after);
    return (float)(pd + t * step);
}

/* 自适应层数：入射角越掠射、深度范围越大，层数越多 */
static int adaptive_layers(Vec3 view, float depth_scale, const ParallaxCfg* c) {
    float grazing = 1.0f - view.z;              /* 0 正视 .. 1 掠射 */
    float range = depth_scale / REF_SCALE;      /* 深度范围因子 */
    float f = grazing * range;
    if (f > 1.0f) f = 1.0f;
    int n = c->min_layers + (int)((float)(c->max_layers - c->min_layers) * f);
    if (n < c->min_layers) n = c->min_layers;
    if (n > c->max_layers) n = c->max_layers;
    return n;
}

static Vec3 pixel_view(int x, int y) {
    float inc = 1.0f * (float)x / (float)(W - 1);        /* 0..~57° 入射角扫描 */
    float az  = 6.2831853f * (float)y / (float)H;        /* 方位角随行变化 */
    Vec3 v = { sinf(inc) * cosf(az), sinf(inc) * sinf(az), cosf(inc) };
    return v;
}

/* 旧版固定层渲染（基线，不得改动行为） */
static void render_legacy(float* out, int fixed_layers, float depth_scale) {
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            float u = 2.0f * (float)x / (float)W, v = 2.0f * (float)y / (float)H;
            out[y * W + x] = pom_depth(u, v, pixel_view(x, y), fixed_layers, depth_scale);
        }
}

static int render(float* out, const ParallaxCfg* c, int* layers_used) {
    if (parallax_check(c) != 0) return -1;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            Vec3 view = pixel_view(x, y);
            int layers = c->adaptive ? adaptive_layers(view, c->depth_scale, c)
                                     : c->fixed_layers;
            if (layers_used) {
                if (layers < layers_used[0]) layers_used[0] = layers;
                if (layers > layers_used[1]) layers_used[1] = layers;
            }
            float u = 2.0f * (float)x / (float)W, v = 2.0f * (float)y / (float)H;
            out[y * W + x] = pom_depth(u, v, view, layers, c->depth_scale);
        }
    return 0;
}

/* 断层像素数：与高精度参考的深度差超过阈值 */
static int tear_pixels(const float* img, const float* ref) {
    int n = 0;
    for (int i = 0; i < W * H; i++)
        if (fabsf(img[i] - ref[i]) > TEAR_TOL) n++;
    return n;
}

int main(void) {
    static float ref[W * H], img[W * H], img2[W * H];
    int fails = 0;

    /* 1. 非法参数必须拒绝 */
    ParallaxCfg good = { 8, 256, 32, SCENE_SCALE, 1 };
    ParallaxCfg bad[] = {
        { 0, 256, 32, SCENE_SCALE, 1 },   /* 层数为零 */
        { 8, 256, 0,  SCENE_SCALE, 0 },   /* 固定层数为零 */
        { 64, 16, 32, SCENE_SCALE, 1 },   /* 上下限颠倒 */
        { 8, 256, 32, 0.0f, 1 },          /* 深度范围为零 */
        { 8, 256, 32, -0.5f, 1 },         /* 深度范围为负 */
        { 8, 256, 32, NAN,  1 },          /* 深度范围非有限 */
    };
    int reject_ok = 1;
    for (int i = 0; i < 6; i++)
        if (render(img, &bad[i], NULL) == 0) reject_ok = 0;
    if (render(img, &good, NULL) != 0) reject_ok = 0;
    printf("reject_invalid=%d\n", reject_ok);
    fails += !reject_ok;

    /* 2. 高精度参考渲染 */
    render_legacy(ref, REF_LAYERS, SCENE_SCALE);

    /* 3. 层数翻倍，断层像素数必须单调下降 */
    int seq[] = { 8, 16, 32, 64, 128, 256 };
    int prev = -1, mono = 1;
    printf("tears:");
    for (int i = 0; i < 6; i++) {
        render_legacy(img, seq[i], SCENE_SCALE);
        int t = tear_pixels(img, ref);
        printf(" L%d=%d", seq[i], t);
        if (prev >= 0 && t >= prev) mono = 0;
        prev = t;
    }
    printf("\nmonotonic_decrease=%d\n", mono);
    fails += !mono;

    /* 4. 关闭自适应后必须与旧版固定层渲染逐像素一致 */
    ParallaxCfg off = { 8, 256, 32, SCENE_SCALE, 0 };
    render(img, &off, NULL);
    render_legacy(img2, 32, SCENE_SCALE);
    int same = memcmp(img, img2, sizeof(img)) == 0;
    printf("adaptive_off_identical=%d\n", same);
    fails += !same;

    /* 5. 自适应开启：层数落在上下限内，且断层数不多于固定最低层 */
    int used[2] = { 1 << 30, 0 };
    render(img, &good, used);
    render_legacy(img2, good.min_layers, SCENE_SCALE);
    int t_adapt = tear_pixels(img, ref), t_min = tear_pixels(img2, ref);
    int adapt_ok = used[0] >= good.min_layers && used[1] <= good.max_layers
                && used[1] > used[0] && t_adapt <= t_min;
    printf("adaptive_layers=[%d..%d] tears=%d (fixed_L%d=%d) ok=%d\n",
           used[0], used[1], t_adapt, good.min_layers, t_min, adapt_ok);
    fails += !adapt_ok;

    printf("%s\n", fails ? "FAIL" : "OK");
    return fails ? 1 : 0;
}
