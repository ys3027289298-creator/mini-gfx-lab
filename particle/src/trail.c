
/* trail-history: per-particle ring-buffered position history for trail
   rendering.

   设计目标:
   - 每个粒子固定长度的位置环, 按帧写入, 环写满后掩码回绕覆盖最旧槽;
   - 历史长度可配(2 的幂, 上限编译期常量), 存储在 init 时一次性确定,
     本文件不使用 malloc, 帧路径上没有任何分配;
   - 渲染读取按年龄/段数取最近若干段, 提供两种视图:
       trail_spans(): 零拷贝, 最多返回两段物理连续区间(回绕必然 <=2 段);
       trail_read() : 拷贝进调用方缓冲, 输出逻辑连续(最旧 -> 最新),
                      全程最多两次 memcpy;
   - 一万粒子各存十六段时, 单帧最坏拷贝 10000*16*12 = 1.92 MB. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <string.h>
#include <time.h>

#define TRAIL_MAX_PARTICLES 10000
#define TRAIL_HIST_MAX 16

typedef struct { float x, y, age; } TrailSample;

typedef struct { const TrailSample* data; int len; } TrailSpan;

typedef struct {
    /* 扁平环: 粒子 p 的第 i 槽位于 ring[p * hist + (i & mask)] */
    TrailSample ring[TRAIL_MAX_PARTICLES * TRAIL_HIST_MAX];
    int head[TRAIL_MAX_PARTICLES];   /* 下一个写入槽位 */
    int count[TRAIL_MAX_PARTICLES];  /* 当前有效样本数 <= hist */
    int n_particles;
    int hist;
    int mask;
} TrailHistory;

/* 文件级静态存储: 唯一一次"分配"是 BSS 装载, 运行期零分配 */
static TrailHistory g_hist;
static TrailSample g_copy_out[TRAIL_MAX_PARTICLES * TRAIL_HIST_MAX];

static int is_pow2(int v) { return v > 0 && (v & (v - 1)) == 0; }

static int trail_init(TrailHistory* h, int n_particles, int hist) {
    if (n_particles <= 0 || n_particles > TRAIL_MAX_PARTICLES) return -1;
    if (!is_pow2(hist) || hist > TRAIL_HIST_MAX) return -1;
    memset(h, 0, sizeof(*h));
    h->n_particles = n_particles;
    h->hist = hist;
    h->mask = hist - 1;
    return 0;
}

/* 粒子死亡/槽位复用时清空它的环 */
static void trail_clear(TrailHistory* h, int p) {
    h->head[p] = 0;
    h->count[p] = 0;
}

/* 写入一帧位置: 回绕即覆盖最旧样本, 计数封顶在 hist */
static void trail_push(TrailHistory* h, int p, float x, float y, float age) {
    int slot = h->head[p];
    TrailSample* s = &h->ring[(long)p * h->hist + slot];
    s->x = x; s->y = y; s->age = age;
    h->head[p] = (slot + 1) & h->mask;
    if (h->count[p] < h->hist) h->count[p]++;
}

/* 计算满足年龄/段数限制后, 实际保留的样本数与逻辑起始槽。
   样本沿时间递增写入, 故从最新槽向回走即可截断过旧样本。 */
static int trail_resolve(const TrailHistory* h, int p,
                         float max_age, int max_segments, int* out_start) {
    int c = h->count[p];
    if (c == 0) { *out_start = 0; return 0; }
    int n = c < max_segments ? c : max_segments;
    int newest = (h->head[p] - 1) & h->mask;
    const TrailSample* s_new = &h->ring[(long)p * h->hist + newest];
    if (max_age >= 0.0f) {
        float cutoff = s_new->age - max_age;
        /* 向回数, 遇到年龄低于 cutoff 的更旧样本即停 */
        int kept = 0;
        for (int k = 0; k < n; k++) {
            int slot = (newest - k) & h->mask;
            if (h->ring[(long)p * h->hist + slot].age < cutoff) break;
            kept++;
        }
        n = kept;
    }
    if (n == 0) { *out_start = 0; return 0; }
    *out_start = (h->head[p] - n) & h->mask;
    return n;
}

/* 零拷贝视图: 最多两段物理连续区间, spans[0] 更旧, 顺序拼接即最旧 -> 最新 */
static int trail_spans(const TrailHistory* h, int p, float max_age,
                       int max_segments, TrailSpan spans[2]) {
    int start;
    int n = trail_resolve(h, p, max_age, max_segments, &start);
    if (n == 0) return 0;
    const TrailSample* base = &h->ring[(long)p * h->hist];
    int first = n < h->hist - start ? n : h->hist - start;
    spans[0].data = base + start;
    spans[0].len = first;
    if (n == first) return 1;
    spans[1].data = base;
    spans[1].len = n - first;
    return 2;
}

/* 连续拷贝视图: 结果按最旧 -> 最新写入 out, 最多两次 memcpy, 返回样本数 */
static int trail_read(const TrailHistory* h, int p, float max_age,
                      int max_segments, TrailSample* out) {
    TrailSpan spans[2];
    int nspans = trail_spans(h, p, max_age, max_segments, spans);
    int written = 0;
    for (int i = 0; i < nspans; i++) {
        memcpy(out + written, spans[i].data,
               (size_t)spans[i].len * sizeof(TrailSample));
        written += spans[i].len;
    }
    return written;
}

/* ---------------- 自测 ---------------- */

static int g_fails = 0;

#define CHECK(cond, msg) do {                                         \
    if (cond) { printf("  PASS %s\n", msg); }                        \
    else { printf("  FAIL %s\n", msg); g_fails++; }                  \
} while (0)

static double median(double* a, int n) {
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++)
            if (a[j] < a[i]) { double t = a[i]; a[i] = a[j]; a[j] = t; }
    return a[n / 2];
}

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void test_fill_and_overwrite(void) {
    printf("[fill + ring overwrite]\n");
    TrailHistory h;
    trail_init(&h, 4, 16);
    for (int i = 0; i < 20; i++)
        trail_push(&h, 0, (float)i, (float)(i * 2), (float)i);
    CHECK(h.count[0] == 16, "count capped at hist after overflow");
    CHECK(h.head[0] == 4, "head wraps to next write slot");

    TrailSample out[16];
    int n = trail_read(&h, 0, -1.0f, 16, out);
    CHECK(n == 16, "read returns full ring");
    int ordered = 1;
    for (int i = 0; i < 16; i++)
        if (out[i].x != (float)(i + 4) || out[i].y != (float)((i + 4) * 2)
            || out[i].age != (float)(i + 4)) ordered = 0;
    CHECK(ordered, "oldest kept sample is push #4, order oldest->newest");

    /* 回绕后必须拆成两段, 且拼接结果与拷贝视图一致 */
    TrailSpan spans[2];
    int ns = trail_spans(&h, 0, -1.0f, 16, spans);
    CHECK(ns == 2, "wrapped ring exposed as exactly two spans");
    int concat_ok = 1;
    int k = 0;
    for (int i = 0; i < ns; i++)
        for (int j = 0; j < spans[i].len; j++) {
            if (spans[i].data[j].age != out[k].age) concat_ok = 0;
            k++;
        }
    CHECK(concat_ok && k == 16, "spans concatenate to same ordered data");
}

static void test_no_wrap_single_span(void) {
    printf("[partial fill stays one span]\n");
    TrailHistory h;
    trail_init(&h, 4, 16);
    for (int i = 0; i < 3; i++) trail_push(&h, 1, i, 0, i);
    TrailSpan spans[2];
    int ns = trail_spans(&h, 1, -1.0f, 16, spans);
    CHECK(ns == 1 && spans[0].len == 3 && spans[0].data[0].age == 0.0f,
          "three pushes remain one contiguous span");
    CHECK(trail_read(&h, 1, -1.0f, 16, g_copy_out) == 3, "read count matches");
}

static void test_age_and_segment_filter(void) {
    printf("[age / segment filter]\n");
    TrailHistory h;
    trail_init(&h, 4, 16);
    for (int i = 0; i < 20; i++) trail_push(&h, 0, i, 0, (float)i);
    /* 最新 age=19, cutoff=19-5.5=13.5, 保留 age 14..19 共 6 个 */
    TrailSample out[16];
    int n = trail_read(&h, 0, 5.5f, 16, out);
    CHECK(n == 6 && out[0].age == 14.0f && out[5].age == 19.0f,
          "max_age trims oldest samples by age");

    /* 只要最近 4 段, 且年龄窗口比需求更宽 */
    n = trail_read(&h, 0, 100.0f, 4, out);
    CHECK(n == 4 && out[0].age == 16.0f && out[3].age == 19.0f,
          "max_segments takes most recent N");

    /* 年龄窗口收紧到只剩 1 个 */
    n = trail_read(&h, 0, 0.0f, 16, out);
    CHECK(n == 1 && out[0].age == 19.0f, "zero window keeps only newest");

    /* 空环 */
    CHECK(trail_read(&h, 3, -1.0f, 16, out) == 0, "empty ring reads zero");
}

static void test_particle_independence_and_reuse(void) {
    printf("[per-particle independence + slot reuse]\n");
    TrailHistory h;
    trail_init(&h, 10000, 16);
    for (int i = 0; i < 30; i++) {
        trail_push(&h, 0, 1000.0f + i, 0, (float)i);
        trail_push(&h, 9999, 2000.0f + i, 0, (float)i);
    }
    TrailSample a[16], b[16];
    int na = trail_read(&h, 0, -1.0f, 16, a);
    int nb = trail_read(&h, 9999, -1.0f, 16, b);
    CHECK(na == 16 && nb == 16 && a[0].x == 1014.0f && b[0].x == 2014.0f,
          "first and last particle keep disjoint rings");

    trail_clear(&h, 0);
    CHECK(h.count[0] == 0 && trail_read(&h, 0, -1.0f, 16, a) == 0,
          "clear empties ring on particle death");
    for (int i = 0; i < 5; i++) trail_push(&h, 0, 7.0f, 0, (float)i);
    CHECK(trail_read(&h, 0, -1.0f, 16, a) == 5 && a[4].x == 7.0f,
          "respawned particle reuses ring from clean state");
}

static void test_config(void) {
    printf("[configurable history length]\n");
    TrailHistory h;
    CHECK(trail_init(&h, 10000, 16) == 0, "hist=16 accepted");
    CHECK(trail_init(&h, 10000, 8) == 0, "hist=8 accepted");
    CHECK(trail_init(&h, 10000, 12) == -1, "non-power-of-two rejected");
    CHECK(trail_init(&h, 10000, 32) == -1, "hist over TRAIL_HIST_MAX rejected");
    CHECK(trail_init(&h, 10001, 16) == -1, "particle count over cap rejected");

    trail_init(&h, 10000, 8);
    for (int i = 0; i < 20; i++) trail_push(&h, 0, i, 0, (float)i);
    TrailSample out[8];
    int n = trail_read(&h, 0, -1.0f, 8, out);
    CHECK(n == 8 && out[0].age == 12.0f && out[7].age == 19.0f,
          "shorter configured ring overwrites oldest correctly");
}

static void benchmark(void) {
    printf("[benchmark 10000 particles x 16 segments]\n");
    TrailHistory* h = &g_hist;
    trail_init(h, TRAIL_MAX_PARTICLES, TRAIL_HIST_MAX);

    enum { FRAMES = 121, WARMUP = 21 };
    double push_us[FRAMES], copy_us[FRAMES];
    int got_full = 1;

    for (int frame = 0; frame < FRAMES; frame++) {
        double t0 = now_sec();
        for (int p = 0; p < TRAIL_MAX_PARTICLES; p++)
            trail_push(h, p, (float)frame, (float)p, (float)frame);
        double t1 = now_sec();
        for (int p = 0; p < TRAIL_MAX_PARTICLES; p++) {
            int n = trail_read(h, p, 1e9f, TRAIL_HIST_MAX,
                               g_copy_out + (long)p * TRAIL_HIST_MAX);
            if (frame >= TRAIL_HIST_MAX && n != TRAIL_HIST_MAX) got_full = 0;
        }
        double t2 = now_sec();
        push_us[frame] = (t1 - t0) * 1e6;
        copy_us[frame] = (t2 - t1) * 1e6;
    }

    double push_med = median(push_us + WARMUP, FRAMES - WARMUP);
    double copy_med = median(copy_us + WARMUP, FRAMES - WARMUP);
    double bytes = (double)TRAIL_MAX_PARTICLES * TRAIL_HIST_MAX
                   * sizeof(TrailSample);
    printf("  median push = %.1f us/frame\n", push_med);
    printf("  median copy = %.1f us/frame over %.2f MB (%.1f GB/s)\n",
           copy_med, bytes / 1e6, bytes / (copy_med * 1e-6) / 1e9);
    CHECK(got_full, "steady-state reads return all 16 segments");
    CHECK(copy_med < 1000.0, "single-frame copy under 1 ms");
}

int main(void) {
    test_fill_and_overwrite();
    test_no_wrap_single_span();
    test_age_and_segment_filter();
    test_particle_independence_and_reuse();
    test_config();
    benchmark();
    if (g_fails) { printf("\nFAILED: %d check(s)\n", g_fails); return 1; }
    printf("\nALL TESTS PASSED\n");
    return 0;
}
