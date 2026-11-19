
#define _POSIX_C_SOURCE 199309L   /* clock_gettime under -std=c11 */

/* moveval: server-authoritative movement validation.
   Each sample's reachable region is a circle of radius max_speed * dt around the
   last authoritative position. Samples outside it are rejected as illegal and
   clamped onto the circle; the client's claimed coordinates never enter the
   authoritative state. Packet-rate anomalies are flagged separately and never
   block a reachable move, so an honest client cannot be false-rejected.
   Lookup is O(1) via a server-assigned slot id, hence a batch of N units is O(N). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define MV_MAX_UNITS  4096
#define MV_EPS        1e-6f    /* absolute slack for near-zero movement */
#define MV_TOL        0.01f    /* relative slack: 1% of reach, absorbs client
                                  float/quantization drift at any speed scale */
#define MV_EWMA_ALPHA 0.2f     /* inter-packet interval smoothing */

/* result bit flags */
#define MV_OK           0
#define MV_ILLEGAL      (1 << 0)  /* out of reach, clamped to the circle */
#define MV_FREQ_FAST    (1 << 1)  /* packets arriving too quickly (replay/burst) */
#define MV_FREQ_GAP     (1 << 2)  /* unusually long gap vs smoothed interval */
#define MV_UNKNOWN_UNIT (1 << 3)  /* id not registered */

typedef struct {
    int    used;
    int    id;
    float  x, y;             /* last authoritative position */
    float  max_speed;        /* world units per second */
    double last_time;        /* server time of the last applied sample */
    int    have_anchor;      /* first sample already trusted */
    float  interval_ewma;    /* smoothed inter-packet interval, seconds */
    int    have_interval;
    int    illegal_count;    /* clamped samples */
    int    freq_fast_count;
    int    freq_gap_count;
} Unit;

typedef struct {
    Unit  units[MV_MAX_UNITS];
    int   count;
    int   high_water;
    float min_interval;      /* nominal packet cadence, e.g. 0.05 (20 Hz) */
    float fast_tol;          /* flag FAST below min_interval * fast_tol */
    float gap_mult;          /* flag GAP above interval_ewma * gap_mult */
} MvWorld;

typedef struct { int id; double t; float x, y; } MvSample;
typedef struct { int flags; float x, y; }        MvResult;

static void mv_init(MvWorld* w, float min_interval, float fast_tol, float gap_mult) {
    memset(w, 0, sizeof(*w));
    w->min_interval = min_interval;
    w->fast_tol     = fast_tol;
    w->gap_mult     = gap_mult;
}

/* id == slot index: this is what keeps per-sample validation O(1). */
static int mv_spawn(MvWorld* w, float x, float y, float max_speed) {
    for (int i = 0; i < MV_MAX_UNITS; i++) {
        if (!w->units[i].used) {
            Unit* u = &w->units[i];
            memset(u, 0, sizeof(*u));
            u->used = 1; u->id = i;
            u->x = x; u->y = y; u->max_speed = max_speed;
            w->count++;
            if (w->count > w->high_water) w->high_water = w->count;
            return i;
        }
    }
    return -1;
}

static void mv_despawn(MvWorld* w, int id) {
    if (id >= 0 && id < MV_MAX_UNITS && w->units[id].used) {
        memset(&w->units[id], 0, sizeof(Unit));
        w->count--;
    }
}

/* Validate one claimed position. The corrected authoritative position is always
   written through ox/oy. Returns MV_OK or a combination of flag bits. */
static int mv_validate(MvWorld* w, int id, double now, float cx, float cy,
                       float* ox, float* oy) {
    if (id < 0 || id >= MV_MAX_UNITS || !w->units[id].used) {
        if (ox) *ox = 0.0f;
        if (oy) *oy = 0.0f;
        return MV_UNKNOWN_UNIT;
    }
    Unit* u = &w->units[id];
    int flags = MV_OK;

    /* non-finite coordinates cannot describe a legal position: drop outright. */
    if (!isfinite(cx) || !isfinite(cy)) {
        u->illegal_count++;
        if (ox) *ox = u->x;
        if (oy) *oy = u->y;
        return MV_ILLEGAL;
    }

    /* first sample after spawn is the trust anchor */
    if (!u->have_anchor) {
        u->have_anchor = 1;
        u->last_time   = now;
        if (ox) *ox = cx;
        if (oy) *oy = cy;
        return MV_OK;
    }

    double dt = now - u->last_time;

    /* ---- frequency checks: advisory only, never affect accept/clamp ---- */
    if (dt <= 0.0) {
        flags |= MV_FREQ_FAST;              /* replay / reorder / zero interval */
        u->freq_fast_count++;
    } else {
        float fdt = (float)dt;
        if (fdt < w->min_interval * w->fast_tol) {
            flags |= MV_FREQ_FAST;
            u->freq_fast_count++;
        }
        if (u->have_interval && fdt > u->interval_ewma * w->gap_mult) {
            flags |= MV_FREQ_GAP;
            u->freq_gap_count++;
        }
        if (!u->have_interval) {
            u->interval_ewma = fdt;
            u->have_interval = 1;
        } else {
            u->interval_ewma += MV_EWMA_ALPHA * (fdt - u->interval_ewma);
        }
    }

    /* ---- reachability: radius = allowed speed * elapsed time ---- */
    float reach = dt > 0.0 ? u->max_speed * (float)dt : 0.0f;
    float dx = cx - u->x;
    float dy = cy - u->y;
    float dist = hypotf(dx, dy);

    /* relative tolerance: honest clients at the speed limit (with float/network
       quantization drift) never trip; a teleport still overshoots by orders of
       magnitude and gets clamped */
    float limit = reach * (1.0f + MV_TOL) + MV_EPS;
    if (dist <= limit) {
        /* legal: pass through untouched (inclusive bound => zero false reject) */
        u->x = cx;
        u->y = cy;
    } else {
        /* illegal: project onto the reach circle, toward the claimed point */
        float s = reach / dist;             /* dist > reach >= 0, safe divide */
        u->x += dx * s;
        u->y += dy * s;
        u->illegal_count++;
        flags |= MV_ILLEGAL;
    }

    if (now > u->last_time) u->last_time = now;  /* replays must not rewind time */
    if (ox) *ox = u->x;
    if (oy) *oy = u->y;
    return flags;
}

/* O(N): one fixed-cost validation per sample, no cross-unit scans. */
static void mv_validate_batch(MvWorld* w, const MvSample* in, MvResult* out, int n) {
    for (int i = 0; i < n; i++) {
        out[i].flags = mv_validate(w, in[i].id, in[i].t, in[i].x, in[i].y,
                                   &out[i].x, &out[i].y);
    }
}

/* ------------------------------ self-test ------------------------------ */

static int g_checks, g_failed;
#define CHECK(cond, name) do {                                   \
    g_checks++;                                                   \
    if (cond) { printf("ok   %s\n", name); }                     \
    else      { g_failed++; printf("FAIL %s\n", name); }         \
} while (0)

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

int main(void) {
    MvWorld w;
    mv_init(&w, 0.05f /*20Hz*/, 0.5f /*<25ms 视为过快*/, 4.0f /*间隔 >4x EWMA 视为缺口*/);
    float ox, oy;
    int f;

    /* 1) 合法移动零误判：限速 10，以 5..9 的速度换方向走 50 步。
          客户端在自己的 float 时间戳上积分并随包上报，服务端用同一时间戳，
          与真实部署一致 => 无系统性漂移 */
    int a = mv_spawn(&w, 0.0f, 0.0f, 10.0f);
    int false_pos = 0, altered = 0;
    float ft = 1.0f, fdt = 0.1f;
    float px = 0.0f, py = 0.0f;
    mv_validate(&w, a, (double)ft, px, py, &ox, &oy);
    for (int i = 0; i < 50; i++) {
        ft += fdt;
        float speed = 5.0f + (float)(i % 5);            /* 5..9, 始终 <= 10 */
        float ang = (float)i * 0.7f;
        float nx = px + cosf(ang) * speed * fdt;
        float ny = py + sinf(ang) * speed * fdt;
        f = mv_validate(&w, a, (double)ft, nx, ny, &ox, &oy);
        if (f & MV_ILLEGAL) false_pos++;
        if (ox != nx || oy != ny) altered++;             /* 合法样本必须原样放行 */
        px = nx; py = ny;
    }
    CHECK(false_pos == 0, "legal moves never flagged illegal");
    CHECK(altered == 0, "legal moves passed through untouched");

    /* 1b) 公差带：边界内 0.5% 的时钟/量化抖动放行，2 倍越界仍拦截 */
    int aj = mv_spawn(&w, 0.0f, 0.0f, 10.0f);
    mv_validate(&w, aj, 1.0, 0.0f, 0.0f, &ox, &oy);
    f = mv_validate(&w, aj, 1.1, 1.005f, 0.0f, &ox, &oy);   /* 超 0.5%，在 1% 公差内 */
    CHECK(f == MV_OK && ox == 1.005f, "sub-tolerance boundary jitter accepted");
    f = mv_validate(&w, aj, 1.2, 3.0f, 0.0f, &ox, &oy);     /* 声称走 2，reach 仅 1 */
    CHECK((f & MV_ILLEGAL) && fabsf(ox - 2.005f) < 1e-3f, "overshoot beyond tolerance clamped");

    /* 2) 恰好等于限速（dist == reach）也必须放行，公差带吸收浮点误差 */
    int b = mv_spawn(&w, 0.0f, 0.0f, 10.0f);
    mv_validate(&w, b, 1.0, 0.0f, 0.0f, &ox, &oy);
    f = mv_validate(&w, b, 1.1, 1.0f, 0.0f, &ox, &oy);  /* reach = 10*0.1 = 1 */
    CHECK(f == MV_OK, "move exactly at speed limit passes");

    /* 3) 瞬移：声称 (100,0)，判定非法并夹回到可达圆边界 (1,0) */
    int c = mv_spawn(&w, 0.0f, 0.0f, 10.0f);
    mv_validate(&w, c, 1.0, 0.0f, 0.0f, &ox, &oy);
    f = mv_validate(&w, c, 1.1, 100.0f, 0.0f, &ox, &oy);
    CHECK((f & MV_ILLEGAL) != 0, "teleport flagged illegal");
    CHECK(fabsf(ox - 1.0f) < 1e-4f && oy == 0.0f, "teleport clamped to reach radius");

    /* 4) 连续瞬移：首包夹回后再 9 包，每包最多推进一个 reach，目标永远到不了 */
    for (int i = 0; i < 9; i++)
        f = mv_validate(&w, c, 1.2 + 0.1 * i, 100.0f, 0.0f, &ox, &oy);
    CHECK(w.units[c].illegal_count == 10, "every illegal sample intercepted");
    CHECK(fabsf(ox - 10.0f) < 1e-3f && ox < 11.0f, "repeated teleport capped at max speed");

    /* 5) 夹回点落在朝目标的线段上：目标 (3,4) 距离 5，夹回 = 0.2*(3,4) */
    int d = mv_spawn(&w, 0.0f, 0.0f, 10.0f);
    mv_validate(&w, d, 1.0, 0.0f, 0.0f, &ox, &oy);
    mv_validate(&w, d, 1.1, 3.0f, 4.0f, &ox, &oy);     /* reach=1, dist=5 */
    CHECK(fabsf(ox - 0.6f) < 1e-4f && fabsf(oy - 0.8f) < 1e-4f,
          "clamp point lies on segment toward target");

    /* 6) 频率过快只标记：dt=1ms 的爆发包，移动在可达内则原样生效 */
    int e = mv_spawn(&w, 0.0f, 0.0f, 10.0f);
    mv_validate(&w, e, 1.0, 0.0f, 0.0f, &ox, &oy);
    int fast_seen = 0, fast_blocked = 0;
    for (int i = 0; i < 10; i++) {
        float nx = (float)(i + 1) * 0.005f;             /* 每包 0.005，reach=0.01 */
        f = mv_validate(&w, e, 1.0 + 0.001 * (i + 1), nx, 0.0f, &ox, &oy);
        if (f & MV_FREQ_FAST) fast_seen++;
        if ((f & MV_ILLEGAL) || ox != nx) fast_blocked++;
    }
    CHECK(fast_seen == 10, "burst packets flagged FREQ_FAST");
    CHECK(fast_blocked == 0, "frequency flag never blocks a reachable move");

    /* 7) 异常长间隔只标记：EWMA≈0.1 后突然 dt=1.0，合法移动仍放行 */
    int g = mv_spawn(&w, 0.0f, 0.0f, 10.0f);
    mv_validate(&w, g, 1.0, 0.0f, 0.0f, &ox, &oy);
    for (int i = 0; i < 5; i++)
        mv_validate(&w, g, 1.1 + 0.1 * i, 0.0f, 0.0f, &ox, &oy);
    f = mv_validate(&w, g, 2.5, 1.0f, 0.0f, &ox, &oy);  /* dt=1.0 > 4*EWMA */
    CHECK((f & MV_FREQ_GAP) != 0 && !(f & MV_ILLEGAL), "long gap flagged but move accepted");
    CHECK(ox == 1.0f, "gap-flagged move applied verbatim");

    /* 8) 重放包（dt<=0）带位移：拦截夹回，且服务端时间不被回拨 */
    int h = mv_spawn(&w, 0.0f, 0.0f, 10.0f);
    mv_validate(&w, h, 1.0, 0.0f, 0.0f, &ox, &oy);
    f = mv_validate(&w, h, 1.0, 5.0f, 0.0f, &ox, &oy);
    CHECK((f & MV_ILLEGAL) && (f & MV_FREQ_FAST), "replay with movement intercepted");
    CHECK(ox == 0.0f && w.units[h].last_time == 1.0, "replay clamps position and keeps time");
    f = mv_validate(&w, h, 1.1, 1.0f, 0.0f, &ox, &oy);  /* 后续正常包按 dt=0.1 判定 */
    CHECK(f == MV_OK && ox == 1.0f, "normal packet after replay works");

    /* 9) NaN/Inf 坐标直接拦截，权威状态不动 */
    int k = mv_spawn(&w, 2.0f, 3.0f, 10.0f);
    mv_validate(&w, k, 1.0, 2.0f, 3.0f, &ox, &oy);
    f = mv_validate(&w, k, 1.1, NAN, INFINITY, &ox, &oy);
    CHECK((f & MV_ILLEGAL) && ox == 2.0f && oy == 3.0f, "non-finite input dropped");

    /* 10) 未知 id 与下线单位 */
    f = mv_validate(&w, 99999, 1.0, 0.0f, 0.0f, &ox, &oy);
    CHECK(f == MV_UNKNOWN_UNIT, "unknown unit rejected");
    mv_despawn(&w, k);
    f = mv_validate(&w, k, 1.2, 0.0f, 0.0f, &ox, &oy);
    CHECK(f == MV_UNKNOWN_UNIT, "despawned slot rejected");

    /* 11) 批量接口 O(N)：每单位固定开销，ns/sample 不随 N 量级抬升 */
    const int sizes[] = { 512, 1024, 2048, 4096 };
    const int rounds = 200;
    printf("bench (rounds=%d per unit):\n", rounds);
    for (size_t si = 0; si < sizeof(sizes) / sizeof(sizes[0]); si++) {
        int n = sizes[si];
        MvWorld bw;
        mv_init(&bw, 0.05f, 0.5f, 4.0f);
        for (int i = 0; i < n; i++) mv_spawn(&bw, 0.0f, 0.0f, 10.0f);
        MvSample* in = (MvSample*)malloc(sizeof(MvSample) * (size_t)n);
        MvResult* out = (MvResult*)malloc(sizeof(MvResult) * (size_t)n);
        for (int i = 0; i < n; i++) { in[i].id = i; in[i].x = 0.0f; in[i].y = 0.0f; }
        double t0 = now_sec();
        for (int r = 0; r < rounds; r++) {
            for (int i = 0; i < n; i++) {
                in[i].t = 1.0 + 0.1 * (r + 1);
                in[i].x = 0.5f * (float)(r + 1);           /* 每轮 0.5，reach=1，恒合法 */
            }
            mv_validate_batch(&bw, in, out, n);
        }
        double el = now_sec() - t0;
        printf("  N=%d  total=%.3fms  %.1f ns/sample  high_water=%d\n",
               n, el * 1e3, el * 1e9 / ((double)n * rounds), bw.high_water);
        CHECK(bw.high_water == n, "batch tracked all units");
        free(in);
        free(out);
    }

    int total_illegal = 0, total_fast = 0, total_gap = 0;
    for (int i = 0; i < MV_MAX_UNITS; i++) if (w.units[i].used) {
        total_illegal += w.units[i].illegal_count;
        total_fast    += w.units[i].freq_fast_count;
        total_gap     += w.units[i].freq_gap_count;
    }
    printf("counters: illegal=%d freq_fast=%d freq_gap=%d (线性汇总 O(N))\n",
           total_illegal, total_fast, total_gap);
    printf("checks=%d failed=%d\n", g_checks, g_failed);
    return g_failed ? 1 : 0;
}
