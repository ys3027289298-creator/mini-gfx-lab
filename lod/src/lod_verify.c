
/* lod-verify: 选级质量对照实验（离线验证，不在渲染热路径上）。
   A. 启发式选级 vs 穷举 LEVELS^FRAMES 条级别序列的全局最优：逐帧比较，
      要求启发式总代价（渲染代价 + 切换代价）不高于最优的 1.2 倍。
   B. 相机抖动场景：统计启发式切换次数，要求不高于固定阈值的 70%。
   C. 单帧选级复杂度：操作计数随物体数线性增长（对照穷举的 LEVELS^N）。 */
#define LOD_LIB
#include "lod.c"
#include <string.h>

#define FRAMES 12
#define SWITCH_COST 150.0f          /* 一次切换的代价，与渲染代价同单位 */
#define COST_RATIO_LIMIT 1.2f

#define JITTER_FRAMES 200
#define JITTER_SWITCH_THRESHOLD 40  /* 固定阈值：抖动场景切换次数的基准上限 */

static int scenario_optimality(void) {
    MeshLod m = { {0.0f, 1.0f, 4.0f, 16.0f}, {1000.0f, 300.0f, 80.0f, 20.0f}, 2.0f, 1.1f };
    /* 后退镜头 + 一次回拉：21 和 82 附近故意落在迟滞带里，让启发式与最优产生分歧 */
    const float dist[FRAMES] = {9, 13, 17, 21, 24.5f, 20.8f, 26, 33, 42, 53, 66, 82};
    float fov = 1.0f, tol = 50.0f; int h = 1080;

    /* 启发式序列 */
    State st = {0, 0, 0};
    int hl[FRAMES]; float hcost = 0.0f;
    for (int i = 0; i < FRAMES; i++) {
        hl[i] = select_level(&m, dist[i], fov, h, tol, &st);
        hcost += m.cost[hl[i]] + (i > 0 && hl[i] != hl[i-1] ? SWITCH_COST : 0.0f);
    }

    /* 每帧每级的屏幕误差表，穷举时查表 */
    float se[FRAMES][LEVELS];
    for (int i = 0; i < FRAMES; i++)
        for (int l = 0; l < LEVELS; l++)
            se[i][l] = screen_error(m.error[l], dist[i], fov, h);

    /* 穷举所有 LEVELS^FRAMES 条级别序列：每帧误差必须 <= tol，取总代价最小者 */
    int lev[FRAMES] = {0}, best_lev[FRAMES] = {0};
    float best = 1e30f; long combos = 0;
    for (;;) {
        combos++;
        float c = 0.0f; int feasible = 1;
        for (int i = 0; i < FRAMES; i++) {
            if (se[i][lev[i]] > tol) { feasible = 0; break; }
            c += m.cost[lev[i]] + (i > 0 && lev[i] != lev[i-1] ? SWITCH_COST : 0.0f);
            if (c >= best) { feasible = 0; break; }   /* 已超当前最优，剪枝 */
        }
        if (feasible && c < best) { best = c; memcpy(best_lev, lev, sizeof lev); }
        int i = FRAMES - 1;                            /* 里程表式枚举下一条 */
        while (i >= 0 && ++lev[i] == LEVELS) { lev[i] = 0; i--; }
        if (i < 0) break;
    }

    printf("frame  dist  heuristic  optimal\n");
    for (int i = 0; i < FRAMES; i++)
        printf("%5d %6.1f %9d %8d\n", i, (double)dist[i], hl[i], best_lev[i]);
    float ratio = hcost / best;
    printf("optimality: heuristic=%.1f optimal=%.1f ratio=%.3f (穷举 %ld 条组合)\n",
           (double)hcost, (double)best, (double)ratio, combos);
    int ok = ratio <= COST_RATIO_LIMIT;
    printf("cost_ratio_ok=%d  (要求 <= %.1f)\n", ok, (double)COST_RATIO_LIMIT);
    return ok;
}

static int scenario_jitter(void) {
    MeshLod m   = { {0.0f, 1.0f, 4.0f, 16.0f}, {1000.0f, 300.0f, 80.0f, 20.0f}, 2.0f, 1.1f };
    MeshLod raw = m; raw.tau = 1.0f;   /* 无迟滞对照组 */
    State sh = {0, 0, 0}, sr = {0, 0, 0};
    float fov = 1.0f, tol = 50.0f; int h = 1080;
    for (int i = 0; i < JITTER_FRAMES; i++) {
        /* 在 0/1 级边界（d≈19.77）附近高频抖动，振幅小于迟滞带宽 */
        float d = 19.77f + 1.5f*sinf(i*0.8f) + 0.5f*sinf(i*0.13f);
        select_level(&m,   d, fov, h, tol, &sh);
        select_level(&raw, d, fov, h, tol, &sr);
    }
    int limit = (int)(JITTER_SWITCH_THRESHOLD * 0.7f);
    printf("jitter: switches=%d no_hysteresis=%d frames=%d\n",
           sh.switches, sr.switches, JITTER_FRAMES);
    int ok = sh.switches <= limit;
    printf("jitter_switches_ok=%d  (要求 <= 固定阈值%d 的 70%% = %d)\n",
           ok, JITTER_SWITCH_THRESHOLD, limit);
    return ok;
}

static int scenario_linear(void) {
    enum { M_MAX = 32 };
    static MeshLod ms[M_MAX]; static State ss[M_MAX]; static float dd[M_MAX];
    for (int k = 0; k < M_MAX; k++) {
        ms[k] = (MeshLod){ {0.0f, 1.0f + 0.05f*k, 4.0f, 16.0f},
                           {1000.0f, 300.0f, 80.0f, 20.0f}, 2.0f, 1.1f };
        ss[k] = (State){0, 0, 0};
        dd[k] = 12.0f + 2.3f*k;
    }
    float fov = 1.0f, tol = 50.0f; int h = 1080;
    const int Ms[3] = {8, 16, 32};
    long ops[3];
    for (int t = 0; t < 3; t++) {
        lod_select_ops = 0;
        for (int k = 0; k < Ms[t]; k++)
            select_level(&ms[k], dd[k], fov, h, tol, &ss[k]);
        ops[t] = lod_select_ops;
    }
    printf("linear: ops(M=8)=%ld ops(M=16)=%ld ops(M=32)=%ld (上界 LEVELS*M)\n",
           ops[0], ops[1], ops[2]);
    printf("  对照：穷举 M=32 个物体的组合数 = %.1e/帧\n", (double)pow(LEVELS, 32));
    /* 每个物体每帧的操作数有常数上界 LEVELS，且新增物体的增量同样线性 */
    int ok = ops[0] <= 8L*LEVELS && ops[1] <= 16L*LEVELS && ops[2] <= 32L*LEVELS
          && ops[1] - ops[0] <= 8L*LEVELS && ops[2] - ops[1] <= 16L*LEVELS;
    printf("linear_time_ok=%d\n", ok);
    return ok;
}

int main(void) {
    int ok = 1;
    ok &= scenario_optimality();
    ok &= scenario_jitter();
    ok &= scenario_linear();
    printf("verify_ok=%d\n", ok);
    return ok ? 0 : 1;
}
