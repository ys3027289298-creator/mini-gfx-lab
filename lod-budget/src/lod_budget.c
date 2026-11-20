
/* lod-budget: multi-frame global-optimal LOD selection under a per-frame
   resource budget.

   对照策略 greedy: 每帧独立决策，超预算就按 误差增量/节省成本 比逐个降级，
                    只看当前帧，遮挡一结束、预算一回升就立刻切回去。
   主策略     dp: 把一帧内所有物体的 LOD 组合枚举成状态，在未来 WINDOW 帧的
                    预测窗口上做动态规划；转移代价 = 质量误差 + 切换惩罚，
                    预算作为硬约束剪枝，天然跨帧平滑。滚动视界执行，只提交
                    窗口首帧；实际预算低于预测而超支时回退到贪心可行解。
   锚点   oracle: 全程已知实际预算的一次性 DP，给出真正的全局最优下界。 */
#include <stdio.h>
#include <string.h>

#define N_OBJ    5
#define LEVELS   4
#define FRAMES   48
#define WINDOW   10
#define CODES    1024              /* LEVELS^N_OBJ */
#define INF      1e30

#ifndef SWITCH_WEIGHT
#define SWITCH_WEIGHT 10.0         /* 单次单物体切换的代价 lambda，可编译期覆盖 */
#endif
#define FORECAST_ALPHA 0.5         /* 预算预测偏差的 EMA 系数 */
#define BUDGET_BASELINE 300.0      /* 名义预算，预测器向它均值回归 */

/* 每级渲染成本（三角形/绘制指令的相对度量），级别越高越粗、越便宜 */
static const double level_cost[N_OBJ][LEVELS] = {
    {100, 55, 30, 15}, {100, 55, 30, 15}, {100, 55, 30, 15},
    {100, 55, 30, 15}, {100, 55, 30, 15},
};
/* 每级质量误差（世界单位），级别越高误差越大 */
static const double level_qerr[N_OBJ][LEVELS] = {
    {0, 8, 28, 80}, {0, 6, 22, 64}, {0, 10, 34, 96},
    {0, 7, 25, 72}, {0, 9, 31, 88},
};

static int   vis[FRAMES][N_OBJ];
static double budget[FRAMES];       /* 实际预算（平稳期恒定 + 4 帧骤降） */
static double forecast[FRAMES];     /* 公开 EMA 预测，仅用于展示滞后；滚动策略内部用带回撤重置的即时预测 */

static int    lvl_of[CODES][N_OBJ];
static double code_cost[CODES];
static int    hamming[CODES][CODES];

/* 确定性伪随机，保证场景可复现 */
static unsigned rng_state = 0x12345678u;
static double rng_unit(void) {
    rng_state ^= rng_state << 13; rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return (rng_state >> 8) * (1.0 / 16777208.0);
}

/* 可见性：全物体同步的 2 帧短遮挡，每 10 帧一次。贪心在遮挡帧整批降级、
   恢复后立刻整批升回（每轮 2*N_OBJ 次切换）；多帧 DP 能看穿短遮挡而保持
   级别不动。另加各物体错相的零星 1 帧遮挡作为噪声。 */
static int visibility(int f, int o) {
    int phase = f % 10;
    if (phase == 0 || phase == 1) return 0;
    if ((f + 7 * o) % 13 == 4) return 0;
    return 1;
}

static void build_scenario(void) {
    for (int f = 0; f < FRAMES; f++)
        for (int o = 0; o < N_OBJ; o++)
            vis[f][o] = visibility(f, o);

    /* 预算：普通帧恒为名义值 300（足够全部物体走 L1 共 275），预测器完全可信；
       固定 4 个骤降帧掉到 ~100（仍高于最低可行成本 75），预测必然滞后 -> 回退。
       普通帧无噪声：回退次数精确等于骤降次数，机制可直接从输出读出。 */
    static const int dip_frames[4] = {7, 18, 29, 40};
    for (int f = 0; f < FRAMES; f++) {
        int dip = 0;
        for (int i = 0; i < 4; i++) if (dip_frames[i] == f) dip = 1;
        budget[f] = dip ? 95.0 + 12.0 * rng_unit() : BUDGET_BASELINE;
    }
    /* 预测 = 名义预算 + 偏差的 EMA；骤降只造成一两帧高估，随后快速均值回归 */
    double ema_dev = 0.0;
    forecast[0] = BUDGET_BASELINE;
    for (int f = 1; f < FRAMES; f++) {
        ema_dev = FORECAST_ALPHA * (budget[f - 1] - BUDGET_BASELINE)
                + (1.0 - FORECAST_ALPHA) * ema_dev;
        forecast[f] = BUDGET_BASELINE + ema_dev;
    }
}

static void build_tables(void) {
    for (int c = 0; c < CODES; c++) {
        int x = c;
        code_cost[c] = 0.0;
        for (int o = 0; o < N_OBJ; o++) {
            lvl_of[c][o] = x % LEVELS;
            x /= LEVELS;
            code_cost[c] += level_cost[o][lvl_of[c][o]];
        }
    }
    for (int a = 0; a < CODES; a++)
        for (int b = 0; b < CODES; b++) {
            int d = 0;
            for (int o = 0; o < N_OBJ; o++)
                d += (lvl_of[a][o] != lvl_of[b][o]);
            hamming[a][b] = d;
        }
}

static double frame_qerr(int f, int code) {
    double q = 0.0;
    for (int o = 0; o < N_OBJ; o++)
        q += vis[f][o] * level_qerr[o][lvl_of[code][o]];
    return q;
}

/* 贪心：从最高 LOD 起，超预算则选择 误差增量/节省成本 最小的一次降级 */
static int greedy_pick(int f, double b) {
    int cur[N_OBJ];
    for (int o = 0; o < N_OBJ; o++) cur[o] = 0;
    double cur_cost = 0.0;
    for (int o = 0; o < N_OBJ; o++) cur_cost += level_cost[o][0];

    for (;;) {
        if (cur_cost <= b) break;
        int best_o = -1; double best_ratio = INF;
        for (int o = 0; o < N_OBJ; o++) {
            if (cur[o] >= LEVELS - 1) continue;
            double dq = vis[f][o] *
                        (level_qerr[o][cur[o] + 1] - level_qerr[o][cur[o]]);
            double saved = level_cost[o][cur[o]] - level_cost[o][cur[o] + 1];
            double ratio = dq / saved;               /* 同级 tie 由 o 序打破 */
            if (ratio < best_ratio) { best_ratio = ratio; best_o = o; }
        }
        if (best_o < 0) break;                        /* 全部已到最粗，尽力而为 */
        cur_cost -= level_cost[best_o][cur[best_o]]
                  - level_cost[best_o][cur[best_o] + 1];
        cur[best_o]++;
    }
    int code = 0;
    for (int o = N_OBJ - 1; o >= 0; o--) code = code * LEVELS + cur[o];
    return code;
}

typedef struct {
    int    code[FRAMES];
    double res[FRAMES];     /* 实际占用的渲染资源 */
    double qerr[FRAMES];
    int    sw[FRAMES];
    double total[FRAMES];   /* qerr + lambda * 切换数 */
    int    overruns;
    int    rollbacks;
    int    total_sw;
    double total_q, total_cost;
} Result;

static void record_frame(Result* r, int f, int code, int prev_code, int count_sw) {
    r->code[f] = code;
    r->res[f] = code_cost[code];
    r->qerr[f] = frame_qerr(f, code);
    r->sw[f] = count_sw ? hamming[prev_code][code] : 0;
    r->total[f] = r->qerr[f] + SWITCH_WEIGHT * r->sw[f];
    if (r->res[f] > budget[f] + 1e-9) r->overruns++;
    r->total_sw += r->sw[f];
    r->total_q += r->qerr[f];
    r->total_cost += r->total[f];
}

static void run_greedy(Result* r) {
    memset(r, 0, sizeof(*r));
    int prev = 0;
    for (int f = 0; f < FRAMES; f++) {
        int code = greedy_pick(f, budget[f]);
        record_frame(r, f, code, prev, f > 0);
        prev = code;
    }
}

/* 在给定预算序列上做一次完整 DP，parent 回溯写入 out_codes */
static void dp_solve(const double* budgets, int n, int start_state,
                     int count_first_switch, int* out_codes) {
    static int     feasible[FRAMES][CODES];
    static int     cnt[FRAMES];
    static double  dp[2][CODES];
    static int     parent[FRAMES][CODES];

    for (int f = 0; f < n; f++) {
        cnt[f] = 0;
        for (int c = 0; c < CODES; c++)
            if (code_cost[c] <= budgets[f] + 1e-9)
                feasible[f][cnt[f]++] = c;
    }

    int cur = 0;
    for (int c = 0; c < CODES; c++) dp[cur][c] = INF;
    for (int i = 0; i < cnt[0]; i++) {
        int c = feasible[0][i];
        double sw = count_first_switch ? SWITCH_WEIGHT * hamming[start_state][c] : 0.0;
        dp[cur][c] = frame_qerr(0, c) + sw;
        parent[0][c] = -1;
    }
    for (int f = 1; f < n; f++) {
        int nxt = cur ^ 1;
        for (int i = 0; i < cnt[f]; i++) {
            int c = feasible[f][i];
            double best = INF; int best_p = -1;
            double q = frame_qerr(f, c);
            for (int j = 0; j < cnt[f - 1]; j++) {
                int p = feasible[f - 1][j];
                double v = dp[cur][p] + q + SWITCH_WEIGHT * hamming[p][c];
                if (v < best) { best = v; best_p = p; }
            }
            dp[nxt][c] = best;
            parent[f][c] = best_p;
        }
        cur = nxt;
    }
    int best_code = feasible[n - 1][0];
    for (int i = 1; i < cnt[n - 1]; i++) {
        int c = feasible[n - 1][i];
        if (dp[cur][c] < dp[cur][best_code]) best_code = c;
    }
    out_codes[n - 1] = best_code;
    for (int f = n - 1; f >= 1; f--)
        out_codes[f - 1] = parent[f][out_codes[f]];
}

/* oracle：全程实际预算已知，一次性全局 DP */
static void run_oracle(Result* r) {
    memset(r, 0, sizeof(*r));
    int codes[FRAMES];
    dp_solve(budget, FRAMES, 0, 0, codes);
    int prev = 0;
    for (int f = 0; f < FRAMES; f++) {
        record_frame(r, f, codes[f], prev, f > 0);
        prev = codes[f];
    }
}

/* 滚动视界 DP：每帧基于预测预算规划 WINDOW 帧，只提交首帧；
   实际预算超支时回退到贪心可行解，并把预测偏差重置为 0 —— 回退已经
   确认这是瞬时冲击而非趋势，下一帧预测立即回到名义预算，DP 可完整升级 */
static void run_rolling(Result* r) {
    memset(r, 0, sizeof(*r));
    int prev = 0;
    double ema_dev = 0.0;
    for (int f = 0; f < FRAMES; f++) {
        int n = FRAMES - f < WINDOW ? FRAMES - f : WINDOW;
        double win_budget[WINDOW];
        for (int k = 0; k < n; k++) {
            /* 用滚动维护的偏差即时预测，回退后偏差已归零，窗口预算立刻恢复 */
            win_budget[k] = BUDGET_BASELINE + ema_dev;
            if (win_budget[k] < code_cost[CODES - 1])   /* 不低于最低可行成本 */
                win_budget[k] = code_cost[CODES - 1];
        }
        int plan[WINDOW];
        dp_solve(win_budget, n, prev, f > 0, plan);

        int code = plan[0];
        if (code_cost[code] > budget[f] + 1e-9) {       /* 预测失准：回退 */
            code = greedy_pick(f, budget[f]);
            r->rollbacks++;
            ema_dev = 0.0;
        } else {
            ema_dev = FORECAST_ALPHA * (budget[f] - BUDGET_BASELINE)
                    + (1.0 - FORECAST_ALPHA) * ema_dev;
        }
        record_frame(r, f, code, prev, 1);
        prev = code;
    }
}

static void levels_str(int code, char* out) {
    for (int o = 0; o < N_OBJ; o++) out[o] = (char)('0' + lvl_of[code][o]);
    out[N_OBJ] = '\0';
}

int main(void) {
    build_scenario();
    build_tables();

    Result greedy, rolling, oracle;
    run_greedy(&greedy);
    run_rolling(&rolling);
    run_oracle(&oracle);

    printf("frame  budget(ema/real) greedy levels/res/q/sw/tot   "
           "rolling levels/res/q/sw/tot rb   oracle tot\n");
    for (int f = 0; f < FRAMES; f++) {
        char g[8], d[8];
        levels_str(greedy.code[f], g);
        levels_str(rolling.code[f], d);
        printf("%4d  %6.0f/%-6.0f  %s %5.0f %6.1f %d %6.1f   "
               "%s %5.0f %6.1f %d %6.1f %2d   %6.1f\n",
               f, forecast[f], budget[f],
               g, greedy.res[f], greedy.qerr[f], greedy.sw[f], greedy.total[f],
               d, rolling.res[f], rolling.qerr[f], rolling.sw[f], rolling.total[f],
               rolling.res[f] > budget[f] + 1e-9 ? 1 : 0,
               oracle.total[f]);
    }

    double gre_res = 0.0, rol_res = 0.0, ora_res = 0.0;
    for (int f = 0; f < FRAMES; f++) {
        gre_res += greedy.res[f]; rol_res += rolling.res[f]; ora_res += oracle.res[f];
    }
    printf("\n== summary ==\n");
    printf("%-8s %12s %8s %12s %10s %10s\n",
           "strategy", "quality_err", "switches", "objective", "res_total", "overruns");
    printf("%-8s %12.2f %8d %12.2f %10.0f %10d\n", "greedy",
           greedy.total_q, greedy.total_sw, greedy.total_cost,
           gre_res, greedy.overruns);
    printf("%-8s %12.2f %8d %12.2f %10.0f %10d  (rollbacks=%d)\n", "rolling",
           rolling.total_q, rolling.total_sw, rolling.total_cost,
           rol_res, rolling.overruns, rolling.rollbacks);
    printf("%-8s %12.2f %8d %12.2f %10.0f %10d\n", "oracle",
           oracle.total_q, oracle.total_sw, oracle.total_cost,
           ora_res, oracle.overruns);

    int pass = 1;
    printf("\n== checks ==\n");
    /* 硬约束：任何策略任何帧都不得超预算（回退必须兜住） */
    int no_overrun = !greedy.overruns && !rolling.overruns && !oracle.overruns;
    printf("budget_never_exceeded=%d\n", no_overrun); pass &= no_overrun;
    /* 回退确实发生过（预测预算在骤降帧高估） */
    printf("rollback_observed=%d (count=%d)\n", rolling.rollbacks > 0, rolling.rollbacks);
    pass &= (rolling.rollbacks > 0);
    /* 总代价不高于贪心：oracle 是全局最优，必然成立；rolling 实测同样成立 */
    int cost_ok = oracle.total_cost <= greedy.total_cost + 1e-9
               && rolling.total_cost <= greedy.total_cost + 1e-9;
    printf("cost_not_worse_than_greedy=%d (greedy=%.2f rolling=%.2f oracle=%.2f)\n",
           cost_ok, greedy.total_cost, rolling.total_cost, oracle.total_cost);
    pass &= cost_ok;
    /* 切换次数低于贪心三成：sw <= 0.7 * greedy_sw */
    int sw_roll = rolling.total_sw * 10 <= 7 * greedy.total_sw;
    int sw_orac = oracle.total_sw * 10 <= 7 * greedy.total_sw;
    printf("switches_30pct_less=%d (greedy=%d rolling=%d oracle=%d, cap=%d)\n",
           sw_roll && sw_orac, greedy.total_sw, rolling.total_sw,
           oracle.total_sw, (7 * greedy.total_sw) / 10);
    pass &= (sw_roll && sw_orac);

    printf("ALL_CHECKS_PASS=%d\n", pass);
    return pass ? 0 : 1;
}
