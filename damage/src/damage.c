/* damage: staged damage pipeline.
 *
 * Base damage flows through four stages in a fixed canonical order:
 *
 *   CRIT    dmg * crit_mult / 1000      (only when roll < crit_chance)
 *   ARMOR   max(0, dmg - armor)         (flat absorption)
 *   RESIST  dmg * (1000 - resist) / 1000
 *   REDUCE  dmg * (1000 - reduction) / 1000
 *
 * Every stage is integer arithmetic (per-mille multipliers, floor after
 * each multiply) so a trace can be checked against hand calculation
 * exactly. Each stage can be disabled; stage outputs saturate to
 * [0, DMG_MAX]. Stats are clamped once, in dmg_config_sanitize, so no
 * call site can apply a half-valid formula.
 *
 * The canonical order is the production contract. Alternate orders are
 * available for balance experiments: dmg_resolve_order evaluates one,
 * dmg_order_sweep evaluates all 24, and dmg_diff quantifies how much a
 * reordering moves the result. */
#include <stdio.h>
#include <limits.h>
#include <string.h>

#define DMG_STAGE_COUNT 4
#define DMG_ORDER_COUNT 24          /* 4! */
#define DMG_PERMILLE 1000
#define DMG_MAX 1000000000LL

#define DMG_CHANCE_MAX_BP 10000     /* 100.00% */
#define DMG_ROLL_MAX_BP 9999
#define DMG_CRIT_MULT_MIN 100       /* 0.1x */
#define DMG_CRIT_MULT_MAX 10000     /* 10x */
#define DMG_RESIST_MIN (-900)       /* -90% = up to +90% vulnerability */
#define DMG_RESIST_MAX 900          /* 90% */
#define DMG_REDUCE_MIN 0
#define DMG_REDUCE_MAX 900          /* 90% */

typedef enum {
    DMG_STAGE_CRIT = 0,
    DMG_STAGE_ARMOR,
    DMG_STAGE_RESIST,
    DMG_STAGE_REDUCE
} DmgStage;

typedef struct {
    int enabled[DMG_STAGE_COUNT];   /* normalized to 0/1 by sanitize */
    int crit_chance_bp;             /* 0..10000 basis points */
    int crit_mult_permille;         /* 100..10000 */
    long long armor;                /* flat absorption, 0..DMG_MAX */
    int resist_permille;            /* -900..900; negative = vulnerability */
    int reduction_permille;         /* 0..900 */
} DmgConfig;

typedef struct {
    long long base;                 /* clamped to [0, DMG_MAX] */
    int crit_roll_bp;               /* deterministic roll, 0..9999 */
} DmgInput;

typedef struct {
    DmgStage stage;
    long long before;
    long long after;
    int skipped;                    /* stage disabled */
    int saturated;                  /* stage output hit 0 or DMG_MAX */
} DmgStep;

typedef struct {
    long long base;
    long long final;
    int saturated;                  /* any input or stage saturated */
    int count;                      /* always DMG_STAGE_COUNT on success */
    DmgStep steps[DMG_STAGE_COUNT];
} DmgTrace;

typedef struct {
    long long final_a;
    long long final_b;
    long long final_delta;          /* |final_b - final_a| */
    int rel_permille;               /* final_delta * 1000 / final_a */
    long long stage_delta[DMG_STAGE_COUNT];  /* per stage, after-values */
    long long max_stage_delta;
} DmgDiff;

typedef struct {
    int count;
    DmgStage orders[DMG_ORDER_COUNT][DMG_STAGE_COUNT];
    long long finals[DMG_ORDER_COUNT];
    long long min_final;
    long long max_final;
    long long spread;               /* max_final - min_final */
    int min_index;
    int max_index;
} DmgSweep;

typedef __int128 dmg_wide;

static const DmgStage CANONICAL_ORDER[DMG_STAGE_COUNT] = {
    DMG_STAGE_CRIT, DMG_STAGE_ARMOR, DMG_STAGE_RESIST, DMG_STAGE_REDUCE
};

static int clamp_int(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static long long clamp_ll(long long v, long long lo, long long hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Saturate a value into the legal damage range; flags the clamp. */
static long long clamp_damage(long long v, int* saturated) {
    if (v < 0) {
        if (saturated) *saturated = 1;
        return 0;
    }
    if (v > DMG_MAX) {
        if (saturated) *saturated = 1;
        return DMG_MAX;
    }
    return v;
}

/* a * num / den with a 128-bit intermediate, saturated to DMG_MAX.
   Inputs are non-negative after sanitization, so only the upper bound
   can be exceeded; the wide product keeps it from wrapping. */
static long long sat_mul_div(long long a, long long num, long long den, int* saturated) {
    if (den <= 0) {
        if (saturated) *saturated = 1;
        return 0;
    }
    dmg_wide v = (dmg_wide)a * (dmg_wide)num / (dmg_wide)den;
    if (v < 0) {
        if (saturated) *saturated = 1;
        return 0;
    }
    if (v > (dmg_wide)DMG_MAX) {
        if (saturated) *saturated = 1;
        return DMG_MAX;
    }
    return (long long)v;
}

void dmg_config_init(DmgConfig* cfg) {
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    for (int i = 0; i < DMG_STAGE_COUNT; i++) cfg->enabled[i] = 1;
    cfg->crit_mult_permille = 2000;
}

/* Single entry point for stat clamping. resolve/sweep always call this,
   so an out-of-range resistance can never reach a stage unclamped. */
void dmg_config_sanitize(const DmgConfig* in, DmgConfig* out) {
    if (!out) return;
    if (!in) {
        memset(out, 0, sizeof(*out));
        return;
    }
    for (int i = 0; i < DMG_STAGE_COUNT; i++)
        out->enabled[i] = in->enabled[i] ? 1 : 0;
    out->crit_chance_bp = clamp_int(in->crit_chance_bp, 0, DMG_CHANCE_MAX_BP);
    out->crit_mult_permille = clamp_int(in->crit_mult_permille,
                                        DMG_CRIT_MULT_MIN, DMG_CRIT_MULT_MAX);
    out->armor = clamp_ll(in->armor, 0, DMG_MAX);
    out->resist_permille = clamp_int(in->resist_permille,
                                     DMG_RESIST_MIN, DMG_RESIST_MAX);
    out->reduction_permille = clamp_int(in->reduction_permille,
                                        DMG_REDUCE_MIN, DMG_REDUCE_MAX);
}

void dmg_canonical_order(DmgStage out[DMG_STAGE_COUNT]) {
    if (!out) return;
    memcpy(out, CANONICAL_ORDER, sizeof(CANONICAL_ORDER));
}

static int order_valid(const DmgStage* order) {
    int seen[DMG_STAGE_COUNT] = { 0 };
    for (int i = 0; i < DMG_STAGE_COUNT; i++) {
        int s = (int)order[i];
        if (s < 0 || s >= DMG_STAGE_COUNT || seen[s]) return 0;
        seen[s] = 1;
    }
    return 1;
}

static long long apply_stage(DmgStage stage, const DmgConfig* cfg,
                             long long dmg, int roll_bp, int* saturated) {
    switch (stage) {
    case DMG_STAGE_CRIT:
        if (roll_bp < cfg->crit_chance_bp)
            return sat_mul_div(dmg, cfg->crit_mult_permille, DMG_PERMILLE, saturated);
        return dmg;
    case DMG_STAGE_ARMOR:
        if (cfg->armor > dmg) {
            if (saturated) *saturated = 1;
            return 0;
        }
        return dmg - cfg->armor;
    case DMG_STAGE_RESIST:
        return sat_mul_div(dmg, DMG_PERMILLE - cfg->resist_permille,
                           DMG_PERMILLE, saturated);
    case DMG_STAGE_REDUCE:
        return sat_mul_div(dmg, DMG_PERMILLE - cfg->reduction_permille,
                           DMG_PERMILLE, saturated);
    default:
        return dmg;
    }
}

/* Evaluate one explicit ordering. Returns the final damage, or -1 when
   the arguments are invalid (NULL config/input or a non-permutation). */
long long dmg_resolve_order(const DmgConfig* cfg, const DmgInput* in,
                            const DmgStage* order, DmgTrace* trace) {
    if (!cfg || !in) return -1;
    if (order && !order_valid(order)) return -1;

    DmgConfig c;
    dmg_config_sanitize(cfg, &c);

    int saturated = 0;
    long long cur = clamp_damage(in->base, &saturated);
    int roll = clamp_int(in->crit_roll_bp, 0, DMG_ROLL_MAX_BP);

    if (trace) {
        memset(trace, 0, sizeof(*trace));
        trace->base = cur;
    }

    for (int i = 0; i < DMG_STAGE_COUNT; i++) {
        DmgStage stage = order ? order[i] : CANONICAL_ORDER[i];
        long long before = cur;
        int stage_sat = 0;
        if (c.enabled[stage])
            cur = apply_stage(stage, &c, before, roll, &stage_sat);
        if (stage_sat) saturated = 1;
        if (trace) {
            DmgStep* step = &trace->steps[trace->count++];
            step->stage = stage;
            step->before = before;
            step->after = cur;
            step->skipped = !c.enabled[stage];
            step->saturated = stage_sat;
        }
    }

    if (trace) {
        trace->final = cur;
        trace->saturated = saturated;
    }
    return cur;
}

/* Production entry point: always the canonical order. */
long long dmg_resolve(const DmgConfig* cfg, const DmgInput* in, DmgTrace* trace) {
    return dmg_resolve_order(cfg, in, NULL, trace);
}

/* Quantify the difference between two runs (possibly different orders). */
int dmg_diff(const DmgTrace* a, const DmgTrace* b, DmgDiff* out) {
    if (!a || !b || !out) return -1;
    memset(out, 0, sizeof(*out));
    out->final_a = a->final;
    out->final_b = b->final;
    out->final_delta = a->final > b->final ? a->final - b->final
                                           : b->final - a->final;

    for (int s = 0; s < DMG_STAGE_COUNT; s++) {
        long long av = 0, bv = 0;
        int found_a = 0, found_b = 0;
        for (int i = 0; i < a->count && i < DMG_STAGE_COUNT; i++)
            if ((int)a->steps[i].stage == s) { av = a->steps[i].after; found_a = 1; }
        for (int i = 0; i < b->count && i < DMG_STAGE_COUNT; i++)
            if ((int)b->steps[i].stage == s) { bv = b->steps[i].after; found_b = 1; }
        if (!found_a || !found_b) return -1;
        long long delta = av > bv ? av - bv : bv - av;
        out->stage_delta[s] = delta;
        if (delta > out->max_stage_delta) out->max_stage_delta = delta;
    }

    long long denom = out->final_a > 0 ? out->final_a : 1;
    long long rel = out->final_delta * 1000 / denom;
    if (rel > INT_MAX) rel = INT_MAX;
    out->rel_permille = (int)rel;
    return 0;
}

static void sweep_rec(const DmgConfig* cfg, const DmgInput* in,
                      DmgStage cur[DMG_STAGE_COUNT], int used[DMG_STAGE_COUNT],
                      int depth, DmgSweep* out) {
    if (depth == DMG_STAGE_COUNT) {
        int idx = out->count++;
        DmgTrace tr;
        memcpy(out->orders[idx], cur, sizeof(out->orders[idx]));
        out->finals[idx] = dmg_resolve_order(cfg, in, cur, &tr);
        return;
    }
    for (int s = 0; s < DMG_STAGE_COUNT; s++) {
        if (used[s]) continue;
        used[s] = 1;
        cur[depth] = (DmgStage)s;
        sweep_rec(cfg, in, cur, used, depth + 1, out);
        used[s] = 0;
    }
}

/* Evaluate all 4! orderings and report the spread. */
int dmg_order_sweep(const DmgConfig* cfg, const DmgInput* in, DmgSweep* out) {
    if (!cfg || !in || !out) return -1;

    DmgConfig c;
    dmg_config_sanitize(cfg, &c);

    memset(out, 0, sizeof(*out));
    DmgStage cur[DMG_STAGE_COUNT];
    int used[DMG_STAGE_COUNT] = { 0 };
    sweep_rec(&c, in, cur, used, 0, out);
    if (out->count != DMG_ORDER_COUNT) return -1;

    out->min_final = out->max_final = out->finals[0];
    out->min_index = out->max_index = 0;
    for (int i = 1; i < out->count; i++) {
        if (out->finals[i] < out->min_final) {
            out->min_final = out->finals[i];
            out->min_index = i;
        }
        if (out->finals[i] > out->max_final) {
            out->max_final = out->finals[i];
            out->max_index = i;
        }
    }
    out->spread = out->max_final - out->min_final;
    return 0;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    DmgConfig cfg;
    dmg_config_init(&cfg);
    check("defaults_enabled", cfg.enabled[0] && cfg.enabled[1] && cfg.enabled[2] && cfg.enabled[3]);
    check("defaults_zero_stats", cfg.crit_chance_bp == 0 && cfg.armor == 0
                                 && cfg.resist_permille == 0 && cfg.reduction_permille == 0);

    /* Hand-checked canonical run:
       1000 -> crit 1500 -> armor 1300 -> resist 975 -> reduce 877 */
    cfg.crit_chance_bp = 5000;
    cfg.crit_mult_permille = 1500;
    cfg.armor = 200;
    cfg.resist_permille = 250;
    cfg.reduction_permille = 100;

    DmgInput in = { 1000, 1000 };
    DmgTrace tr;
    long long final_dmg = dmg_resolve(&cfg, &in, &tr);
    check("full_final", final_dmg == 877);
    check("full_trace_base", tr.base == 1000 && tr.final == 877);
    check("full_trace_count", tr.count == DMG_STAGE_COUNT);
    check("full_crit_step", tr.steps[0].stage == DMG_STAGE_CRIT
                              && tr.steps[0].before == 1000 && tr.steps[0].after == 1500);
    check("full_armor_step", tr.steps[1].stage == DMG_STAGE_ARMOR
                              && tr.steps[1].before == 1500 && tr.steps[1].after == 1300);
    check("full_resist_step", tr.steps[2].before == 1300 && tr.steps[2].after == 975);
    check("full_reduce_step", tr.steps[3].before == 975 && tr.steps[3].after == 877);
    check("full_no_saturation", tr.saturated == 0);
    check("trace_optional", dmg_resolve(&cfg, &in, NULL) == 877);

    in.crit_roll_bp = 9000;
    check("no_crit_final", dmg_resolve(&cfg, &in, &tr) == 540);

    in.crit_roll_bp = 4999;
    dmg_resolve(&cfg, &in, &tr);
    check("crit_boundary_hit", tr.steps[0].after == 1500);
    in.crit_roll_bp = 5000;
    dmg_resolve(&cfg, &in, &tr);
    check("crit_boundary_miss", tr.steps[0].after == 1000);

    cfg.crit_chance_bp = 1;
    in.crit_roll_bp = INT_MIN;      /* roll clamps to 0, so it crits */
    dmg_resolve(&cfg, &in, &tr);
    check("crit_roll_clamped", tr.steps[0].after == 1500);

    /* Per-stage toggles. */
    cfg.crit_chance_bp = 10000;
    in.crit_roll_bp = 0;

    DmgConfig off = cfg;
    off.enabled[DMG_STAGE_ARMOR] = 0;
    check("toggle_armor", dmg_resolve(&off, &in, &tr) == 1012);
    check("toggle_armor_trace", tr.steps[1].skipped == 1
                                 && tr.steps[1].before == 1500 && tr.steps[1].after == 1500);

    off = cfg;
    off.enabled[DMG_STAGE_RESIST] = 0;
    check("toggle_resist", dmg_resolve(&off, &in, &tr) == 1170);

    off = cfg;
    off.enabled[DMG_STAGE_REDUCE] = 0;
    check("toggle_reduce", dmg_resolve(&off, &in, &tr) == 975);

    off = cfg;
    for (int i = 0; i < DMG_STAGE_COUNT; i++) off.enabled[i] = 0;
    check("toggle_all_off", dmg_resolve(&off, &in, &tr) == 1000);

    off.enabled[DMG_STAGE_CRIT] = 7;    /* any non-zero means enabled */
    check("toggle_normalize", dmg_resolve(&off, &in, &tr) == 1500);

    /* Saturation instead of overflow. */
    DmgConfig satc;
    dmg_config_init(&satc);
    DmgInput huge = { LLONG_MAX, 0 };
    check("base_high_saturates", dmg_resolve(&satc, &huge, &tr) == DMG_MAX);
    check("base_high_flag", tr.base == DMG_MAX && tr.saturated == 1);

    DmgInput tiny = { LLONG_MIN, 0 };
    check("base_low_saturates", dmg_resolve(&satc, &tiny, &tr) == 0);
    check("base_low_flag", tr.saturated == 1);

    satc.crit_chance_bp = 10000;
    satc.crit_mult_permille = INT_MAX;  /* sanitized to 10x */
    DmgInput capped = { DMG_MAX, 0 };
    check("crit_mult_saturates", dmg_resolve(&satc, &capped, &tr) == DMG_MAX);
    check("crit_mult_flag", tr.saturated == 1 && tr.steps[0].saturated == 1);

    DmgConfig tank;
    dmg_config_init(&tank);
    tank.armor = LLONG_MAX;
    DmgInput hit = { 1000, 0 };
    check("armor_saturates", dmg_resolve(&tank, &hit, &tr) == 0);
    check("armor_flag", tr.steps[1].saturated == 1);

    /* Stats clamp once, at the boundary. */
    DmgConfig raw;
    memset(&raw, 0, sizeof(raw));
    raw.enabled[0] = 9;
    raw.crit_chance_bp = INT_MAX;
    raw.crit_mult_permille = INT_MAX;
    raw.armor = LLONG_MAX;
    raw.resist_permille = INT_MIN;
    raw.reduction_permille = INT_MIN;
    DmgConfig norm;
    dmg_config_sanitize(&raw, &norm);
    check("sanitize_chance", norm.crit_chance_bp == DMG_CHANCE_MAX_BP);
    check("sanitize_mult", norm.crit_mult_permille == DMG_CRIT_MULT_MAX);
    check("sanitize_armor", norm.armor == DMG_MAX);
    check("sanitize_resist", norm.resist_permille == DMG_RESIST_MIN);
    check("sanitize_reduce", norm.reduction_permille == DMG_REDUCE_MIN);
    check("sanitize_enabled", norm.enabled[0] == 1 && norm.enabled[1] == 0);

    DmgConfig vuln;
    dmg_config_init(&vuln);
    vuln.resist_permille = -500;
    DmgInput vbase = { 1000, 0 };
    check("vulnerability", dmg_resolve(&vuln, &vbase, &tr) == 1500);
    vuln.resist_permille = -5000;
    check("vulnerability_clamped", dmg_resolve(&vuln, &vbase, &tr) == 1900);

    /* Argument and order validation. */
    DmgStage canon[DMG_STAGE_COUNT];
    dmg_canonical_order(canon);
    check("canonical_order", canon[0] == DMG_STAGE_CRIT && canon[1] == DMG_STAGE_ARMOR
                             && canon[2] == DMG_STAGE_RESIST && canon[3] == DMG_STAGE_REDUCE);
    check("reject_null_cfg", dmg_resolve(NULL, &in, &tr) == -1);
    check("reject_null_input", dmg_resolve(&cfg, NULL, &tr) == -1);
    DmgStage bad_order[DMG_STAGE_COUNT] = {
        DMG_STAGE_CRIT, DMG_STAGE_CRIT, DMG_STAGE_RESIST, DMG_STAGE_REDUCE
    };
    check("reject_duplicate_order", dmg_resolve_order(&cfg, &in, bad_order, &tr) == -1);

    /* Reordering: crit-then-armor vs armor-then-crit.
       100 * 2 = 200 - 50 = 150   vs   (100 - 50) * 2 = 100 */
    DmgConfig ord;
    dmg_config_init(&ord);
    ord.crit_chance_bp = 10000;
    ord.crit_mult_permille = 2000;
    ord.armor = 50;
    DmgInput oi = { 100, 0 };
    DmgTrace canon_trace, armor_trace;
    long long canon_final = dmg_resolve(&ord, &oi, &canon_trace);
    DmgStage armor_first[DMG_STAGE_COUNT] = {
        DMG_STAGE_ARMOR, DMG_STAGE_CRIT, DMG_STAGE_RESIST, DMG_STAGE_REDUCE
    };
    long long armor_final = dmg_resolve_order(&ord, &oi, armor_first, &armor_trace);
    check("canonical_expected", canon_final == 150);
    check("armor_first_expected", armor_final == 100);

    DmgDiff diff;
    check("diff_ok", dmg_diff(&canon_trace, &armor_trace, &diff) == 0);
    check("diff_finals", diff.final_a == 150 && diff.final_b == 100);
    check("diff_final_delta", diff.final_delta == 50 && diff.rel_permille == 333);
    check("diff_stage_delta", diff.stage_delta[DMG_STAGE_CRIT] == 100
                              && diff.stage_delta[DMG_STAGE_ARMOR] == 100
                              && diff.stage_delta[DMG_STAGE_RESIST] == 50);
    check("diff_max_stage_delta", diff.max_stage_delta == 100);

    DmgSweep sweep;
    check("sweep_ok", dmg_order_sweep(&ord, &oi, &sweep) == 0);
    check("sweep_count", sweep.count == DMG_ORDER_COUNT);
    check("sweep_range", sweep.min_final == 100 && sweep.max_final == 150
                          && sweep.spread == 50);
    int low = 0, high = 0;
    for (int i = 0; i < sweep.count; i++) {
        if (sweep.finals[i] == 100) low++;
        else if (sweep.finals[i] == 150) high++;
    }
    check("sweep_split", low == 12 && high == 12);
    check("sweep_indices", sweep.finals[sweep.min_index] == 100
                            && sweep.finals[sweep.max_index] == 150);

    /* Multiplicative stages only: order changes are pure per-stage
       rounding, here one point on a base of 3. */
    DmgConfig rnd;
    dmg_config_init(&rnd);
    rnd.enabled[DMG_STAGE_ARMOR] = 0;
    rnd.crit_chance_bp = 10000;
    rnd.crit_mult_permille = 1500;
    rnd.resist_permille = 500;
    rnd.reduction_permille = 200;
    DmgInput rbase = { 3, 0 };
    DmgSweep rs;
    check("rounding_sweep_ok", dmg_order_sweep(&rnd, &rbase, &rs) == 0);
    check("rounding_spread", rs.min_final == 0 && rs.max_final == 1 && rs.spread == 1);

    DmgConfig none;
    dmg_config_init(&none);
    for (int i = 0; i < DMG_STAGE_COUNT; i++) none.enabled[i] = 0;
    DmgInput nbase = { 777, 0 };
    DmgSweep ns;
    check("disabled_sweep_ok", dmg_order_sweep(&none, &nbase, &ns) == 0);
    int all_same = 1;
    for (int i = 0; i < ns.count; i++) if (ns.finals[i] != 777) all_same = 0;
    check("disabled_sweep_flat", ns.spread == 0 && all_same);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
