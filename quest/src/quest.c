#define _POSIX_C_SOURCE 200809L
/* quest: 数据驱动的任务系统。
 * 条件树由文本配置定义（counter/flag/cond/quest），不写死在代码里；
 * 进度通过计数器累加，支持按日志回滚与提交；完成状态锁存，回调只触发一次；
 * 加载时校验配置：检出条件树成环与引用缺失。 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>

#define MAX_COUNTERS 128
#define MAX_FLAGS 64
#define MAX_CONDS 4096
#define MAX_QUESTS 1024
#define MAX_CHILDREN 8
#define MAX_PENDING (MAX_CONDS * MAX_CHILDREN + MAX_QUESTS)
#define MAX_JOURNAL 16384
#define MAX_ERRORS 64
#define NAME_LEN 32

typedef enum { COND_COUNTER_GE = 0, COND_FLAG_SET, COND_AND, COND_OR, COND_NOT } CondType;

typedef struct {
    char name[NAME_LEN];
    int type;
    int target;                 /* 叶子：counter/flag 下标 */
    long threshold;             /* COND_COUNTER_GE */
    int children[MAX_CHILDREN]; /* 内部节点：cond 下标 */
    int nchildren;
} Cond;

typedef struct {
    char name[NAME_LEN];
    int root;        /* 条件树根 */
    int completed;   /* 锁存：一旦完成不再撤销 */
    int fire_count;  /* 完成回调触发次数，必须 <= 1 */
} Quest;

typedef struct QuestSys QuestSys;
struct QuestSys {
    char counter_names[MAX_COUNTERS][NAME_LEN];
    long counters[MAX_COUNTERS];
    int ncounters;
    char flag_names[MAX_FLAGS][NAME_LEN];
    int flags[MAX_FLAGS];
    int nflags;
    Cond conds[MAX_CONDS];
    int nconds;
    Quest quests[MAX_QUESTS];
    int nquests;
    struct { int counter; long delta; } journal[MAX_JOURNAL]; /* 回滚日志：仅计数器 */
    int njournal;
    char errors[MAX_ERRORS][128];
    int nerrors;
    void (*on_complete)(QuestSys* qs, int quest_idx, void* user);
    void* user;
};

void quest_init(QuestSys* qs) {
    memset(qs, 0, sizeof(*qs));
}

static void add_error(QuestSys* qs, const char* fmt, ...) {
    if (qs->nerrors >= MAX_ERRORS) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(qs->errors[qs->nerrors], sizeof(qs->errors[0]), fmt, ap);
    va_end(ap);
    qs->nerrors++;
}

int quest_error_count(const QuestSys* qs) { return qs->nerrors; }
const char* quest_error(const QuestSys* qs, int i) { return qs->errors[i]; }

static int find_name(char names[][NAME_LEN], int n, const char* s) {
    for (int i = 0; i < n; i++)
        if (!strcmp(names[i], s)) return i;
    return -1;
}

int quest_counter_id(const QuestSys* qs, const char* name) {
    return find_name((char(*)[NAME_LEN])qs->counter_names, qs->ncounters, name);
}
int quest_flag_id(const QuestSys* qs, const char* name) {
    return find_name((char(*)[NAME_LEN])qs->flag_names, qs->nflags, name);
}
int quest_find(const QuestSys* qs, const char* name) {
    for (int i = 0; i < qs->nquests; i++)
        if (!strcmp(qs->quests[i].name, name)) return i;
    return -1;
}
long quest_counter_value(const QuestSys* qs, int c) { return qs->counters[c]; }
int quest_completed(const QuestSys* qs, int q) { return qs->quests[q].completed; }
int quest_fire_count(const QuestSys* qs, int q) { return qs->quests[q].fire_count; }
int quest_count(const QuestSys* qs) { return qs->nquests; }

/* ---- 运行时：累加 / 回滚 / 提交 ---- */

int quest_add(QuestSys* qs, int counter, long delta) {
    if (counter < 0 || counter >= qs->ncounters) return -1;
    if (qs->njournal >= MAX_JOURNAL) return -2;
    qs->counters[counter] += delta;
    qs->journal[qs->njournal].counter = counter;
    qs->journal[qs->njournal].delta = delta;
    qs->njournal++;
    return 0;
}

int quest_undo(QuestSys* qs) {
    if (qs->njournal == 0) return -1;
    qs->njournal--;
    qs->counters[qs->journal[qs->njournal].counter] -= qs->journal[qs->njournal].delta;
    return 0;
}

void quest_commit(QuestSys* qs) { qs->njournal = 0; } /* 提交点：之前的进度不可再回滚 */

int quest_set_flag(QuestSys* qs, int flag, int v) {
    if (flag < 0 || flag >= qs->nflags) return -1;
    qs->flags[flag] = v != 0;
    return 0;
}

/* ---- 求值 ---- */

static int eval_cond(const QuestSys* qs, int ci) {
    const Cond* c = &qs->conds[ci];
    switch (c->type) {
    case COND_COUNTER_GE: return qs->counters[c->target] >= c->threshold;
    case COND_FLAG_SET: return qs->flags[c->target] != 0;
    case COND_AND:
        for (int i = 0; i < c->nchildren; i++)
            if (!eval_cond(qs, c->children[i])) return 0;
        return 1;
    case COND_OR:
        for (int i = 0; i < c->nchildren; i++)
            if (eval_cond(qs, c->children[i])) return 1;
        return 0;
    case COND_NOT: return !eval_cond(qs, c->children[0]);
    }
    return 0;
}

/* 全量求值一遍；完成的锁存并触发回调。返回本次新完成的任务数。 */
int quest_evaluate(QuestSys* qs) {
    int newly = 0;
    for (int i = 0; i < qs->nquests; i++) {
        Quest* q = &qs->quests[i];
        if (q->completed) continue;
        if (eval_cond(qs, q->root)) {
            q->completed = 1;
            q->fire_count++;
            newly++;
            if (qs->on_complete) qs->on_complete(qs, i, qs->user);
        }
    }
    return newly;
}

/* ---- 配置加载与校验 ----
 * 语法（# 开头为注释）：
 *   counter NAME
 *   flag NAME
 *   cond NAME counter CNAME >= N      （counter/flag 须先声明）
 *   cond NAME flag FNAME
 *   cond NAME and A B ...             （1..8 个子条件，可前向引用）
 *   cond NAME or  A B ...
 *   cond NAME not A
 *   quest NAME COND
 */
typedef struct { int is_quest; int owner; int slot; char name[NAME_LEN]; } PendingRef;
static PendingRef pending[MAX_PENDING];

static int push_pending(QuestSys* qs, int* np, int is_quest, int owner, int slot, const char* name) {
    if (*np >= MAX_PENDING) { add_error(qs, "too many references"); return -1; }
    PendingRef* p = &pending[(*np)++];
    p->is_quest = is_quest; p->owner = owner; p->slot = slot;
    snprintf(p->name, NAME_LEN, "%s", name);
    return 0;
}

static int reg_cond(QuestSys* qs, const char* name, Cond** out) {
    if (qs->nconds >= MAX_CONDS) { add_error(qs, "too many conds"); return -1; }
    for (int i = 0; i < qs->nconds; i++)
        if (!strcmp(qs->conds[i].name, name)) { add_error(qs, "duplicate cond '%s'", name); return -1; }
    Cond* c = &qs->conds[qs->nconds];
    memset(c, 0, sizeof(*c));
    snprintf(c->name, NAME_LEN, "%s", name);
    qs->nconds++;
    *out = c;
    return 0;
}

int quest_load(QuestSys* qs, const char* text) {
    qs->nerrors = 0;
    int npending = 0;
    char* buf = malloc(strlen(text) + 1);
    if (!buf) { add_error(qs, "oom"); return -1; }
    strcpy(buf, text);

    char* save1;
    for (char* line = strtok_r(buf, "\n", &save1); line; line = strtok_r(NULL, "\n", &save1)) {
        char* tok[20];
        int nt = 0;
        char* save2;
        for (char* t = strtok_r(line, " \t\r", &save2); t && nt < 20; t = strtok_r(NULL, " \t\r", &save2))
            tok[nt++] = t;
        if (nt == 0 || tok[0][0] == '#') continue;

        if (!strcmp(tok[0], "counter") && nt == 2) {
            if (strlen(tok[1]) >= NAME_LEN) { add_error(qs, "name too long '%s'", tok[1]); continue; }
            if (qs->ncounters >= MAX_COUNTERS) { add_error(qs, "too many counters"); continue; }
            if (quest_counter_id(qs, tok[1]) >= 0) { add_error(qs, "duplicate counter '%s'", tok[1]); continue; }
            snprintf(qs->counter_names[qs->ncounters++], NAME_LEN, "%s", tok[1]);
        } else if (!strcmp(tok[0], "flag") && nt == 2) {
            if (strlen(tok[1]) >= NAME_LEN) { add_error(qs, "name too long '%s'", tok[1]); continue; }
            if (qs->nflags >= MAX_FLAGS) { add_error(qs, "too many flags"); continue; }
            if (quest_flag_id(qs, tok[1]) >= 0) { add_error(qs, "duplicate flag '%s'", tok[1]); continue; }
            snprintf(qs->flag_names[qs->nflags++], NAME_LEN, "%s", tok[1]);
        } else if (!strcmp(tok[0], "cond") && nt >= 3) {
            if (strlen(tok[1]) >= NAME_LEN) { add_error(qs, "name too long '%s'", tok[1]); continue; }
            Cond* c = NULL;
            if (reg_cond(qs, tok[1], &c) != 0) continue;
            int ci = qs->nconds - 1;
            if (!strcmp(tok[2], "counter") && nt == 6 && !strcmp(tok[4], ">=")) {
                int id = quest_counter_id(qs, tok[3]);
                if (id < 0) { add_error(qs, "unknown counter '%s'", tok[3]); continue; }
                c->type = COND_COUNTER_GE;
                c->target = id;
                c->threshold = atol(tok[5]);
            } else if (!strcmp(tok[2], "flag") && nt == 4) {
                int id = quest_flag_id(qs, tok[3]);
                if (id < 0) { add_error(qs, "unknown flag '%s'", tok[3]); continue; }
                c->type = COND_FLAG_SET;
                c->target = id;
            } else if ((!strcmp(tok[2], "and") || !strcmp(tok[2], "or")) && nt >= 4 && nt - 3 <= MAX_CHILDREN) {
                c->type = tok[2][0] == 'a' ? COND_AND : COND_OR;
                c->nchildren = nt - 3;
                for (int i = 0; i < c->nchildren; i++)
                    push_pending(qs, &npending, 0, ci, i, tok[3 + i]);
            } else if (!strcmp(tok[2], "not") && nt == 4) {
                c->type = COND_NOT;
                c->nchildren = 1;
                push_pending(qs, &npending, 0, ci, 0, tok[3]);
            } else {
                add_error(qs, "bad cond line for '%s'", tok[1]);
            }
        } else if (!strcmp(tok[0], "quest") && nt == 3) {
            if (strlen(tok[1]) >= NAME_LEN) { add_error(qs, "name too long '%s'", tok[1]); continue; }
            if (qs->nquests >= MAX_QUESTS) { add_error(qs, "too many quests"); continue; }
            if (quest_find(qs, tok[1]) >= 0) { add_error(qs, "duplicate quest '%s'", tok[1]); continue; }
            Quest* q = &qs->quests[qs->nquests];
            memset(q, 0, sizeof(*q));
            snprintf(q->name, NAME_LEN, "%s", tok[1]);
            push_pending(qs, &npending, 1, qs->nquests, 0, tok[2]);
            qs->nquests++;
        } else {
            add_error(qs, "bad line: '%s'", line);
        }
    }
    free(buf);

    /* 解析引用：缺失引用在此检出 */
    for (int i = 0; i < npending; i++) {
        PendingRef* p = &pending[i];
        int idx = -1;
        for (int j = 0; j < qs->nconds; j++)
            if (!strcmp(qs->conds[j].name, p->name)) { idx = j; break; }
        if (idx < 0) {
            add_error(qs, "missing ref '%s'", p->name);
            continue;
        }
        if (p->is_quest) qs->quests[p->owner].root = idx;
        else qs->conds[p->owner].children[p->slot] = idx;
    }
    if (qs->nerrors) return -1;

    /* 环检测：三色 DFS */
    {
        unsigned char* color = calloc((size_t)qs->nconds, 1);
        if (!color) { add_error(qs, "oom"); return -1; }
        for (int s = 0; s < qs->nconds; s++) {
            /* 迭代式 DFS 避免深递归 */
            int stack[MAX_CONDS];
            int sp = 0;
            if (color[s]) continue;
            stack[sp++] = s;
            while (sp > 0) {
                int ci = stack[sp - 1];
                if (color[ci] == 0) {
                    color[ci] = 1;
                    Cond* c = &qs->conds[ci];
                    for (int i = 0; i < c->nchildren; i++) {
                        int ch = c->children[i];
                        if (color[ch] == 1)
                            add_error(qs, "cycle detected at cond '%s'", qs->conds[ch].name);
                        else if (color[ch] == 0)
                            stack[sp++] = ch;
                    }
                } else {
                    color[ci] = 2;
                    sp--;
                }
            }
        }
        free(color);
    }
    return qs->nerrors ? -1 : 0;
}

/* ================= 自检 ================= */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static const char* DEMO_CFG =
    "counter kills\n"
    "counter deaths\n"
    "counter gold\n"
    "flag vip\n"
    "flag banned\n"
    "cond k10 counter kills >= 10\n"
    "cond d3 counter deaths >= 3\n"
    "cond g100 counter gold >= 100\n"
    "cond fvip flag vip\n"
    "cond fban flag banned\n"
    "cond c_and and k10 fvip\n"
    "cond c_or or d3 g100\n"
    "cond c_not not fban\n"
    "cond c_mix and c_or c_not\n"
    "cond c_top or c_and c_mix\n"
    "cond c_tri and k10 d3 g100\n"
    "quest q_k10 k10\n"
    "quest q_and c_and\n"
    "quest q_or c_or\n"
    "quest q_not c_not\n"
    "quest q_mix c_mix\n"
    "quest q_top c_top\n"
    "quest q_tri c_tri\n";

/* 与 DEMO_CFG 逐条对应的手工判定参考实现 */
static void manual_eval(long kills, long deaths, long gold, int vip, int banned,
                        int done[7], int fire[7]) {
    int k10 = kills >= 10, d3 = deaths >= 3, g100 = gold >= 100;
    int c_and = k10 && vip, c_or = d3 || g100, c_not = !banned;
    int c_mix = c_or && c_not, c_top = c_and || c_mix, c_tri = k10 && d3 && g100;
    int conds[7] = { k10, c_and, c_or, c_not, c_mix, c_top, c_tri };
    for (int i = 0; i < 7; i++)
        if (!done[i] && conds[i]) { done[i] = 1; fire[i]++; }
}

static unsigned rng_state = 0x12345678u;
static unsigned rnd(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

static double now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e6 + ts.tv_nsec / 1e3;
}

int main(void) {
    /* 1. 基本累加 + 完成锁存 + 回滚后不重触发 */
    {
        QuestSys* qs = malloc(sizeof(QuestSys));
        quest_init(qs);
        check("load_demo", quest_load(qs, DEMO_CFG) == 0);
        int kills = quest_counter_id(qs, "kills");
        int q = quest_find(qs, "q_k10");
        quest_add(qs, kills, 5);
        quest_evaluate(qs);
        check("not_done_at_5", !quest_completed(qs, q));
        quest_add(qs, kills, 5);
        check("newly_one", quest_evaluate(qs) == 1);
        check("done_at_10", quest_completed(qs, q));
        check("fired_once", quest_fire_count(qs, q) == 1);
        check("no_newly_again", quest_evaluate(qs) == 0);
        check("undo_ok", quest_undo(qs) == 0);
        check("rolled_back", quest_counter_value(qs, kills) == 5);
        quest_evaluate(qs);
        check("latched_after_rollback", quest_completed(qs, q));
        quest_add(qs, kills, 5);
        quest_evaluate(qs);
        check("no_retrigger", quest_fire_count(qs, q) == 1);
        check("undo_empty", (quest_undo(qs), quest_undo(qs), quest_undo(qs)) == -1);
        check("undone_to_zero", quest_counter_value(qs, kills) == 0);
        quest_add(qs, kills, 3);
        quest_commit(qs);
        check("undo_after_commit", quest_undo(qs) == -1);
        check("committed_value", quest_counter_value(qs, kills) == 3);
        free(qs);
    }

    /* 2. 配置校验：缺失引用 / 成环 / 未知计数器 / 重复名 */
    {
        QuestSys* qs = malloc(sizeof(QuestSys));
        quest_init(qs);
        check("missing_ref_rejected",
              quest_load(qs, "counter a\ncond c1 counter a >= 1\ncond c2 and c1 cX\nquest q c2\n") == -1);
        check("missing_ref_reported", quest_error_count(qs) > 0);

        quest_init(qs);
        check("cycle_rejected",
              quest_load(qs, "cond a and b\ncond b and a\nquest q a\n") == -1);
        quest_init(qs);
        check("self_cycle_rejected",
              quest_load(qs, "cond a and a\nquest q a\n") == -1);
        quest_init(qs);
        check("unknown_counter_rejected",
              quest_load(qs, "cond c counter nope >= 1\nquest q c\n") == -1);
        quest_init(qs);
        check("dup_name_rejected",
              quest_load(qs, "counter a\ncounter a\n") == -1);
        quest_init(qs);
        check("forward_ref_ok",
              quest_load(qs, "counter a\ncond top and leaf\ncond leaf counter a >= 1\nquest q top\n") == 0);
        free(qs);
    }

    /* 3. 随机操作对拍：系统结果必须与逐条手工判定一致 */
    {
        QuestSys* qs = malloc(sizeof(QuestSys));
        quest_init(qs);
        quest_load(qs, DEMO_CFG);
        int cid[3] = { quest_counter_id(qs, "kills"), quest_counter_id(qs, "deaths"), quest_counter_id(qs, "gold") };
        int fid[2] = { quest_flag_id(qs, "vip"), quest_flag_id(qs, "banned") };
        long m_cnt[3] = { 0, 0, 0 };
        int m_flag[2] = { 0, 0 };
        int m_done[7] = { 0 }, m_fire[7] = { 0 };
        enum { OPS = 4000 };
        int mj_c[OPS]; long mj_d[OPS]; int mj_n = 0;
        int consistent = 1, fire_ok = 1;

        for (int step = 0; step < OPS; step++) {
            unsigned r = rnd() % 100;
            if (r < 55) {
                int c = (int)(rnd() % 3);
                long d = (long)(rnd() % 31) - 5; /* -5..25：含负增量直接回退 */
                if (quest_add(qs, cid[c], d) == 0) {
                    m_cnt[c] += d;
                    mj_c[mj_n] = c; mj_d[mj_n] = d; mj_n++;
                }
            } else if (r < 75) {
                int f = (int)(rnd() % 2), v = (int)(rnd() & 1);
                quest_set_flag(qs, fid[f], v);
                m_flag[f] = v;
            } else if (r < 90) {
                if (quest_undo(qs) == 0) {
                    mj_n--;
                    m_cnt[mj_c[mj_n]] -= mj_d[mj_n];
                }
            } else {
                quest_commit(qs);
                mj_n = 0;
            }
            quest_evaluate(qs);
            manual_eval(m_cnt[0], m_cnt[1], m_cnt[2], m_flag[0], m_flag[1], m_done, m_fire);
            for (int i = 0; i < 3; i++)
                if (quest_counter_value(qs, cid[i]) != m_cnt[i]) consistent = 0;
            for (int i = 0; i < 7; i++) {
                if (quest_completed(qs, i) != m_done[i]) consistent = 0;
                if (quest_fire_count(qs, i) != m_fire[i]) consistent = 0;
                if (quest_fire_count(qs, i) > 1) fire_ok = 0;
            }
        }
        check("consistent_with_manual", consistent);
        check("fire_at_most_once", fire_ok);
        free(qs);
    }

    /* 4. 基准：1000 条任务全量求值耗时 */
    {
        enum { NQ = 1000, NC = 50 };
        char* cfg = malloc(1 << 20);
        size_t off = 0;
        for (int j = 0; j < NC; j++) off += sprintf(cfg + off, "counter c%d\n", j);
        for (int i = 0; i < NQ; i++) off += sprintf(cfg + off, "cond k%d counter c%d >= %d\n", i, i % NC, (i % NC + 1) * 10);
        for (int i = 0; i < NQ; i++) off += sprintf(cfg + off, "cond a%d and k%d k%d\n", i, i, (i * 8) % NQ);
        for (int i = 0; i < NQ; i++) off += sprintf(cfg + off, "quest q%d a%d\n", i, i);

        QuestSys* qs = malloc(sizeof(QuestSys));
        quest_init(qs);
        check("bench_load", quest_load(qs, cfg) == 0);
        check("bench_count", quest_count(qs) == NQ);

        /* 满负荷单遍：所有计数器达标，1000 条任务在同一遍内全部完成（AND 两个子节点都求值）。
         * 每遍重新加载配置，只计时 quest_evaluate 本身。 */
        int reps = 100;
        double full_us = 0.0;
        for (int r = 0; r < reps; r++) {
            quest_init(qs);
            quest_load(qs, cfg);
            for (int j = 0; j < NC; j++) {
                char nm[NAME_LEN];
                snprintf(nm, NAME_LEN, "c%d", j);
                quest_add(qs, quest_counter_id(qs, nm), (j + 1) * 10);
            }
            double t0 = now_us();
            quest_evaluate(qs);
            full_us += now_us() - t0;
        }
        full_us /= reps;

        /* 手工核算：所有计数器达标时每个叶子都成立，1000 条应全部完成 */
        int completed = 0;
        for (int i = 0; i < NQ; i++) completed += quest_completed(qs, i);
        check("bench_matches_manual", completed == NQ);

        /* 稳态：全部锁存后再求值应接近零开销 */
        int warm_reps = 200;
        double t0 = now_us();
        for (int r = 0; r < warm_reps; r++) quest_evaluate(qs);
        double warm_us = (now_us() - t0) / warm_reps;

        printf("bench_quests=%d\n", NQ);
        printf("bench_completed=%d\n", completed);
        printf("bench_full_pass_us=%.1f\n", full_us);
        printf("bench_warm_pass_us=%.1f\n", warm_us);
        free(qs);
        free(cfg);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
