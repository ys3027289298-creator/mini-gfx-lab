
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget,
   emitter priority scheduling and frame-time driven load shedding.

   降载策略：
   - 帧时间超阈值时逐级降低发射倍率与更新频率，每个超阈值帧恰好记一次降载；
   - 帧时间回到阈值内后每帧只回升一级，逐步恢复、无突变；
   - 更新降载仅跳过更新时机，dt 累加后一次性补算，已有粒子的轨迹不变；
   - over_frames 与 shed_frames 同步累加，降载次数 == 超阈值帧数。 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_PARTICLES 4096
#define MAX_EMITTERS 8
#define SHED_LEVELS 8

typedef struct { float x, y, vx, vy, life, age; int alive; int tag; } Particle;
typedef struct { int id; int quota; int spawned; int priority; } Emitter;

typedef struct {
    Particle pool[MAX_PARTICLES];
    int alive_count, high_water;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;
} World;

/* 降载等级表：发射倍率逐级下降，更新步长逐级翻倍（每 stride 帧补算一次） */
static const float shed_emit_scale[SHED_LEVELS] = {
    1.00f, 0.85f, 0.70f, 0.55f, 0.40f, 0.28f, 0.18f, 0.10f
};
static const int shed_update_stride[SHED_LEVELS] = {
    1, 1, 2, 2, 4, 4, 8, 8
};

typedef struct {
    float threshold;      /* 帧时间阈值（秒） */
    int level;            /* 当前降载等级，0 = 满载 */
    int frame_index;      /* 已观测帧数，决定补算时机 */
    int over_frames;      /* 超阈值帧数 */
    int shed_frames;      /* 降载帧数，始终与 over_frames 一致 */
    float pending_dt;     /* 跳帧期间累加的 dt，补算前不清空 */
} LoadShed;

static void world_init(World* w) {
    memset(w, 0, sizeof(*w));
    w->emitter_count = 0;
}

static int emitter_add(World* w, int id, int quota, int priority) {
    if (w->emitter_count >= MAX_EMITTERS) return -1;
    Emitter* e = &w->emitters[w->emitter_count++];
    e->id = id; e->quota = quota; e->spawned = 0; e->priority = priority;
    return e->id;
}

static int pool_find_free(World* w) {
    for (int i = 0; i < MAX_PARTICLES; i++) if (!w->pool[i].alive) return i;
    return -1;
}

static int spawn_one(World* w, int emitter_id) {
    int idx = pool_find_free(w);
    if (idx < 0) return -1;
    Particle* p = &w->pool[idx];
    p->alive = 1; p->age = 0.0f;
    p->x = 0.0f; p->y = 0.0f;
    p->vx = 1.0f; p->vy = 0.5f;
    p->life = 2.0f;
    p->tag = emitter_id;
    w->alive_count++;
    if (w->alive_count > w->high_water) w->high_water = w->alive_count;
    return idx;
}

/* highest priority emitters spend their quota first */
static void spawn_frame(World* w, int budget) {
    int left = budget;
    int order[MAX_EMITTERS]; int n = w->emitter_count;
    for (int i = 0; i < n; i++) order[i] = i;
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++)
            if (w->emitters[order[j]].priority > w->emitters[order[i]].priority) {
                int t = order[i]; order[i] = order[j]; order[j] = t;
            }
    for (int k = 0; k < n && left > 0; k++) {
        Emitter* e = &w->emitters[order[k]];
        e->spawned = 0;
        int want = e->quota < left ? e->quota : left;
        for (int i = 0; i < want; i++) if (spawn_one(w, e->id) >= 0) { e->spawned++; left--; }
    }
}

static void update(World* w, float dt) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive) continue;
        p->age += dt;
        p->x += p->vx * dt; p->y += p->vy * dt;
        if (p->age >= p->life) { p->alive = 0; w->alive_count--; }
    }
}

static void shed_init(LoadShed* s, float threshold) {
    memset(s, 0, sizeof(*s));
    s->threshold = threshold;
}

/* 观测一帧的帧时间并更新等级，返回当前降载等级。
   每帧等级至多变化一级：超阈值升级（饱和后仍计降载），恢复时降级。 */
static int shed_observe(LoadShed* s, float frame_time) {
    s->frame_index++;
    if (frame_time > s->threshold) {
        s->over_frames++;
        s->shed_frames++;          /* 每个超阈值帧恰好记一次降载 */
        if (s->level < SHED_LEVELS - 1) s->level++;
    } else if (s->level > 0) {
        s->level--;                /* 恢复每帧最多一级，倍率无突变 */
    }
    return s->level;
}

/* 发射降载：基础预算乘当前等级倍率，只影响新粒子 */
static int shed_budget(const LoadShed* s, int base_budget) {
    return (int)(base_budget * shed_emit_scale[s->level]);
}

/* 降载更新：跳过的帧只累加 dt，到补算时机一次性积分。
   线性积分下 x += vx*(dt1+dt2) 与逐帧结果路径一致，轨迹不变。
   注意：每帧须先 shed_observe 再调用本函数。 */
static void shed_update(World* w, LoadShed* s, float dt) {
    s->pending_dt += dt;
    if (s->frame_index % shed_update_stride[s->level] == 0) {
        update(w, s->pending_dt);
        s->pending_dt = 0.0f;
    }
}

/* 模拟结束时补齐残留的累加时间，便于在同一时刻对比两个世界 */
static void shed_flush(World* w, LoadShed* s) {
    if (s->pending_dt > 0.0f) {
        update(w, s->pending_dt);
        s->pending_dt = 0.0f;
    }
}

/* 对比两个世界同槽位粒子：存活标志一致，位置/年龄误差在 eps 内 */
static int worlds_agree(const World* a, const World* b, float eps, float* max_diff) {
    int ok = (a->alive_count == b->alive_count);
    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle* p = &a->pool[i];
        const Particle* q = &b->pool[i];
        if (p->alive != q->alive) { ok = 0; continue; }
        if (!p->alive) continue;
        float d = fabsf(p->x - q->x);
        if (d > *max_diff) *max_diff = d;
        if (fabsf(p->y - q->y) > d) d = fabsf(p->y - q->y);
        if (fabsf(p->age - q->age) > d) d = fabsf(p->age - q->age);
        if (d > *max_diff) *max_diff = d;
        if (d > eps) ok = 0;
    }
    return ok;
}

int main(void) {
    /* ---- 原有：配额与发射器优先级 ---- */
    World w; world_init(&w);
    emitter_add(&w, 1, 40, 2);
    emitter_add(&w, 2, 30, 9);
    emitter_add(&w, 3, 20, 5);
    for (int frame = 0; frame < 10; frame++) {
        spawn_frame(&w, 50);
        update(&w, 0.1f);
    }
    printf("alive=%d high_water=%d\n", w.alive_count, w.high_water);
    int tag1 = 0, tag2 = 0, tag3 = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (w.pool[i].alive) {
        if (w.pool[i].tag == 1) tag1++; else if (w.pool[i].tag == 2) tag2++; else tag3++;
    }
    printf("by_emitter=%d,%d,%d\n", tag1, tag2, tag3);
    /* quota 优先：高优先级 2 应先拿满 30 */
    printf("priority_respected=%d\n", tag2 >= tag3 && tag3 >= 0);

    /* ---- 降载：脚本化帧时间序列 ----
       0..3 正常，4..13 超阈值（10 帧，超过等级数以验证饱和计数），
       14..25 回阈值内，观察逐步恢复。 */
    LoadShed s; shed_init(&s, 0.020f);
    const float normal_ft = 0.016f, bad_ft = 0.033f;
    int levels[32] = {0};
    int bad_frames = 0, peak_level = 0;
    for (int frame = 0; frame < 26; frame++) {
        float ft = (frame >= 4 && frame < 14) ? bad_ft : normal_ft;
        if (ft > s.threshold) bad_frames++;
        levels[frame] = shed_observe(&s, ft);
        if (levels[frame] > peak_level) peak_level = levels[frame];
    }
    /* 降载次数与超阈值帧数一致，饱和不丢计数 */
    printf("over=%d shed=%d bad=%d\n", s.over_frames, s.shed_frames, bad_frames);
    printf("shed_count_matches=%d\n",
           s.over_frames == bad_frames && s.shed_frames == s.over_frames);
    printf("peak_level=%d (capped at %d)\n", peak_level, SHED_LEVELS - 1);

    /* 每帧等级变化至多一级：升降级均无突变 */
    int step_bounded = 1;
    for (int frame = 1; frame < 26; frame++) {
        int d = levels[frame] - levels[frame - 1];
        if (d < -1 || d > 1) step_bounded = 0;
    }
    printf("level_step_bounded=%d\n", step_bounded);

    /* 恢复段（frame 14 起）：倍率单调回升、单步有界、需多帧才回到 1.0 */
    int recovery_frames = 0, gradual = 1;
    float prev_scale = 0.0f;
    for (int frame = 14; frame < 26 && levels[frame] > 0; frame++) {
        float scale = shed_emit_scale[levels[frame]];
        if (recovery_frames > 0) {
            float jump = scale - prev_scale;
            if (jump <= 0.0f || jump > 0.16f) gradual = 0;
        }
        prev_scale = scale;
        recovery_frames++;
    }
    /* 恢复首帧不得直接满载，且整个恢复跨多帧 */
    int first_recovery_scale = (int)(shed_emit_scale[levels[14]] * 100.0f);
    printf("recovery_frames=%d first_scale=0.%02d\n",
           recovery_frames, first_recovery_scale);
    printf("recovery_gradual=%d\n",
           gradual && recovery_frames > 1 && first_recovery_scale < 100);

    /* 发射预算随等级降低，峰值等级下预算显著收缩 */
    int full_budget = shed_budget(&(LoadShed){.threshold = 0.020f, .level = 0}, 50);
    LoadShed peak = s; peak.level = peak_level;
    int peak_budget = shed_budget(&peak, 50);
    printf("budget full=%d peak=%d\n", full_budget, peak_budget);
    printf("budget_reduced=%d\n", peak_budget < full_budget && full_budget == 50);

    /* ---- 轨迹不变性：A 逐帧全量更新，B 走降载补算路径 ---- */
    World a, b; world_init(&a); world_init(&b);
    LoadShed sb; shed_init(&sb, 0.020f);
    spawn_frame(&a, 64);            /* 降载前预生成相同粒子 */
    spawn_frame(&b, 64);
    const float dt = 0.1f;
    int flush_checks = 0, trajectory_ok = 1;
    float max_diff = 0.0f;
    for (int frame = 0; frame < 20; frame++) {
        shed_observe(&sb, bad_ft);  /* 持续施压，stride 逐级拉大 */
        update(&a, dt);
        shed_update(&b, &sb, dt);
        if (sb.pending_dt == 0.0f) {   /* B 本帧补算，两世界处于同一模拟时刻 */
            flush_checks++;
            if (!worlds_agree(&a, &b, 1e-5f, &max_diff)) trajectory_ok = 0;
        }
    }
    shed_flush(&b, &sb);            /* 补齐残留时间后做终态对比（含死亡时刻） */
    if (!worlds_agree(&a, &b, 1e-5f, &max_diff)) trajectory_ok = 0;
    printf("flush_checks=%d max_diff=%.2e alive=%d,%d\n",
           flush_checks, max_diff, a.alive_count, b.alive_count);
    /* 降载只改变更新采样密度：位置/年龄一致，死亡时刻同步 */
    printf("trajectory_unchanged=%d\n",
           trajectory_ok && flush_checks > 1 && a.alive_count == b.alive_count);

    return 0;
}
