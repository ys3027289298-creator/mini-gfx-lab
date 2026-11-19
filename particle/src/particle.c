
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget,
   emitter priority scheduling, and same-source effect stacking rules
   (stack / merge / replace / reject) with priority-based eviction. */
#include <stdio.h>
#include <string.h>

#define MAX_PARTICLES 4096
#define MAX_EMITTERS 8
#define MAX_RULES 16
#define MAX_EFFECTS 32
#define FADE_TIME 0.5f      /* 替换/驱逐时旧特效的优雅退场时长 */
#define PARTICLE_LIFE 2.0f

/* 返回码 */
#define OK 0
#define ERR_FULL -1              /* 池/表满，或没有可被覆盖的低优先级特效 */
#define ERR_UNKNOWN_SOURCE -2    /* 来源未注册叠加规则 */
#define ERR_RULE_CONFLICT -3     /* 同源重复注册的规则不一致 */
#define ERR_BAD_RULE -4          /* 非法规则枚举或参数 */
#define ERR_REJECTED -5          /* REJECT 规则下同源特效仍活跃 */

typedef enum {
    RULE_STACK = 0,   /* 同源共存：每次施放都生成新实例 */
    RULE_MERGE,       /* 同源合并：只保留一个实例，粒子补齐到上限 */
    RULE_REPLACE,     /* 同源替换：旧实例优雅退场，新实例接管 */
    RULE_REJECT       /* 同源活跃期间拒绝新施放 */
} StackRule;

typedef enum { INST_ACTIVE = 0, INST_FADING } InstState;

typedef struct {
    float x, y, vx, vy, life, age;
    int alive;
    int tag;       /* 发射器 id；特效粒子为 -1 */
    int effect;    /* 特效实例 id；发射器粒子为 0 */
} Particle;
typedef struct { int id; int quota; int spawned; int priority; } Emitter;

typedef struct {
    int source;     /* 技能/特效来源 id */
    int rule;       /* StackRule */
    int priority;   /* 池满时的覆盖优先级，大者覆盖小者 */
    int burst;      /* 每次施放生成的粒子数 */
    int max_alive;  /* MERGE 时的同源存活上限 */
    int used;
} EffectRule;

typedef struct {
    int id;
    int source;
    int priority;
    int state;      /* INST_ACTIVE / INST_FADING */
    int alive;      /* 该实例仍存活的粒子数 */
} EffectInst;

typedef struct {
    Particle pool[MAX_PARTICLES];
    int alive_count, high_water;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;
    EffectRule rules[MAX_RULES];
    int rule_count;
    EffectInst effects[MAX_EFFECTS];
    int effect_count;
    int next_effect_id;
} World;

static void world_init(World* w) {
    memset(w, 0, sizeof(*w));
    w->next_effect_id = 1;
}

/* ---------- 发射器（原有配额 + 优先级调度） ---------- */

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
    p->life = PARTICLE_LIFE;
    p->tag = emitter_id; p->effect = 0;
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

/* ---------- 叠加规则注册 ---------- */

static int rule_register(World* w, int source, int rule, int priority, int burst, int max_alive) {
    if (rule < RULE_STACK || rule > RULE_REJECT) return ERR_BAD_RULE;
    if (burst <= 0 || max_alive <= 0 || max_alive > MAX_PARTICLES) return ERR_BAD_RULE;
    for (int i = 0; i < w->rule_count; i++) {
        if (w->rules[i].source != source) continue;
        EffectRule* r = &w->rules[i];
        if (r->rule == rule && r->priority == priority &&
            r->burst == burst && r->max_alive == max_alive)
            return OK;   /* 幂等重复注册 */
        return ERR_RULE_CONFLICT;
    }
    if (w->rule_count >= MAX_RULES) return ERR_FULL;
    EffectRule* r = &w->rules[w->rule_count++];
    r->source = source; r->rule = rule; r->priority = priority;
    r->burst = burst; r->max_alive = max_alive; r->used = 0;
    return OK;
}

static int rule_find(World* w, int source) {
    for (int i = 0; i < w->rule_count; i++)
        if (w->rules[i].source == source) return i;
    return -1;
}

/* ---------- 特效实例 ---------- */

static EffectInst* inst_find(World* w, int id) {
    for (int i = 0; i < w->effect_count; i++)
        if (w->effects[i].id == id) return &w->effects[i];
    return NULL;
}

static int inst_find_active(World* w, int source) {
    for (int i = 0; i < w->effect_count; i++)
        if (w->effects[i].source == source && w->effects[i].state == INST_ACTIVE)
            return i;
    return -1;
}

static int inst_create(World* w, const EffectRule* r) {
    if (w->effect_count >= MAX_EFFECTS) return ERR_FULL;
    EffectInst* in = &w->effects[w->effect_count++];
    in->id = w->next_effect_id++;
    in->source = r->source;
    in->priority = r->priority;
    in->state = INST_ACTIVE;
    in->alive = 0;
    return in->id;
}

static void particle_release(World* w, int idx) {
    Particle* p = &w->pool[idx];
    if (!p->alive) return;
    EffectInst* in = inst_find(w, p->effect);
    if (in) in->alive--;
    p->alive = 0;
    p->effect = 0;
    w->alive_count--;
}

/* 优雅退场：实例转入 FADING，剩余寿命截断到 FADE_TIME，而不是瞬杀 */
static void inst_retire(World* w, int slot) {
    EffectInst* in = &w->effects[slot];
    if (in->state == INST_FADING) return;
    in->state = INST_FADING;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive || p->effect != in->id) continue;
        float remain = p->life - p->age;
        if (remain > FADE_TIME) p->life = p->age + FADE_TIME;
    }
}

/* 从退场中的实例回收最老的一个粒子腾位 */
static int reclaim_fading(World* w) {
    int best = -1;
    float oldest = -1.0f;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive || p->effect == 0) continue;
        EffectInst* in = inst_find(w, p->effect);
        if (in && in->state == INST_FADING && p->age > oldest) {
            oldest = p->age;
            best = i;
        }
    }
    if (best >= 0) particle_release(w, best);
    return best;
}

/* 优先级覆盖：淘汰优先级严格更低的最弱活跃实例（同级不可互盖） */
static int evict_lower(World* w, int priority) {
    int slot = -1;
    for (int i = 0; i < w->effect_count; i++) {
        EffectInst* in = &w->effects[i];
        if (in->state != INST_ACTIVE || in->alive == 0 || in->priority >= priority)
            continue;
        if (slot < 0 || in->priority < w->effects[slot].priority ||
            (in->priority == w->effects[slot].priority && in->id < w->effects[slot].id))
            slot = i;
    }
    if (slot < 0) return -1;
    inst_retire(w, slot);
    return reclaim_fading(w);
}

static int effect_spawn(World* w, EffectInst* in, int count) {
    int done = 0;
    for (int i = 0; i < count; i++) {
        int idx = pool_find_free(w);
        if (idx < 0) idx = reclaim_fading(w);          /* 先让出退场中的槽位 */
        if (idx < 0) idx = evict_lower(w, in->priority); /* 再覆盖低优先级 */
        if (idx < 0) break;                            /* 无可覆盖对象：放弃 */
        Particle* p = &w->pool[idx];
        p->alive = 1; p->age = 0.0f;
        p->x = 0.0f; p->y = 0.0f;
        p->vx = 1.0f; p->vy = 0.5f;
        p->life = PARTICLE_LIFE;
        p->tag = -1; p->effect = in->id;
        in->alive++;
        w->alive_count++;
        if (w->alive_count > w->high_water) w->high_water = w->alive_count;
        done++;
    }
    return done;
}

/* 施放一次同源特效，按注册的叠加规则处理 */
static int effect_cast(World* w, int source) {
    int ri = rule_find(w, source);
    if (ri < 0) return ERR_UNKNOWN_SOURCE;
    EffectRule* r = &w->rules[ri];

    int slot = inst_find_active(w, source);
    if (slot >= 0) {
        EffectInst* old = &w->effects[slot];
        switch (r->rule) {
        case RULE_REJECT:
            return ERR_REJECTED;
        case RULE_MERGE: {
            int room = r->max_alive - old->alive;
            if (room > 0)
                effect_spawn(w, old, room < r->burst ? room : r->burst);
            return old->id;   /* 合并到原实例，不新建 */
        }
        case RULE_REPLACE:
            inst_retire(w, slot);   /* 旧特效优雅退场 */
            slot = -1;
            break;
        case RULE_STACK:
        default:
            break;                  /* 落到下方新建实例 */
        }
    }

    int id = inst_create(w, r);
    if (id < 0) return id;
    EffectInst* in = inst_find(w, id);
    int want = (r->rule == RULE_MERGE && r->burst > r->max_alive) ? r->max_alive : r->burst;
    int spawned = effect_spawn(w, in, want);
    if (spawned == 0) {   /* 无槽可抢且无可覆盖对象：撤掉空实例并报错 */
        w->effect_count--;
        return ERR_FULL;
    }
    return id;
}

/* ---------- 每帧推进：过期自动回收 + 空实例回收 ---------- */

static void update(World* w, float dt) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive) continue;
        p->age += dt;
        p->x += p->vx * dt; p->y += p->vy * dt;
        if (p->age >= p->life) particle_release(w, i);
    }
    for (int i = w->effect_count - 1; i >= 0; i--) {
        if (w->effects[i].alive == 0) {
            w->effects[i] = w->effects[w->effect_count - 1];
            w->effect_count--;
        }
    }
}

/* ---------- 测试用查询 ---------- */

static int source_alive_particles(World* w, int source) {
    int n = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive || p->effect == 0) continue;
        EffectInst* in = inst_find(w, p->effect);
        if (in && in->source == source) n++;
    }
    return n;
}

static int source_inst_in_state(World* w, int source, int state) {
    int n = 0;
    for (int i = 0; i < w->effect_count; i++)
        if (w->effects[i].source == source && w->effects[i].state == state)
            n++;
    return n;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    /* --- 原有行为：固定池 + 每帧配额 + 发射器优先级 --- */
    World w; world_init(&w);
    emitter_add(&w, 1, 40, 2);
    emitter_add(&w, 2, 30, 9);
    emitter_add(&w, 3, 20, 5);
    for (int frame = 0; frame < 10; frame++) {
        spawn_frame(&w, 50);
        update(&w, 0.1f);
    }
    int tag1 = 0, tag2 = 0, tag3 = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (w.pool[i].alive) {
        if (w.pool[i].tag == 1) tag1++;
        else if (w.pool[i].tag == 2) tag2++;
        else if (w.pool[i].tag == 3) tag3++;
    }
    check("emitter_budget_total", w.alive_count == 500);
    check("emitter_high_priority_first", tag2 == 300 && tag3 == 200 && tag1 == 0);

    /* --- MERGE：同源合并，存活数严格等于上限，不叠出第二份 --- */
    World m; world_init(&m);
    check("merge_register", rule_register(&m, 1, RULE_MERGE, 5, 60, 100) == OK);
    int m1 = effect_cast(&m, 1);
    update(&m, 0.1f);
    int m2 = effect_cast(&m, 1);
    check("merge_same_instance", m1 == m2);
    check("merge_caps_at_max", m.alive_count == 100 && source_alive_particles(&m, 1) == 100);
    check("merge_single_instance", m.effect_count == 1);
    int m3 = effect_cast(&m, 1);   /* 已满：补 0 粒，仍是同一实例 */
    check("merge_full_no_growth", m3 == m1 && m.alive_count == 100);

    /* --- REPLACE：新实例接管，旧实例优雅退场而非瞬杀 --- */
    World r; world_init(&r);
    check("replace_register", rule_register(&r, 2, RULE_REPLACE, 5, 60, 60) == OK);
    effect_cast(&r, 2);
    update(&r, 0.1f);
    effect_cast(&r, 2);
    check("replace_old_kept_fading", r.alive_count == 120
        && source_inst_in_state(&r, 2, INST_ACTIVE) == 1
        && source_inst_in_state(&r, 2, INST_FADING) == 1);
    for (int i = 0; i < 5; i++) update(&r, 0.1f);  /* 退场寿命截断为 0.5s */
    check("replace_fade_recycled", r.alive_count == 60 && r.effect_count == 1
        && source_inst_in_state(&r, 2, INST_FADING) == 0);
    for (int i = 0; i < 15; i++) update(&r, 0.1f); /* 新实例自然过期 */
    check("replace_expire_recycle", r.alive_count == 0 && r.effect_count == 0);

    /* --- STACK：无合并，每次施放独立共存 --- */
    World s; world_init(&s);
    check("stack_register", rule_register(&s, 3, RULE_STACK, 5, 40, 40) == OK);
    effect_cast(&s, 3);
    effect_cast(&s, 3);
    check("stack_two_instances", s.effect_count == 2 && s.alive_count == 80);

    /* --- REJECT：同源活跃期拒绝新施放 --- */
    World j; world_init(&j);
    check("reject_register", rule_register(&j, 4, RULE_REJECT, 5, 10, 10) == OK);
    check("reject_first_cast", effect_cast(&j, 4) >= 0);
    check("reject_second_cast", effect_cast(&j, 4) == ERR_REJECTED);
    check("reject_keeps_single", j.alive_count == 10 && j.effect_count == 1);

    /* --- 优先级覆盖：高优先级可驱逐低优先级，反向不行 --- */
    World p; world_init(&p);
    rule_register(&p, 10, RULE_MERGE, 1, 256, MAX_PARTICLES); /* 低优先级，单实例即可填满池 */
    rule_register(&p, 11, RULE_MERGE, 9, 64, MAX_PARTICLES);  /* 高优先级 */
    rule_register(&p, 12, RULE_MERGE, 2, 256, MAX_PARTICLES); /* 中间优先级 */
    rule_register(&p, 13, RULE_STACK, 1, 50, 50);             /* 最低优先级，用于反向验证 */
    while (p.alive_count < MAX_PARTICLES) {
        int rc = effect_cast(&p, 10);
        if (rc < 0) break;
    }
    check("pool_filled_by_low", p.alive_count == MAX_PARTICLES);
    check("high_evicts_low", effect_cast(&p, 11) >= 0
        && source_alive_particles(&p, 11) == 64
        && source_inst_in_state(&p, 10, INST_FADING) == 1
        && p.alive_count == MAX_PARTICLES);
    for (int i = 0; i < 6; i++) update(&p, 0.1f);    /* 退场实例整体寿命截断为 0.5s */
    check("fade_frees_slots", p.alive_count == 64
        && source_inst_in_state(&p, 10, INST_FADING) == 0);
    while (p.alive_count < MAX_PARTICLES) {          /* 中间优先级补齐空槽 */
        int rc = effect_cast(&p, 12);
        if (rc < 0) break;
    }
    check("low_refills_pool", p.alive_count == MAX_PARTICLES
        && source_alive_particles(&p, 12) == MAX_PARTICLES - 64);
    check("low_cannot_evict", effect_cast(&p, 13) == ERR_FULL
        && source_alive_particles(&p, 13) == 0
        && p.alive_count == MAX_PARTICLES);
    check("high_evicts_again", effect_cast(&p, 11) >= 0
        && source_alive_particles(&p, 11) == 128
        && source_inst_in_state(&p, 12, INST_FADING) == 1
        && p.alive_count == MAX_PARTICLES);

    /* --- 规则冲突与非法输入必须报错 --- */
    World c; world_init(&c);
    check("conflict_register_ok", rule_register(&c, 20, RULE_MERGE, 5, 10, 50) == OK);
    check("conflict_idempotent", rule_register(&c, 20, RULE_MERGE, 5, 10, 50) == OK);
    check("conflict_rule_changed", rule_register(&c, 20, RULE_REPLACE, 5, 10, 50) == ERR_RULE_CONFLICT);
    check("conflict_priority_changed", rule_register(&c, 20, RULE_MERGE, 6, 10, 50) == ERR_RULE_CONFLICT);
    check("bad_rule_enum", rule_register(&c, 21, 99, 5, 10, 50) == ERR_BAD_RULE);
    check("bad_rule_args", rule_register(&c, 21, RULE_STACK, 5, 0, 50) == ERR_BAD_RULE);
    check("unknown_source", effect_cast(&c, 999) == ERR_UNKNOWN_SOURCE);

    /* --- 过期自动回收 + 槽位复用 --- */
    World e; world_init(&e);
    rule_register(&e, 30, RULE_STACK, 5, 10, 10);
    effect_cast(&e, 30);
    for (int i = 0; i < 20; i++) update(&e, 0.1f);
    check("expire_all_recycled", e.alive_count == 0 && e.effect_count == 0 && e.high_water == 10);
    check("slots_reused", effect_cast(&e, 30) >= 0 && e.alive_count == 10);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
