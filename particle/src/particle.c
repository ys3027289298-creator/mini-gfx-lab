
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget,
   emitter priority scheduling, and prewarm (预热) support.

   预热设计：
   - 随机源唯一：World 内嵌 xorshift32，预热与正式运行共用同一份状态，
     预热消耗掉的随机数不会重演，序列自然连续；
   - 路径唯一：预热和正式帧都走同一个 frame()（发射+更新），
     预热 N 步得到的状态与连续运行 N 帧逐字节一致；
   - 回调抑制：prewarming 标志置位期间不触发 on_spawn/on_death。 */
#define _POSIX_C_SOURCE 199309L /* clock_gettime */
#include <stdio.h>
#include <string.h>
#include <time.h>

#define MAX_PARTICLES 4096
#define MAX_EMITTERS 8

typedef struct { float x, y, vx, vy, life, age; int alive; int tag; } Particle;
typedef struct { int id; int quota; int spawned; int priority; } Emitter;

typedef struct World World;
typedef void (*ParticleCallback)(World* w, int emitter_id);

struct World {
    Particle pool[MAX_PARTICLES];
    int alive_count, high_water;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;
    unsigned rng;            /* 模拟内唯一随机源，预热与正式运行共用 */
    int prewarming;          /* 非 0 时抑制外部回调 */
    ParticleCallback on_spawn, on_death;
    long spawn_events, death_events; /* 测试用回调计数 */
};

static void world_init(World* w, unsigned seed) {
    memset(w, 0, sizeof(*w));
    w->emitter_count = 0;
    w->rng = seed ? seed : 0x9E3779B9u; /* xorshift 状态不能为 0 */
}

static unsigned rng_next(World* w) {
    unsigned x = w->rng;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    w->rng = x;
    return x;
}

static float rng_float(World* w) { /* [0, 1) */
    return (rng_next(w) >> 8) * (1.0f / 16777216.0f);
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
    p->vx = 0.5f + rng_float(w);   /* 0.5 .. 1.5 */
    p->vy = 0.5f * rng_float(w);   /* 0.0 .. 0.5 */
    p->life = 1.5f + rng_float(w); /* 1.5 .. 2.5 */
    p->tag = emitter_id;
    w->alive_count++;
    if (w->alive_count > w->high_water) w->high_water = w->alive_count;
    if (!w->prewarming && w->on_spawn) w->on_spawn(w, emitter_id);
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
        if (p->age >= p->life) {
            p->alive = 0; w->alive_count--;
            if (!w->prewarming && w->on_death) w->on_death(w, p->tag);
        }
    }
}

/* 一个普通帧：先按预算发射，再推进模拟。
   预热复用同一步骤，保证预热 N 步与连续运行 N 帧状态完全相同。 */
static void frame(World* w, float dt, int budget) {
    spawn_frame(w, budget);
    update(w, dt);
}

/* 预热：按 dt 步长先把模拟跑 duration 秒，期间不触发外部回调。
   RNG 状态随预热推进，预热结束后的随机序列与连续运行自然衔接。 */
static void world_prewarm(World* w, float duration, float dt, int budget) {
    int steps = (int)(duration / dt + 0.5f);
    int prev = w->prewarming;
    w->prewarming = 1;
    for (int i = 0; i < steps; i++) frame(w, dt, budget);
    w->prewarming = prev;
}

/* ---- test ---- */

static void setup_emitters(World* w) {
    emitter_add(w, 1, 40, 2);
    emitter_add(w, 2, 30, 9);
    emitter_add(w, 3, 20, 5);
}

static void count_spawn(World* w, int id) { (void)id; w->spawn_events++; }
static void count_death(World* w, int id) { (void)id; w->death_events++; }

static int worlds_equal(const World* a, const World* b) {
    return a->alive_count == b->alive_count
        && a->high_water == b->high_water
        && a->rng == b->rng
        && memcmp(a->pool, b->pool, sizeof a->pool) == 0;
}

static double now_sec(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

int main(void) {
    World w; world_init(&w, 12345);
    setup_emitters(&w);
    for (int f = 0; f < 10; f++) frame(&w, 0.1f, 50);
    printf("alive=%d high_water=%d\n", w.alive_count, w.high_water);
    int tag1 = 0, tag2 = 0, tag3 = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (w.pool[i].alive) {
        if (w.pool[i].tag == 1) tag1++; else if (w.pool[i].tag == 2) tag2++; else tag3++;
    }
    printf("by_emitter=%d,%d,%d\n", tag1, tag2, tag3);
    /* quota 优先：高优先级 2 应先拿满 30 */
    printf("priority_respected=%d\n", tag2 >= tag3 && tag3 >= 0);

    /* 预热一致性：A 预热 5s(50 步)再跑 20 帧，B 连续跑 70 帧，
       最终粒子分布与 RNG 状态必须完全一致 */
    World a, b;
    world_init(&a, 777); world_init(&b, 777);
    setup_emitters(&a); setup_emitters(&b);
    a.on_spawn = count_spawn; a.on_death = count_death;
    b.on_spawn = count_spawn; b.on_death = count_death;
    world_prewarm(&a, 5.0f, 0.1f, 50);
    long prewarm_spawn_events = a.spawn_events;
    long prewarm_death_events = a.death_events;
    for (int i = 0; i < 20; i++) frame(&a, 0.1f, 50);
    for (int i = 0; i < 70; i++) frame(&b, 0.1f, 50);
    printf("prewarm_matches_continuous=%d\n", worlds_equal(&a, &b));
    printf("prewarm_rng_continuous=%d\n", a.rng == b.rng);
    printf("prewarm_no_callbacks=%d\n",
           prewarm_spawn_events == 0 && prewarm_death_events == 0);
    printf("callbacks_fire_when_live=%d\n", b.spawn_events > 0 && b.death_events > 0);

    /* 预热耗时：50 步预热不得超过一次普通帧耗时的 100 倍 */
    World c; world_init(&c, 999); setup_emitters(&c);
    for (int i = 0; i < 100; i++) frame(&c, 0.1f, 50); /* 进入稳态 */
    double t0 = now_sec();
    for (int i = 0; i < 200; i++) frame(&c, 0.1f, 50);
    double frame_avg = (now_sec() - t0) / 200.0;

    World d; world_init(&d, 999); setup_emitters(&d);
    double p0 = now_sec();
    world_prewarm(&d, 5.0f, 0.1f, 50);
    double prewarm_t = now_sec() - p0;
    printf("prewarm_cost=%.1fx limit=100x ok=%d\n",
           prewarm_t / frame_avg, prewarm_t <= frame_avg * 100.0);
    return 0;
}
