
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget,
   emitter priority scheduling, and oldest-first eviction.

   Storage:
     - free list: singletons in pool[], allocation/return are O(1).
     - age list: intrusive doubly-linked list over live particles, ordered by
       spawn sequence (head = oldest, tail = newest). Unlink on death and
       unlink-head on eviction are O(1), so a full pool never drops a spawn
       request and never scans for a victim. */
#include <stdio.h>
#include <string.h>
#include <time.h>

#define MAX_PARTICLES 10000
#define MAX_EMITTERS 8
#define NO_SLOT (-1)

enum { POOL_REJECT_ON_FULL = 0, POOL_EVICT_OLDEST = 1 };

typedef struct {
    float x, y, vx, vy, life, age;
    int alive;
    int tag;
    int age_prev, age_next;   /* age list links (oldest .. newest) */
    int free_next;            /* free list link, only valid when !alive */
    long long spawn_seq;
} Particle;

typedef struct { int id; int quota; int spawned; int priority; } Emitter;

typedef struct {
    Particle pool[MAX_PARTICLES];
    int capacity;
    int alive_count, high_water;
    int age_head, age_tail;
    int free_head, free_count;
    long long next_seq;
    long long spawned_total, expired_total, evicted_total, refused_total;
    double evicted_age_sum;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;
    int policy;
} World;

static void world_init(World* w, int capacity, int policy) {
    if (capacity < 1) capacity = 1;
    if (capacity > MAX_PARTICLES) capacity = MAX_PARTICLES;
    memset(w, 0, sizeof(*w));
    w->capacity = capacity;
    w->policy = policy;
    w->age_head = w->age_tail = NO_SLOT;
    w->free_head = 0;
    for (int i = 0; i < capacity; i++) {
        w->pool[i].age_prev = w->pool[i].age_next = NO_SLOT;
        w->pool[i].free_next = (i + 1 < capacity) ? i + 1 : NO_SLOT;
    }
    w->free_count = capacity;
}

static int emitter_add(World* w, int id, int quota, int priority) {
    if (w->emitter_count >= MAX_EMITTERS) return -1;
    Emitter* e = &w->emitters[w->emitter_count++];
    e->id = id; e->quota = quota; e->spawned = 0; e->priority = priority;
    return e->id;
}

/* ---- age list: oldest at head, newest at tail, all operations O(1) ---- */

static void age_link_tail(World* w, int idx) {
    Particle* p = &w->pool[idx];
    p->age_next = NO_SLOT;
    p->age_prev = w->age_tail;
    if (w->age_tail != NO_SLOT) w->pool[w->age_tail].age_next = idx;
    else w->age_head = idx;
    w->age_tail = idx;
}

static void age_unlink(World* w, int idx) {
    Particle* p = &w->pool[idx];
    if (p->age_prev != NO_SLOT) w->pool[p->age_prev].age_next = p->age_next;
    else w->age_head = p->age_next;
    if (p->age_next != NO_SLOT) w->pool[p->age_next].age_prev = p->age_prev;
    else w->age_tail = p->age_prev;
    p->age_prev = p->age_next = NO_SLOT;
}

/* ---- free list + eviction ------------------------------------------------- */

static void slot_free(World* w, int idx) {
    Particle* p = &w->pool[idx];
    if (p->alive) {
        age_unlink(w, idx);
        p->alive = 0;
        w->alive_count--;
    }
    p->free_next = w->free_head;
    w->free_head = idx;
    w->free_count++;
}

/* reclaim the live particle with the largest survival time (list head) */
static int pool_evict_oldest(World* w) {
    int idx = w->age_head;
    if (idx == NO_SLOT) return NO_SLOT;
    w->evicted_age_sum += w->pool[idx].age;
    slot_free(w, idx);
    w->evicted_total++;
    return idx;
}

static int pool_alloc(World* w) {
    int idx = w->free_head;
    if (idx == NO_SLOT) {
        if (w->policy != POOL_EVICT_OLDEST) return NO_SLOT;
        idx = pool_evict_oldest(w);
        if (idx == NO_SLOT) return NO_SLOT;
    }
    w->free_head = w->pool[idx].free_next;
    w->free_count--;
    return idx;
}

static int spawn_one(World* w, int emitter_id) {
    int idx = pool_alloc(w);
    if (idx < 0) { w->refused_total++; return NO_SLOT; }
    Particle* p = &w->pool[idx];
    p->alive = 1; p->age = 0.0f;
    p->x = 0.0f; p->y = 0.0f;
    p->vx = 1.0f; p->vy = 0.5f;
    p->life = 2.0f;
    p->tag = emitter_id;
    p->spawn_seq = w->next_seq++;
    age_link_tail(w, idx);
    w->alive_count++;
    w->spawned_total++;
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
    int idx = w->age_head;
    while (idx != NO_SLOT) {
        int next = w->pool[idx].age_next;   /* save before unlink */
        Particle* p = &w->pool[idx];
        p->age += dt;
        p->x += p->vx * dt; p->y += p->vy * dt;
        if (p->age >= p->life) {
            w->expired_total++;
            slot_free(w, idx);
        }
        idx = next;
    }
}

/* ---- invariant check ------------------------------------------------------ */

static int check_invariants(const World* w) {
    int alive = 0, age_len = 0, free_len = 0, ordered = 1;
    long long prev_seq = -1;
    for (int i = 0; i < w->capacity; i++) if (w->pool[i].alive) alive++;
    for (int idx = w->age_head; idx != NO_SLOT; idx = w->pool[idx].age_next) {
        if (++age_len > w->capacity) return 0;
        if (!w->pool[idx].alive) return 0;
        if (w->pool[idx].spawn_seq < prev_seq) ordered = 0;
        prev_seq = w->pool[idx].spawn_seq;
    }
    for (int idx = w->free_head; idx != NO_SLOT; idx = w->pool[idx].free_next) {
        if (++free_len > w->capacity) return 0;
    }
    return alive == w->alive_count && age_len == alive &&
           free_len == w->free_count && alive + free_len == w->capacity &&
           ordered;
}

/* ---- tests ---------------------------------------------------------------- */

static int failures = 0;

static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void test_priority_scheduling(void) {
    static World w;
    world_init(&w, MAX_PARTICLES, POOL_EVICT_OLDEST);
    emitter_add(&w, 1, 40, 2);
    emitter_add(&w, 2, 30, 9);
    emitter_add(&w, 3, 20, 5);
    for (int frame = 0; frame < 10; frame++) {
        spawn_frame(&w, 50);
        update(&w, 0.1f);
    }
    int tag[4] = {0};
    for (int i = 0; i < w.capacity; i++) if (w.pool[i].alive) tag[w.pool[i].tag]++;
    printf("priority_alive=%d by_emitter=%d,%d,%d\n", w.alive_count, tag[1], tag[2], tag[3]);
    /* priority 9 first: 30/frame, then priority 5: 20/frame, priority 2 starves */
    check("priority_quota_ok", tag[2] == 300 && tag[3] == 200 && tag[1] == 0);
    check("priority_invariants_ok", check_invariants(&w));
}

static void test_evict_oldest(void) {
    static World w;
    world_init(&w, 4, POOL_EVICT_OLDEST);
    for (int i = 0; i < 4; i++) check("evict_fill_ok", spawn_one(&w, i + 1) == i);
    check("evict_oldest_before", w.pool[w.age_head].spawn_seq == 0 && w.evicted_total == 0);

    int idx = spawn_one(&w, 9);
    printf("evict idx=%d evicted=%lld age_head_seq=%lld alive=%d\n",
           idx, w.evicted_total, w.pool[w.age_head].spawn_seq, w.alive_count);
    /* seq 0 was the oldest and its slot is immediately reused -> O(1) recycle */
    check("evict_oldest_reclaimed", w.evicted_total == 1 && idx == 0 &&
                                    w.pool[idx].spawn_seq == 4);
    check("evict_alive_capacity", w.alive_count == 4);
    check("evict_order_ok", w.pool[w.age_head].spawn_seq == 1 &&
                            w.pool[w.age_tail].spawn_seq == 4);
    check("evict_invariants_ok", check_invariants(&w));
}

static void test_expiry_reuses_slot_without_eviction(void) {
    static World w;
    world_init(&w, 2, POOL_EVICT_OLDEST);
    int a = spawn_one(&w, 1);
    int b = spawn_one(&w, 2);
    w.pool[a].life = 0.0f;
    update(&w, 0.1f);
    check("expiry_frees_slot", w.alive_count == 1 && w.evicted_total == 0);

    int c = spawn_one(&w, 3);   /* must reuse the expired slot, not evict */
    check("expiry_slot_reused", c == a && w.evicted_total == 0);

    int d = spawn_one(&w, 4);   /* full again: oldest live (seq 1) goes */
    check("expiry_then_evict", d == b && w.evicted_total == 1 &&
                               w.pool[d].spawn_seq == 3 &&
                               w.pool[w.age_head].spawn_seq == 2);
    check("expiry_invariants_ok", check_invariants(&w));
}

/* ---- burst-load simulation: reject-on-full vs oldest-first eviction ------ */

#define SIM_FRAMES 1200
#define SIM_BASE_QUOTA 60
#define SIM_BURST_QUOTA 700
#define SIM_BURST_PERIOD 200
#define SIM_BURST_LEN 40
#define SIM_DT (1.0f / 60.0f)

typedef struct {
    long long frames, requests, served, refused, drop_frames;
    long long evicted;
    double avg_evicted_age;
    int invariants_ok;
} SimStats;

static SimStats sim_run(int policy, int capacity) {
    static World w;
    world_init(&w, capacity, policy);
    emitter_add(&w, 1, 0, 1);
    SimStats s;
    memset(&s, 0, sizeof(s));
    s.invariants_ok = 1;
    for (int f = 0; f < SIM_FRAMES; f++) {
        int quota = ((f % SIM_BURST_PERIOD) < SIM_BURST_LEN) ? SIM_BURST_QUOTA
                                                              : SIM_BASE_QUOTA;
        w.emitters[0].quota = quota;
        long long served0 = w.spawned_total, refused0 = w.refused_total;
        spawn_frame(&w, quota);
        update(&w, SIM_DT);
        s.requests += quota;
        s.served += w.spawned_total - served0;
        if (w.refused_total > refused0) s.drop_frames++;
        if ((f % 100) == 0 && !check_invariants(&w)) s.invariants_ok = 0;
        s.frames++;
    }
    s.refused = w.refused_total;
    s.evicted = w.evicted_total;
    s.avg_evicted_age = s.evicted > 0 ? w.evicted_age_sum / (double)s.evicted : 0.0;
    if (!check_invariants(&w)) s.invariants_ok = 0;
    return s;
}

static double pct(long long part, long long whole) {
    return whole > 0 ? 100.0 * (double)part / (double)whole : 0.0;
}

static void test_eviction_vs_reject(void) {
    SimStats old_pool = sim_run(POOL_REJECT_ON_FULL, MAX_PARTICLES);
    SimStats evict = sim_run(POOL_EVICT_OLDEST, MAX_PARTICLES);

    printf("[compare] capacity=%d frames=%d requests=%lld\n",
           MAX_PARTICLES, SIM_FRAMES, old_pool.requests);
    printf("old_fixed_pool      served=%lld refused=%lld drop_frames=%lld "
           "drop_frame_rate=%.2f%%\n",
           old_pool.served, old_pool.refused, old_pool.drop_frames,
           pct(old_pool.drop_frames, old_pool.frames));
    printf("evict_oldest_pool   served=%lld refused=%lld drop_frames=%lld "
           "drop_frame_rate=%.2f%% evicted=%lld avg_evicted_age=%.3fs\n",
           evict.served, evict.refused, evict.drop_frames,
           pct(evict.drop_frames, evict.frames), evict.evicted,
           evict.avg_evicted_age);
    printf("infinite_pool       served=%lld refused=0 drop_frames=0 "
           "drop_frame_rate=0.00%%\n", evict.requests);

    double reduction = old_pool.drop_frames > 0
        ? 100.0 * (double)(old_pool.drop_frames - evict.drop_frames) /
          (double)old_pool.drop_frames
        : 100.0;
    printf("drop_frame_rate_reduction=%.1f%%\n", reduction);
    check("compare_invariants_ok", old_pool.invariants_ok && evict.invariants_ok);
    check("reject_pool_actually_drops", old_pool.drop_frames > 0);
    check("evict_pool_serves_all", evict.served == evict.requests);
    check("drop_frame_rate_reduction_over_80pct", reduction >= 80.0);
}

/* ---- per-frame update budget at 10k capacity ----------------------------- */

static void test_update_performance(void) {
    static World w;
    world_init(&w, MAX_PARTICLES, POOL_EVICT_OLDEST);
    for (int i = 0; i < MAX_PARTICLES; i++) spawn_one(&w, 1);
    for (int i = 0; i < MAX_PARTICLES; i++) w.pool[i].life = 1e9f;  /* stay full */

    const int frames = 2000;
    double total_us = 0.0, max_us = 0.0;
    for (int f = 0; f < frames; f++) {
        struct timespec t0, t1;
        timespec_get(&t0, TIME_UTC);
        update(&w, SIM_DT);
        timespec_get(&t1, TIME_UTC);
        double us = (double)(t1.tv_sec - t0.tv_sec) * 1e6 +
                    (double)(t1.tv_nsec - t0.tv_nsec) / 1e3;
        total_us += us;
        if (us > max_us) max_us = us;
    }
    printf("update capacity=%d alive=%d frames=%d avg_us=%.2f max_us=%.2f\n",
           w.capacity, w.alive_count, frames, total_us / frames, max_us);
    check("update_frame_under_2ms", max_us <= 2000.0);
}

int main(void) {
    test_priority_scheduling();
    test_evict_oldest();
    test_expiry_reuses_slot_without_eviction();
    test_eviction_vs_reject();
    test_update_performance();
    printf("all_ok=%d\n", failures == 0 ? 1 : 0);
    return failures == 0 ? 0 : 1;
}
