
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget
   and emitter priority scheduling, plus ring-buffer frame replay. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_PARTICLES 4096
#define MAX_EMITTERS 8

typedef struct { float x, y, vx, vy, life, age; int alive; int tag; } Particle;
typedef struct { int id; int quota; int spawned; int priority; } Emitter;

typedef struct {
    Particle pool[MAX_PARTICLES];
    int alive_count, high_water;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;
} World;

/* ---------- replay error codes: callers must handle, never silent ---------- */
enum {
    REPLAY_OK = 0,
    REPLAY_ERR_CAPACITY = 1,     /* capacity <= 0 */
    REPLAY_ERR_ALLOC = 2,
    REPLAY_ERR_FRAME_JUMP = 3,   /* frame number is not the expected next frame */
    REPLAY_ERR_FRAME_MISSING = 4 /* requested frame is outside the retained window */
};

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

/* ---------- per-frame inputs: spawn parameters plus external input ---------- */
typedef struct {
    int frame;                 /* sequential frame number, starts at 0 */
    float dt;
    int budget;                /* per-frame spawn budget */
    float gust_x, gust_y;      /* external input: acceleration applied this frame */
    int quota[MAX_EMITTERS];   /* per-emitter spawn quota this frame, by emitter slot */
} FrameInput;

/* fixed-capacity ring of frame inputs, plus a snapshot of the oldest retained
   frame's world state so overwrite never destroys the ability to recompute. */
typedef struct {
    FrameInput* ring;
    int capacity;
    int head_slot;             /* ring index of the oldest retained record */
    int count;                 /* valid records currently held */
    int next_frame;            /* expected frame number of the next record */
    int base_frame;            /* frame number whose state is held in base */
    int overwrites;            /* count of oldest records evicted */
    World base;                /* world state S_base_frame */
} History;

static int history_init(History* h, int capacity) {
    if (capacity <= 0) return REPLAY_ERR_CAPACITY;
    h->ring = (FrameInput*)malloc((size_t)capacity * sizeof(FrameInput));
    if (!h->ring) return REPLAY_ERR_ALLOC;
    h->capacity = capacity;
    h->head_slot = 0;
    h->count = 0;
    h->next_frame = 0;
    h->base_frame = 0;
    h->overwrites = 0;
    world_init(&h->base);
    return REPLAY_OK;
}

static void history_destroy(History* h) {
    free(h->ring);
    h->ring = NULL;
    h->capacity = h->count = 0;
}

/* sync emitter configuration (and any pre-frame state) into the base snapshot */
static void history_attach(History* h, const World* w) {
    h->base = *w;
    h->base_frame = 0;
}

/* physics with external gust; gust == 0 reproduces update() bit for bit */
static void update_gust(World* w, float dt, float gust_x, float gust_y) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive) continue;
        p->age += dt;
        p->vx += gust_x * dt;
        p->vy += gust_y * dt;
        p->x += p->vx * dt; p->y += p->vy * dt;
        if (p->age >= p->life) { p->alive = 0; w->alive_count--; }
    }
}

/* apply one recorded frame: spawn parameters then physics, in fixed order */
static void apply_input(World* w, const FrameInput* in) {
    for (int i = 0; i < w->emitter_count; i++)
        w->emitters[i].quota = in->quota[i];
    spawn_frame(w, in->budget);
    update_gust(w, in->dt, in->gust_x, in->gust_y);
}

/* record one frame, evicting the oldest record when the ring is full */
static int history_record(History* h, const FrameInput* in) {
    if (in->frame != h->next_frame) return REPLAY_ERR_FRAME_JUMP;
    if (h->count == h->capacity) {
        /* advance the base snapshot across the evicted record so the
           surviving window still has a known state to recompute from */
        FrameInput* oldest = &h->ring[h->head_slot];
        apply_input(&h->base, oldest);
        h->base_frame++;
        h->head_slot = (h->head_slot + 1) % h->capacity;
        h->count--;
        h->overwrites++;
    }
    int slot = (h->head_slot + h->count) % h->capacity;
    h->ring[slot] = *in;
    h->count++;
    h->next_frame++;
    return REPLAY_OK;
}

/* record then step; a rejected record leaves the world untouched */
static int step_recorded(World* w, History* h, const FrameInput* in) {
    int rc = history_record(h, in);
    if (rc != REPLAY_OK) return rc;
    apply_input(w, in);
    return REPLAY_OK;
}

/* roll back to the state after target frames and recompute from the base
   snapshot; records at or after target are dropped so the frame can be re-run */
static int history_rollback(History* h, World* out, int target) {
    if (target < h->base_frame || target > h->next_frame)
        return REPLAY_ERR_FRAME_MISSING;
    *out = h->base;
    for (int f = h->base_frame; f < target; f++) {
        int slot = (h->head_slot + (f - h->base_frame)) % h->capacity;
        apply_input(out, &h->ring[slot]);
    }
    h->count = target - h->base_frame;
    h->next_frame = target;
    return REPLAY_OK;
}

/* =============================== demo / tests =============================== */

static World make_world(void) {
    World w;
    world_init(&w);
    emitter_add(&w, 1, 40, 2);
    emitter_add(&w, 2, 30, 9);
    emitter_add(&w, 3, 20, 5);
    return w;
}

static FrameInput make_input(int frame, float gust_x, float gust_y,
                             const int* quota_override) {
    FrameInput in;
    memset(&in, 0, sizeof(in));
    in.frame = frame;
    in.dt = 0.1f;
    in.budget = 50;
    in.gust_x = gust_x;
    in.gust_y = gust_y;
    in.quota[0] = 40; in.quota[1] = 30; in.quota[2] = 20;
    if (quota_override)
        for (int i = 0; i < 3; i++) in.quota[i] = quota_override[i];
    return in;
}

/* uninterrupted reference world built straight from the input log */
static World build_reference(const FrameInput* inputs, int frames) {
    World w = make_world();
    for (int f = 0; f < frames; f++) apply_input(&w, &inputs[f]);
    return w;
}

static int g_failures;

static void check(const char* name, int cond) {
    printf("  [%s] %s\n", cond ? "PASS" : "FAIL", name);
    if (!cond) g_failures++;
}

static void print_summary(const World* w) {
    printf("alive=%d high_water=%d\n", w->alive_count, w->high_water);
    int tag1 = 0, tag2 = 0, tag3 = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (w->pool[i].alive) {
        if (w->pool[i].tag == 1) tag1++; else if (w->pool[i].tag == 2) tag2++; else tag3++;
    }
    printf("by_emitter=%d,%d,%d\n", tag1, tag2, tag3);
    /* quota 优先：高优先级 2 应先拿满 30 */
    printf("priority_respected=%d\n", tag2 >= tag3 && tag3 >= 0);
}

static int run_replay_tests(void) {
    enum { FRAMES = 10, CAP = 6 };
    FrameInput inputs[FRAMES];
    for (int f = 0; f < FRAMES; f++)
        inputs[f] = make_input(f, 0.0f, 0.0f, NULL);
    inputs[5].gust_x = 3.0f; inputs[5].gust_y = -1.5f;   /* 外部输入 */
    int quota7[3] = { 40, 50, 20 };                        /* 发射参数变化 */
    inputs[7] = make_input(7, 0.0f, 0.0f, quota7);

    /* 1. replay disabled: FrameInput path must match the legacy API bit for bit.
       Use a gust-free log so both worlds see identical external input. */
    FrameInput plain[FRAMES];
    for (int f = 0; f < FRAMES; f++)
        plain[f] = make_input(f, 0.0f, 0.0f, NULL);
    World legacy = make_world();
    for (int f = 0; f < FRAMES; f++) {
        spawn_frame(&legacy, 50);
        update(&legacy, 0.1f);
    }
    World via_input = build_reference(plain, FRAMES);
    check("replay-off path bit-identical to legacy run",
          memcmp(&legacy, &via_input, sizeof(World)) == 0);

    /* 2. ring overwrite: 10 frames into capacity 6 evicts the oldest 4 */
    History h;
    int rc = history_init(&h, CAP);
    check("history init capacity 6 ok", rc == REPLAY_OK);
    World tracked = make_world();
    history_attach(&h, &tracked);
    for (int f = 0; f < FRAMES; f++) {
        rc = step_recorded(&tracked, &h, &inputs[f]);
        if (rc != REPLAY_OK) { check("record all frames ok", 0); break; }
    }
    check("record all frames ok", rc == REPLAY_OK);
    check("overwrite count == 4", h.overwrites == 4);
    check("oldest retained frame is 4", h.base_frame == 4);
    check("ring holds frames 4..9", h.count == 6 && h.next_frame == 10);
    World ref10 = build_reference(inputs, 10);
    check("tracked world equals uninterrupted run",
          memcmp(&tracked, &ref10, sizeof(World)) == 0);

    /* 3. rollback by frame number and recompute, per-particle identical */
    int targets[] = { 10, 8, 6, 4 };
    for (unsigned k = 0; k < sizeof(targets) / sizeof(targets[0]); k++) {
        int t = targets[k];
        History hh;
        history_init(&hh, CAP);
        World w = make_world();
        history_attach(&hh, &w);
        for (int f = 0; f < FRAMES; f++) step_recorded(&w, &hh, &inputs[f]);
        World rolled;
        rc = history_rollback(&hh, &rolled, t);
        char label[96];
        snprintf(label, sizeof(label), "rollback to frame %d succeeds", t);
        check(label, rc == REPLAY_OK);
        World reft = build_reference(inputs, t);
        snprintf(label, sizeof(label),
                 "frame %d recompute matches uninterrupted run per-particle", t);
        check(label, memcmp(&rolled, &reft, sizeof(World)) == 0);
        check("rollback shrinks the retained window",
              hh.next_frame == t && hh.count == t - hh.base_frame);
        history_destroy(&hh);
    }

    /* 4. rollback then continue with the same inputs reaches the same state */
    History hc;
    history_init(&hc, CAP);
    World cont = make_world();
    history_attach(&hc, &cont);
    for (int f = 0; f < FRAMES; f++) step_recorded(&cont, &hc, &inputs[f]);
    World restored;
    rc = history_rollback(&hc, &restored, 6);
    check("rollback to frame 6 ok", rc == REPLAY_OK);
    for (int f = 6; f < FRAMES; f++) {
        rc = step_recorded(&restored, &hc, &inputs[f]);
        if (rc != REPLAY_OK) break;
    }
    check("re-run frames 6..9 ok", rc == REPLAY_OK && hc.next_frame == 10);
    check("recontinued state equals uninterrupted run",
          memcmp(&restored, &ref10, sizeof(World)) == 0);

    /* and a changed external input must actually diverge */
    History hd;
    history_init(&hd, CAP);
    World diverge = make_world();
    history_attach(&hd, &diverge);
    for (int f = 0; f < FRAMES; f++) step_recorded(&diverge, &hd, &inputs[f]);
    World dd;
    history_rollback(&hd, &dd, 6);
    FrameInput changed = inputs[6];
    changed.gust_y = 5.0f;
    apply_input(&dd, &changed);
    for (int f = 7; f < FRAMES; f++) apply_input(&dd, &inputs[f]);
    check("modified external input diverges from reference",
          memcmp(&dd, &ref10, sizeof(World)) != 0);
    history_destroy(&hd);
    history_destroy(&hc);
    history_destroy(&h);

    /* 4b. no eviction at all: rollback to frame 0 returns the base snapshot */
    History h0;
    history_init(&h0, FRAMES + 4);
    World w0 = make_world();
    history_attach(&h0, &w0);
    for (int f = 0; f < FRAMES; f++) step_recorded(&w0, &h0, &plain[f]);
    check("non-full ring never overwrites", h0.overwrites == 0 && h0.base_frame == 0);
    World rolled0;
    rc = history_rollback(&h0, &rolled0, 0);
    World ref0 = make_world();
    check("rollback to frame 0 ok", rc == REPLAY_OK);
    check("frame 0 state equals fresh base snapshot",
          memcmp(&rolled0, &ref0, sizeof(World)) == 0);
    history_destroy(&h0);

    /* 5. error cases: reported, never silently ignored */
    History bad;
    rc = history_init(&bad, 0);
    check("zero capacity rejected", rc == REPLAY_ERR_CAPACITY);
    rc = history_init(&bad, -3);
    check("negative capacity rejected", rc == REPLAY_ERR_CAPACITY);

    History hj;
    history_init(&hj, 8);
    World wj = make_world();
    history_attach(&hj, &wj);
    rc = step_recorded(&wj, &hj, &inputs[0]);
    check("frame 0 recorded", rc == REPLAY_OK);
    FrameInput jumped = inputs[2];
    rc = history_record(&hj, &jumped);
    check("frame jump 0 -> 2 rejected", rc == REPLAY_ERR_FRAME_JUMP);
    FrameInput backwards = inputs[0];
    rc = history_record(&hj, &backwards);
    check("backward frame number rejected", rc == REPLAY_ERR_FRAME_JUMP);
    World ref1 = build_reference(inputs, 1);
    check("world untouched after rejected record",
          memcmp(&wj, &ref1, sizeof(World)) == 0);
    history_destroy(&hj);

    History hm;
    history_init(&hm, CAP);
    World wm = make_world();
    history_attach(&hm, &wm);
    for (int f = 0; f < FRAMES; f++) step_recorded(&wm, &hm, &inputs[f]);
    World scratch;
    check("rollback before retained window rejected",
          history_rollback(&hm, &scratch, 3) == REPLAY_ERR_FRAME_MISSING);
    check("rollback to negative frame rejected",
          history_rollback(&hm, &scratch, -1) == REPLAY_ERR_FRAME_MISSING);
    check("rollback beyond newest frame rejected",
          history_rollback(&hm, &scratch, 11) == REPLAY_ERR_FRAME_MISSING);
    history_destroy(&hm);

    if (g_failures == 0) printf("ALL REPLAY CHECKS PASSED\n");
    else printf("%d REPLAY CHECK(S) FAILED\n", g_failures);
    return g_failures ? 1 : 0;
}

int main(int argc, char** argv) {
    if (argc >= 2 && strcmp(argv[1], "replay") == 0)
        return run_replay_tests();

    /* replay disabled: identical scenario and output to the original pool */
    World w; world_init(&w);
    emitter_add(&w, 1, 40, 2);
    emitter_add(&w, 2, 30, 9);
    emitter_add(&w, 3, 20, 5);
    for (int frame = 0; frame < 10; frame++) {
        spawn_frame(&w, 50);
        update(&w, 0.1f);
    }
    print_summary(&w);
    return 0;
}
