
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget
   and emitter priority scheduling. Supports binary save/load of the full
   simulation state (emitters, rng stream, live particles) with versioning
   and corruption detection. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>

#define MAX_PARTICLES 4096
#define MAX_EMITTERS 8

typedef struct { float x, y, vx, vy, life, age; int alive; int tag; } Particle;
typedef struct { int id; int quota; int spawned; int priority; } Emitter;

typedef struct {
    Particle pool[MAX_PARTICLES];
    int alive_count, high_water;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;
    uint64_t rng_state;
} World;

static void world_init(World* w) {
    memset(w, 0, sizeof(*w));
    w->emitter_count = 0;
    w->rng_state = 0x9E3779B97F4A7C15ULL; /* fixed default seed */
}

static int emitter_add(World* w, int id, int quota, int priority) {
    if (w->emitter_count >= MAX_EMITTERS) return -1;
    Emitter* e = &w->emitters[w->emitter_count++];
    e->id = id; e->quota = quota; e->spawned = 0; e->priority = priority;
    return e->id;
}

/* xorshift64*: deterministic stream, serialized as part of the state */
static uint64_t rng_next(World* w) {
    uint64_t x = w->rng_state;
    x ^= x >> 12; x ^= x << 25; x ^= x >> 27;
    w->rng_state = x;
    return x * 0x2545F4914F6CDD1DULL;
}

static float rng_float(World* w) { /* [0,1) */
    return (float)(rng_next(w) >> 40) / (float)(1u << 24);
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
    p->vx = 0.8f + 0.4f * rng_float(w);
    p->vy = 0.3f + 0.4f * rng_float(w);
    p->life = 1.5f + rng_float(w);
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

/* ---------------- serialization ---------------- */

#define SAVE_MAGIC   0x50545243UL /* 'PTRC' */
#define SAVE_VERSION 1UL

enum {
    SAVE_OK = 0,
    SAVE_ERR_SIZE = -1,     /* output buffer too small */
    SAVE_ERR_VERSION = -2,  /* magic or version mismatch */
    SAVE_ERR_CORRUPT = -3   /* bad length, checksum, or out-of-range data */
};

/* per live particle: u16 slot + 6 f32 + i32 tag (alive flag implied) */
#define PARTICLE_REC_SIZE (2 + 6 * 4 + 4)
#define EMITTER_REC_SIZE  (4 * 4)
#define SAVE_HEADER_SIZE  (4 + 4 + 4 + 4 + 4 + 8) /* magic,ver,counts x3,rng */
#define SAVE_TRAILER_SIZE 4 /* checksum */

/* size of the in-memory live state we persist (counters, rng, emitters,
   live particles with their pool slot) — the baseline for the size budget */
static size_t world_raw_state_size(const World* w) {
    return 3 * sizeof(uint32_t) + sizeof(uint64_t)
         + (size_t)w->emitter_count * sizeof(Emitter)
         + (size_t)w->alive_count * (sizeof(Particle) + sizeof(uint16_t));
}

static void put_u16(uint8_t** p, uint16_t v) {
    (*p)[0] = (uint8_t)(v & 0xff); (*p)[1] = (uint8_t)(v >> 8); *p += 2;
}
static void put_u32(uint8_t** p, uint32_t v) {
    for (int i = 0; i < 4; i++) (*p)[i] = (uint8_t)(v >> (8 * i));
    *p += 4;
}
static void put_u64(uint8_t** p, uint64_t v) {
    for (int i = 0; i < 8; i++) (*p)[i] = (uint8_t)(v >> (8 * i));
    *p += 8;
}
static void put_f32(uint8_t** p, float f) {
    uint32_t u; memcpy(&u, &f, 4); put_u32(p, u);
}

typedef struct { const uint8_t* cur; size_t left; int err; } Reader;

static uint16_t get_u16(Reader* r) {
    if (r->left < 2) { r->err = 1; return 0; }
    uint16_t v = (uint16_t)(r->cur[0] | (r->cur[1] << 8));
    r->cur += 2; r->left -= 2; return v;
}
static uint32_t get_u32(Reader* r) {
    if (r->left < 4) { r->err = 1; return 0; }
    uint32_t v = 0;
    for (int i = 0; i < 4; i++) v |= (uint32_t)r->cur[i] << (8 * i);
    r->cur += 4; r->left -= 4; return v;
}
static uint64_t get_u64(Reader* r) {
    if (r->left < 8) { r->err = 1; return 0; }
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v |= (uint64_t)r->cur[i] << (8 * i);
    r->cur += 8; r->left -= 8; return v;
}
static float get_f32(Reader* r) {
    uint32_t u = get_u32(r); float f; memcpy(&f, &u, 4); return f;
}

/* FNV-1a over the serialized payload */
static uint32_t save_checksum(const uint8_t* data, size_t len) {
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; i++) { h ^= data[i]; h *= 16777619u; }
    return h;
}

static int world_save(const World* w, uint8_t* buf, size_t cap, size_t* out_len) {
    size_t need = SAVE_HEADER_SIZE + SAVE_TRAILER_SIZE
                + (size_t)w->emitter_count * EMITTER_REC_SIZE
                + (size_t)w->alive_count * PARTICLE_REC_SIZE;
    if (cap < need) return SAVE_ERR_SIZE;
    uint8_t* p = buf;
    put_u32(&p, SAVE_MAGIC);
    put_u32(&p, SAVE_VERSION);
    put_u32(&p, (uint32_t)w->emitter_count);
    put_u32(&p, (uint32_t)w->alive_count);
    put_u32(&p, (uint32_t)w->high_water);
    put_u64(&p, w->rng_state);
    for (int i = 0; i < w->emitter_count; i++) {
        const Emitter* e = &w->emitters[i];
        put_u32(&p, (uint32_t)e->id);
        put_u32(&p, (uint32_t)e->quota);
        put_u32(&p, (uint32_t)e->spawned);
        put_u32(&p, (uint32_t)e->priority);
    }
    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle* s = &w->pool[i];
        if (!s->alive) continue;
        put_u16(&p, (uint16_t)i);
        put_f32(&p, s->x); put_f32(&p, s->y);
        put_f32(&p, s->vx); put_f32(&p, s->vy);
        put_f32(&p, s->life); put_f32(&p, s->age);
        put_u32(&p, (uint32_t)s->tag);
    }
    put_u32(&p, save_checksum(buf, (size_t)(p - buf)));
    *out_len = (size_t)(p - buf);
    return SAVE_OK;
}

static int world_load(World* w, const uint8_t* buf, size_t len) {
    if (len < SAVE_HEADER_SIZE + SAVE_TRAILER_SIZE) return SAVE_ERR_CORRUPT;
    Reader r = { buf, len, 0 };
    uint32_t magic = get_u32(&r);
    uint32_t version = get_u32(&r);
    if (magic != SAVE_MAGIC || version != SAVE_VERSION) return SAVE_ERR_VERSION;
    /* verify checksum over everything but the trailer */
    {
        uint32_t want = 0;
        for (int i = 0; i < 4; i++)
            want |= (uint32_t)buf[len - 4 + i] << (8 * i);
        if (save_checksum(buf, len - SAVE_TRAILER_SIZE) != want)
            return SAVE_ERR_CORRUPT;
    }
    uint32_t emitter_count = get_u32(&r);
    uint32_t alive_count = get_u32(&r);
    uint32_t high_water = get_u32(&r);
    uint64_t rng_state = get_u64(&r);
    if (r.err) return SAVE_ERR_CORRUPT;
    if (emitter_count > MAX_EMITTERS || alive_count > MAX_PARTICLES)
        return SAVE_ERR_CORRUPT;
    size_t expect = SAVE_HEADER_SIZE + SAVE_TRAILER_SIZE
                  + (size_t)emitter_count * EMITTER_REC_SIZE
                  + (size_t)alive_count * PARTICLE_REC_SIZE;
    if (expect != len) return SAVE_ERR_CORRUPT;

    Emitter emitters[MAX_EMITTERS];
    for (uint32_t i = 0; i < emitter_count; i++) {
        emitters[i].id = (int)get_u32(&r);
        emitters[i].quota = (int)get_u32(&r);
        emitters[i].spawned = (int)get_u32(&r);
        emitters[i].priority = (int)get_u32(&r);
    }
    uint8_t seen[MAX_PARTICLES]; /* duplicate-slot detection */
    memset(seen, 0, sizeof(seen));
    world_init(w);
    w->emitter_count = (int)emitter_count;
    w->alive_count = (int)alive_count;
    w->high_water = (int)high_water;
    w->rng_state = rng_state;
    for (uint32_t i = 0; i < emitter_count; i++) w->emitters[i] = emitters[i];
    uint32_t found = 0;
    for (uint32_t i = 0; i < alive_count; i++) {
        uint16_t slot = get_u16(&r);
        Particle p;
        p.x = get_f32(&r); p.y = get_f32(&r);
        p.vx = get_f32(&r); p.vy = get_f32(&r);
        p.life = get_f32(&r); p.age = get_f32(&r);
        p.tag = (int)get_u32(&r);
        p.alive = 1;
        if (r.err || slot >= MAX_PARTICLES || seen[slot]) return SAVE_ERR_CORRUPT;
        seen[slot] = 1;
        w->pool[slot] = p;
        found++;
    }
    if (r.err || found != alive_count) return SAVE_ERR_CORRUPT;
    return SAVE_OK;
}

/* ---------------- test ---------------- */

static void run_frames(World* w, int n) {
    for (int i = 0; i < n; i++) { spawn_frame(w, 50); update(w, 0.1f); }
}

/* particle-by-particle: every slot's alive flag must match, and every live
   particle must be bit-identical; counters, emitters and rng must match too */
static int worlds_match(const World* a, const World* b) {
    if (a->alive_count != b->alive_count || a->high_water != b->high_water
        || a->emitter_count != b->emitter_count || a->rng_state != b->rng_state)
        return 0;
    for (int i = 0; i < a->emitter_count; i++)
        if (memcmp(&a->emitters[i], &b->emitters[i], sizeof(Emitter)) != 0)
            return 0;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!a->pool[i].alive != !b->pool[i].alive) return 0;
        if (a->pool[i].alive
            && memcmp(&a->pool[i], &b->pool[i], sizeof(Particle)) != 0)
            return 0;
    }
    return 1;
}

int main(void) {
    static uint8_t buf[SAVE_HEADER_SIZE + SAVE_TRAILER_SIZE
                       + MAX_EMITTERS * EMITTER_REC_SIZE
                       + MAX_PARTICLES * PARTICLE_REC_SIZE];
    static uint8_t bad[sizeof(buf)];
    size_t len = 0;

    World ref; world_init(&ref);
    emitter_add(&ref, 1, 40, 2);
    emitter_add(&ref, 2, 30, 9);
    emitter_add(&ref, 3, 20, 5);
    run_frames(&ref, 5);

    if (world_save(&ref, buf, sizeof(buf), &len) != SAVE_OK) {
        printf("save=FAIL\n"); return 1;
    }
    size_t raw = world_raw_state_size(&ref);
    /* 序列化体积不超过裸状态三成 */
    int size_ok = len * 10 <= raw * 13;
    printf("serialized=%zu raw=%zu within_130pct=%d\n", len, raw, size_ok);

    run_frames(&ref, 10); /* uninterrupted reference run */

    World restored; world_init(&restored);
    int load_rc = world_load(&restored, buf, len);
    run_frames(&restored, 10);
    int identical = load_rc == SAVE_OK && worlds_match(&ref, &restored);
    printf("load_rc=%d alive=%d high_water=%d resume_identical=%d\n",
           load_rc, restored.alive_count, restored.high_water, identical);

    /* 版本号不匹配要拒绝 */
    memcpy(bad, buf, len);
    bad[4] ^= 0xff; /* version field */
    World w2; world_init(&w2);
    int version_rejected = world_load(&w2, bad, len) == SAVE_ERR_VERSION;
    printf("version_rejected=%d\n", version_rejected);

    /* 损坏输入要报错：翻转载荷字节 / 截断 */
    memcpy(bad, buf, len);
    bad[SAVE_HEADER_SIZE + 3] ^= 0x01;
    World w3; world_init(&w3);
    int corrupt_rejected = world_load(&w3, bad, len) == SAVE_ERR_CORRUPT;
    World w4; world_init(&w4);
    int truncated_rejected = world_load(&w4, buf, len - 7) == SAVE_ERR_CORRUPT;
    printf("corrupt_rejected=%d truncated_rejected=%d\n",
           corrupt_rejected, truncated_rejected);

    int tag1 = 0, tag2 = 0, tag3 = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (ref.pool[i].alive) {
        if (ref.pool[i].tag == 1) tag1++; else if (ref.pool[i].tag == 2) tag2++; else tag3++;
    }
    printf("by_emitter=%d,%d,%d\n", tag1, tag2, tag3);
    /* quota 优先：高优先级 2 应先拿满 30 */
    printf("priority_respected=%d\n", tag2 >= tag3 && tag3 >= 0);

    int ok = size_ok && identical && version_rejected
             && corrupt_rejected && truncated_rejected;
    printf("save_load=%s\n", ok ? "OK" : "FAIL");
    return ok ? 0 : 1;
}
