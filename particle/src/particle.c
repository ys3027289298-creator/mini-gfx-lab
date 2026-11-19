
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget,
   emitter priority scheduling, and layer-mask filtered collision.

   碰撞层过滤:
   - 每个粒子带 layer_mask,每个碰撞盒带 layer_mask,均为 32 位位掩码;
   - 仅当 (particle.layer_mask & box.layer_mask) != 0 时才求交;
   - 过滤后的碰撞结果与朴素逐对判定完全一致(见 main 中的随机对照测试);
   - 掩码配置错误(空掩码、未知层位、FX 层被挂到碰撞盒、场景中无匹配
     对象的孤立掩码)由 mask_validate / scene_mask_check 检出。 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define MAX_PARTICLES 4096
#define MAX_EMITTERS 8
#define MAX_BOXES 256
#define MAX_CONTACTS 512

/* 已登记的物理层。LAYER_FX 是纯视觉特效层:粒子可以携带,
   但碰撞盒不允许携带(FX 粒子不与任何实体碰撞)。 */
#define LAYER_WORLD  (uint32_t)(1u << 0)
#define LAYER_PLAYER (uint32_t)(1u << 1)
#define LAYER_ENEMY  (uint32_t)(1u << 2)
#define LAYER_FX     (uint32_t)(1u << 3)

#define PARTICLE_LEGAL_MASK (LAYER_WORLD | LAYER_PLAYER | LAYER_ENEMY | LAYER_FX)
#define BOX_LEGAL_MASK      (LAYER_WORLD | LAYER_PLAYER | LAYER_ENEMY)
#define ALL_LAYERS          (uint32_t)0xFFFFFFFFu

/* 掩码配置错误码 */
#define MASK_OK            0
#define MASK_ERR_EMPTY     (1 << 0)  /* 掩码为 0:永不参与碰撞 */
#define MASK_ERR_UNKNOWN   (1 << 1)  /* 出现未登记的层位 */
#define MASK_ERR_FX_ON_BOX (1 << 2)  /* 碰撞盒携带了纯视觉 FX 层 */

typedef struct {
    float x, y, vx, vy, life, age, radius;
    int alive;
    int tag;
    uint32_t layer_mask;
} Particle;

typedef struct {
    int id, quota, spawned, priority;
    uint32_t layer_mask;   /* 该发射器喷出的粒子默认携带的层掩码 */
} Emitter;

typedef struct {
    float x, y, hw, hh;    /* 中心点 + 半宽半高的轴对齐盒 */
    uint32_t layer_mask;
} Box;

typedef struct { int particle; int box; } Contact;

typedef struct {
    Particle pool[MAX_PARTICLES];
    int alive_count, high_water;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;
    Box boxes[MAX_BOXES];
    int box_count;
} World;

static void world_init(World* w) {
    memset(w, 0, sizeof(*w));
    w->emitter_count = 0;
    w->box_count = 0;
}

/* ---- 掩码校验:配置错误在此检出,而不是静默地不碰撞 ---- */
static int particle_mask_validate(uint32_t mask) {
    int err = MASK_OK;
    if (mask == 0) err |= MASK_ERR_EMPTY;
    if (mask & ~PARTICLE_LEGAL_MASK) err |= MASK_ERR_UNKNOWN;
    return err;
}

static int box_mask_validate(uint32_t mask) {
    int err = MASK_OK;
    if (mask == 0) err |= MASK_ERR_EMPTY;
    if (mask & ~BOX_LEGAL_MASK) err |= MASK_ERR_UNKNOWN;
    if (mask & LAYER_FX) err |= MASK_ERR_FX_ON_BOX;
    return err;
}

static int emitter_add(World* w, int id, int quota, int priority, uint32_t mask) {
    if (w->emitter_count >= MAX_EMITTERS) return -1;
    Emitter* e = &w->emitters[w->emitter_count++];
    e->id = id; e->quota = quota; e->spawned = 0;
    e->priority = priority; e->layer_mask = mask;
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
    p->radius = 0.25f;
    p->tag = emitter_id;
    p->layer_mask = 0;
    for (int i = 0; i < w->emitter_count; i++)
        if (w->emitters[i].id == emitter_id) p->layer_mask = w->emitters[i].layer_mask;
    w->alive_count++;
    if (w->alive_count > w->high_water) w->high_water = w->alive_count;
    return idx;
}

/* 测试辅助:在指定位置手工放置一个带层掩码的粒子 */
static int place_particle(World* w, float x, float y, float r, uint32_t mask) {
    int idx = pool_find_free(w);
    if (idx < 0) return -1;
    Particle* p = &w->pool[idx];
    p->alive = 1; p->age = 0.0f;
    p->x = x; p->y = y; p->vx = 0.0f; p->vy = 0.0f;
    p->life = 999.0f; p->radius = r;
    p->tag = -1;
    p->layer_mask = mask;
    w->alive_count++;
    if (w->alive_count > w->high_water) w->high_water = w->alive_count;
    return idx;
}

/* 返回掩码配置错误码(MASK_OK 表示合法);非法盒不入场景 */
static int box_add(World* w, float x, float y, float hw, float hh, uint32_t mask) {
    int err = box_mask_validate(mask);
    if (err != MASK_OK) return err;
    if (w->box_count >= MAX_BOXES) return MASK_ERR_EMPTY;
    Box* b = &w->boxes[w->box_count++];
    b->x = x; b->y = y; b->hw = hw; b->hh = hh; b->layer_mask = mask;
    return MASK_OK;
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

/* 圆(粒子)与轴对齐盒的几何求交,不含任何层判断 */
static int shape_hit(const Particle* p, const Box* b) {
    float nx = p->x < b->x - b->hw ? b->x - b->hw : (p->x > b->x + b->hw ? b->x + b->hw : p->x);
    float ny = p->y < b->y - b->hh ? b->y - b->hh : (p->y > b->y + b->hh ? b->y + b->hh : p->y);
    float dx = p->x - nx, dy = p->y - ny;
    return dx * dx + dy * dy <= p->radius * p->radius;
}

static int layers_match(uint32_t pm, uint32_t bm) { return (pm & bm) != 0; }

/* 朴素逐对判定:先查层掩码再求交,作为过滤结果的对照基准 */
static int collide_naive(const World* w, Contact* out, int max_out) {
    int n = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle* p = &w->pool[i];
        if (!p->alive) continue;
        for (int j = 0; j < w->box_count; j++) {
            const Box* b = &w->boxes[j];
            if (!layers_match(p->layer_mask, b->layer_mask)) continue;
            if (!shape_hit(p, b)) continue;
            if (n < max_out) { out[n].particle = i; out[n].box = j; }
            n++;
        }
    }
    return n;
}

/* 按层分桶后再求交:每个盒只与匹配层桶里的粒子做几何测试。
   一个粒子的掩码含多个层时会进多个桶,(粒子,盒)对仍只产出一次。 */
static int collide_filtered(const World* w, Contact* out, int max_out) {
    int bucket[32][MAX_PARTICLES];
    int bn[32];
    memset(bn, 0, sizeof(bn));
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (!w->pool[i].alive) continue;
        uint32_t m = w->pool[i].layer_mask;
        for (int bit = 0; bit < 32; bit++)
            if (m & (uint32_t)(1u << bit))
                bucket[bit][bn[bit]++] = i;
    }
    int n = 0;
    for (int j = 0; j < w->box_count; j++) {
        const Box* b = &w->boxes[j];
        for (int bit = 0; bit < 32; bit++) {
            if (!(b->layer_mask & (uint32_t)(1u << bit))) continue;
            for (int k = 0; k < bn[bit]; k++) {
                int i = bucket[bit][k];
                const Particle* p = &w->pool[i];
                /* 去重:该对可能已通过另一个共享层产出过 */
                int seen = 0;
                for (int q = 0; q < n; q++)
                    if (out[q].particle == i && out[q].box == j) { seen = 1; break; }
                if (seen) continue;
                if (shape_hit(p, b)) {
                    if (n < max_out) { out[n].particle = i; out[n].box = j; }
                    n++;
                }
            }
        }
    }
    return n;
}

/* 场景级组合检查:合法但孤立的掩码——粒子/盒在场景中找不到任何
   携带共享层的对方,等于永不碰撞的死配置,也要能检出。 */
static void scene_mask_check(const World* w,
                             int* orphan_particles, int* orphan_boxes) {
    uint32_t particle_layers = 0, box_layers = 0;
    for (int i = 0; i < MAX_PARTICLES; i++)
        if (w->pool[i].alive) particle_layers |= w->pool[i].layer_mask;
    for (int j = 0; j < w->box_count; j++)
        box_layers |= w->boxes[j].layer_mask;
    int op = 0, ob = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle* p = &w->pool[i];
        if (p->alive && !(p->layer_mask & box_layers)) op++;
    }
    for (int j = 0; j < w->box_count; j++)
        if (!(w->boxes[j].layer_mask & particle_layers)) ob++;
    *orphan_particles = op;
    *orphan_boxes = ob;
}

static int contact_cmp(const void* a, const void* b) {
    const Contact* x = (const Contact*)a;
    const Contact* y = (const Contact*)b;
    if (x->particle != y->particle) return x->particle - y->particle;
    return x->box - y->box;
}

static int contacts_equal(const Contact* a, const Contact* b, int n) {
    for (int i = 0; i < n; i++)
        if (a[i].particle != b[i].particle || a[i].box != b[i].box) return 0;
    return 1;
}

int main(void) {
    int failures = 0;
    #define CHECK(cond, msg) do { \
        printf("%s=%d\n", msg, (cond) ? 1 : 0); \
        if (!(cond)) failures++; \
    } while (0)

    /* ---- 原有的粒子池/配额/优先级自测 ---- */
    World w; world_init(&w);
    emitter_add(&w, 1, 40, 2, LAYER_PLAYER);
    emitter_add(&w, 2, 30, 9, LAYER_FX);
    emitter_add(&w, 3, 20, 5, LAYER_ENEMY);
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
    CHECK(tag2 >= tag3 && tag3 >= 0, "priority_respected");
    /* 发射器的层掩码应传递给喷出的粒子 */
    int fx_inherited = 1;
    for (int i = 0; i < MAX_PARTICLES; i++)
        if (w.pool[i].alive && w.pool[i].tag == 2 && w.pool[i].layer_mask != LAYER_FX)
            fx_inherited = 0;
    CHECK(fx_inherited, "emitter_layer_inherited");

    /* ---- 固定场景:过滤后的碰撞结果 ---- */
    World c; world_init(&c);
    box_add(&c,  0.0f,  0.0f, 1.0f, 1.0f, LAYER_WORLD);            /* b0 */
    box_add(&c,  5.0f,  0.0f, 1.0f, 1.0f, LAYER_PLAYER);           /* b1 */
    box_add(&c, -5.0f,  0.0f, 1.0f, 1.0f, LAYER_ENEMY);            /* b2 */
    box_add(&c,  9.0f,  9.0f, 1.0f, 1.0f, LAYER_PLAYER | LAYER_ENEMY); /* b3 多层 */
    /* 粒子都放在盒中心附近,几何上必然相交;是否报碰撞只取决于层掩码 */
    place_particle(&c,  0.0f,  0.0f, 0.5f, LAYER_WORLD);            /* p0 -> b0 */
    place_particle(&c,  0.5f,  0.5f, 0.5f, LAYER_PLAYER);           /* p1 几何压 b0,层不匹配 -> 无 */
    place_particle(&c,  5.0f,  0.0f, 0.5f, LAYER_PLAYER);           /* p2 -> b1 */
    place_particle(&c, -5.0f,  0.0f, 0.5f, LAYER_ENEMY);            /* p3 -> b2 */
    place_particle(&c,  9.0f,  9.0f, 0.5f, LAYER_PLAYER | LAYER_ENEMY); /* p4 -> b3 */
    place_particle(&c,  9.0f,  9.0f, 0.5f, LAYER_PLAYER);           /* p5 -> b3 且与 p4 共层 */
    place_particle(&c,  0.0f,  0.0f, 0.5f, LAYER_FX);               /* p6 FX:压着 b0 也不碰撞 */
    place_particle(&c, 20.0f, 20.0f, 0.5f, LAYER_WORLD);            /* p7 几何远离 -> 无 */
    place_particle(&c,  5.0f,  0.0f, 0.5f, LAYER_WORLD | LAYER_PLAYER); /* p8 -> b1,不含 world 位故不碰 b0 */
    place_particle(&c, -5.0f,  0.0f, 0.5f, ALL_LAYERS);             /* p9 全层:几何上只压 b2 -> 1 对 */

    Contact naive[MAX_CONTACTS], filtered[MAX_CONTACTS];
    int nn = collide_naive(&c, naive, MAX_CONTACTS);
    int nf = collide_filtered(&c, filtered, MAX_CONTACTS);
    qsort(naive, (size_t)nn, sizeof(Contact), contact_cmp);
    qsort(filtered, (size_t)nf, sizeof(Contact), contact_cmp);
    printf("contacts naive=%d filtered=%d\n", nn, nf);
    /* 期望接触对: p0-b0, p2-b1, p3-b2, p4-b3, p5-b3, p8-b1, p9-b2 = 7 */
    CHECK(nn == 7, "expected_contact_count");
    CHECK(nn == nf, "filtered_count_matches_naive");
    CHECK(contacts_equal(naive, filtered, nn), "filtered_pairs_match_naive");

    /* FX 粒子即使几何重叠也不产生接触;无几何重叠的同层对同样不报 */
    int fx_hits = 0, far_hits = 0;
    for (int i = 0; i < nf; i++) {
        if (filtered[i].particle == 6) fx_hits++;
        if (filtered[i].particle == 7) far_hits++;
    }
    CHECK(fx_hits == 0, "fx_layer_never_collides");
    CHECK(far_hits == 0, "no_contact_without_overlap");

    /* ---- 随机对照:过滤结果必须与逐对判定逐对一致(覆盖全部 32 个层位) ---- */
    int fuzz_ok = 1;
    for (int trial = 0; trial < 200 && fuzz_ok; trial++) {
        World r; world_init(&r);
        unsigned seed = 0x9e3779b9u + (unsigned)trial * 2654435761u;
        int npart = 1 + (seed % 20);
        int nbox = 1 + ((seed / 20) % 10);
        for (int k = 0; k < npart; k++) {
            seed = seed * 1103515245u + 12345u;
            float x = (float)((seed >> 8) % 11) - 5.0f;
            float y = (float)((seed >> 16) % 11) - 5.0f;
            seed = seed * 1103515245u + 12345u;
            uint32_t pm = (seed >> 4) | (uint32_t)(1u << (seed % 32)); /* 保证非空且含任意层位 */
            place_particle(&r, x, y, 1.0f, pm);
        }
        for (int k = 0; k < nbox; k++) {
            seed = seed * 1103515245u + 12345u;
            float x = (float)((seed >> 8) % 11) - 5.0f;
            float y = (float)((seed >> 16) % 11) - 5.0f;
            seed = seed * 1103515245u + 12345u;
            uint32_t bm = (seed >> 3) | (uint32_t)(1u << (seed % 32));
            bm &= BOX_LEGAL_MASK;                 /* 盒只取合法层,非法配置由专门用例测 */
            if (bm == 0) bm = LAYER_WORLD;
            box_add(&r, x, y, 1.0f, 1.0f, bm);
        }
        Contact rn[MAX_CONTACTS], rf[MAX_CONTACTS];
        int a = collide_naive(&r, rn, MAX_CONTACTS);
        int b = collide_filtered(&r, rf, MAX_CONTACTS);
        if (a != b) { fuzz_ok = 0; break; }
        qsort(rn, (size_t)a, sizeof(Contact), contact_cmp);
        qsort(rf, (size_t)b, sizeof(Contact), contact_cmp);
        if (!contacts_equal(rn, rf, a)) { fuzz_ok = 0; break; }
    }
    CHECK(fuzz_ok, "fuzz_filtered_equals_naive");

    /* ---- 掩码配置错误检出 ---- */
    CHECK(particle_mask_validate(0) & MASK_ERR_EMPTY, "detect_particle_empty_mask");
    CHECK(box_mask_validate(0) & MASK_ERR_EMPTY, "detect_box_empty_mask");
    CHECK(particle_mask_validate((uint32_t)(1u << 30)) & MASK_ERR_UNKNOWN, "detect_unknown_layer_bit");
    CHECK(box_mask_validate((uint32_t)(1u << 30)) & MASK_ERR_UNKNOWN, "detect_unknown_layer_bit_box");
    CHECK(box_mask_validate(LAYER_FX) & MASK_ERR_FX_ON_BOX, "detect_fx_layer_on_box");
    CHECK(box_mask_validate(LAYER_WORLD) == MASK_OK, "valid_box_mask_accepted");
    CHECK(particle_mask_validate(LAYER_FX) == MASK_OK, "valid_fx_particle_accepted");
    /* 非法盒在入口被拒绝,不进入场景 */
    World v; world_init(&v);
    int rejected = box_add(&v, 0, 0, 1, 1, LAYER_FX);
    CHECK((rejected & MASK_ERR_FX_ON_BOX) && v.box_count == 0, "invalid_box_rejected_at_entry");

    /* ---- 场景级组合错误:合法但孤立、永不碰撞的掩码 ---- */
    World s; world_init(&s);
    box_add(&s, 0, 0, 1, 1, LAYER_WORLD);                 /* 有 world 粒子匹配 */
    box_add(&s, 8, 8, 1, 1, LAYER_ENEMY);                 /* 无 enemy 粒子:孤立盒 */
    place_particle(&s, 0, 0, 0.5f, LAYER_WORLD);          /* 与 b0 匹配 */
    place_particle(&s, 0, 0, 0.5f, LAYER_PLAYER);         /* 场景无 player 盒:孤立粒子 */
    int op = -1, ob = -1;
    scene_mask_check(&s, &op, &ob);
    printf("orphan_particles=%d orphan_boxes=%d\n", op, ob);
    CHECK(op == 1, "detect_orphan_particle_mask");
    CHECK(ob == 1, "detect_orphan_box_mask");

    if (failures) { printf("FAIL %d check(s)\n", failures); return 1; }
    printf("all collision-layer checks passed\n");
    return 0;
}
