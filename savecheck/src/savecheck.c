/* savecheck: checkpoint / rollback for the mini-gfx-lab world.
   A checkpoint is a self-describing package of every mutable cell in the
   World; a rollback inflates it back into a World and must match, field for
   field, a fresh init replayed to the same tick. Deltas are encoded against
   a base checkpoint and carry only changed cells. Checkpoint size and
   restore time are reported by the self-test. */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

/* ------------------------------------------------------------------ world */

#define MAX_ENTS 24
#define MAX_EMIT 3
#define MAX_KEYS 8
#define MAX_PART 48          /* live particles; pool is compacted */
#define MAX_CELLS 640

typedef enum { K_LINEAR = 0, K_QUAD_IN, K_QUAD_OUT, K_CUBIC_IO, K_STEP } KEase;

typedef struct {
    float pos[3];
    float vel[3];
    float yaw;
    float hp;
    float anim_t;            /* playback position, wraps at the last key */
    float anim_v;            /* playback speed */
    int anim_keys;           /* keyframe count, checked against the scene */
    int state;               /* 1 alive, 2 dead */
    int solid;               /* derived from geometry */
} Entity;

typedef struct { int id, left, prio; float pace; } Emitter;
typedef struct { int player, emitter; float ttl; } Particle;

typedef struct {
    uint64_t tick;
    float clock;
    int32_t rng;             /* hidden state: drives emitter quotas */
    float hit_until;
    Entity ent[MAX_ENTS];   int n_ent;
    Emitter emit[MAX_EMIT]; int n_emit;
    Particle part[MAX_PART]; int n_part;
} World;

/* cell layout: every mutable value is one u32 lane, so a field has a stable
   index and deltas can be expressed as (index, value) pairs */
enum {
    MF_TICK_LO = 0, MF_TICK_HI, MF_CLOCK, MF_RNG, MF_HIT_UNTIL,
    MF_N_ENT, MF_N_EMIT, MF_N_PART, MF_COUNT
};
enum {
    EF_POSX = 0, EF_POSY, EF_POSZ, EF_VELX, EF_VELY, EF_VELZ,
    EF_YAW, EF_HP, EF_ANIM_T, EF_ANIM_V, EF_ANIM_KEYS, EF_STATE, EF_SOLID, EF_COUNT
};
enum { EMF_ID = 0, EMF_LEFT, EMF_PACE, EMF_PRIO, EMF_COUNT };
enum { PF_PLAYER = 0, PF_EMITTER, PF_TTL, PF_COUNT };

/* ------------------------------------------------------------------ scene */

typedef struct { float t, v; KEase ease; } KeyDef;
typedef struct {
    const char* name;
    float pos[3];
    float hp;
    float anim_v;
    int state;
    int atk;
    KeyDef keys[MAX_KEYS];
    int nkey;
} EntDef;
typedef struct { int id, left, prio; float pace; } EmitDef;

typedef struct {
    const char* name;
    float dt;
    int seed;
    int player;
    EntDef ents[8]; int n_ent;
    EmitDef emits[MAX_EMIT]; int n_emit;
    int hit_ticks[32]; int n_hit;
} Scene;

static const Scene kSandbox = {
    "sandbox", 1.0f / 60.0f, 7, 1,
    {
        { "player", { 4.0f, 4.0f, 1.0f }, 1.0f, 1.00f, 1, 1, {
            { 0.0f, 4.0f, K_LINEAR }, { 0.5f, 0.0f, K_QUAD_IN }, { 1.0f, 100.0f, K_LINEAR } }, 3 },
        { "crate", { 9.0f, 2.0f, 1.0f }, 40.0f, 1.05f, 1, 0, {
            { 0.2f, 2.0f, K_CUBIC_IO }, { 0.6f, 30.0f, K_STEP } }, 2 },
        { "drone", { 7.0f, 8.0f, 1.0f }, 0.9f, 1.10f, 1, 0, {
            { 0.3f, 7.0f, K_QUAD_OUT }, { 0.4f, 8.0f, K_LINEAR } }, 2 },
        { "pickup", { 2.0f, 6.0f, 1.0f }, 10.0f, 1.15f, 1, 0, {
            { 0.0f, 0.0f, K_LINEAR } }, 1 },
    }, 4,
    { { 1, 2, 9, 0.25f }, { 2, 3, 2, 0.50f }, { 3, 1, 5, 0.75f } }, 3,
    { 8, 20, 95, 140 }, 4
};

static float key_eval(int nk, const KeyDef* keys, float t) {
    if (nk <= 0) return 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    if (t <= keys[0].t) return keys[0].v;
    for (int i = 1; i < nk; i++) {
        const KeyDef* a = &keys[i - 1];
        const KeyDef* b = &keys[i];
        if (t <= b->t) {
            float span = b->t - a->t;
            float u = span > 0.0f ? (t - a->t) / span : 1.0f;
            float w;
            switch (a->ease) {
            case K_QUAD_IN: w = u * u; break;
            case K_QUAD_OUT: w = 1.0f - (1.0f - u) * (1.0f - u); break;
            case K_CUBIC_IO: w = u < 0.5f ? 4.0f * u * u * u
                                          : 1.0f - powf(-2.0f * u + 2.0f, 3.0f) / 2.0f; break;
            case K_STEP: w = u < 1.0f ? 0.0f : 1.0f; break;
            default: w = u; break;
            }
            return a->v + (b->v - a->v) * w;
        }
    }
    return keys[nk - 1].v;
}

static float anim_end(const Scene* sc, int i) {
    if (i < 0 || i >= sc->n_ent || sc->ents[i].nkey == 0) return 0.0f;
    return sc->ents[i].keys[sc->ents[i].nkey - 1].t;
}
static float anim_frame(const Scene* sc, int i, float anim_t) {
    if (i < 0 || i >= sc->n_ent) return 0.0f;
    const EntDef* d = &sc->ents[i];
    return key_eval(d->nkey, d->keys, anim_t);
}

/* ------------------------------------------------------------- simulation */

static void world_init(const Scene* sc, World* w) {
    memset(w, 0, sizeof(*w));
    w->tick = 0;
    w->clock = 0.0f;
    w->rng = sc->seed;
    w->hit_until = -1.0f;
    w->n_ent = sc->n_ent;
    for (int i = 0; i < sc->n_ent; i++) {
        const EntDef* d = &sc->ents[i];
        Entity* e = &w->ent[i];
        e->pos[0] = d->pos[0]; e->pos[1] = d->pos[1]; e->pos[2] = d->pos[2];
        e->vel[0] = d->pos[0] * 0.1f;
        e->vel[1] = (float)d->atk * 0.01f;
        e->vel[2] = 0.05f;
        e->yaw = 0.2f + 0.3f * (float)i;
        e->hp = d->hp;
        e->anim_t = 0.05f * (float)i;
        e->anim_v = d->anim_v;
        e->anim_keys = d->nkey;
        e->state = d->state;
        e->solid = (e->pos[1] >= 0.5f);
    }
    w->n_emit = sc->n_emit;
    for (int i = 0; i < sc->n_emit; i++) {
        w->emit[i].id = sc->emits[i].id;
        w->emit[i].left = sc->emits[i].left;
        w->emit[i].pace = sc->emits[i].pace;
        w->emit[i].prio = sc->emits[i].prio;
    }
    w->n_part = 0;
}

/* local player chases the drone; only uses state inside the World */
static void step_control(World* w) {
    if (w->n_ent < 3) return;
    Entity* p = &w->ent[0];
    const Entity* d = &w->ent[2];
    p->vel[0] = (d->pos[0] - p->pos[0]) * 0.02f;
    p->vel[2] = (d->pos[2] - p->pos[2]) * 0.02f;
    p->yaw += 0.01f + d->pos[1] * 0.001f;
}

static void step_emitters(World* w) {
    for (int i = 0; i < w->n_emit; i++) {
        Emitter* e = &w->emit[i];
        e->left = (int)(((uint32_t)w->rng >> 16) % 4u) + 1;
        e->pace = 0.10f + 0.01f * (float)e->left;
    }
}

static void step_particles(World* w) {
    int out = 0;
    for (int i = 0; i < w->n_part; i++) {
        Particle q = w->part[i];
        q.ttl -= 0.015f + 0.005f * (float)(q.emitter + 1);
        if (q.ttl > 0.0f) w->part[out++] = q;
    }
    w->n_part = out;
    for (int k = 0; k < w->n_emit && w->n_part < MAX_PART; k++) {
        for (int n = 0; n < w->emit[k].left && w->n_part < MAX_PART; n++) {
            Particle q;
            q.player = w->n_ent > 0 ? w->emit[k].id % w->n_ent : 0;
            q.emitter = k;
            q.ttl = 0.35f + 0.01f * (float)w->emit[k].left;
            w->part[w->n_part++] = q;
        }
    }
}

static int scene_hit(const Scene* sc, uint64_t tick) {
    for (int i = 0; i < sc->n_hit; i++)
        if ((uint64_t)sc->hit_ticks[i] == tick) return 1;
    return 0;
}

/* exactly one discrete step; tick never advances by more than one */
static void world_step(const Scene* sc, World* w) {
    w->tick += 1;
    w->clock = (float)w->tick * sc->dt;
    w->rng = (int32_t)((uint32_t)w->rng * 1103515245u + 12345u);

    step_control(w);
    for (int i = 0; i < w->n_ent; i++) {
        Entity* e = &w->ent[i];
        for (int a = 0; a < 3; a++) e->pos[a] += e->vel[a] * sc->dt;
        float end = anim_end(sc, i);
        float t = e->anim_t + e->anim_v * sc->dt;
        if (end > 0.0f) {
            t = fmodf(t, end);
            if (t < 0.0f) t += end;
        } else {
            t = 0.0f;
        }
        e->anim_t = t;
    }
    step_emitters(w);
    step_particles(w);

    int hit = scene_hit(sc, w->tick);
    if (hit && w->n_ent > 0) {
        int idx = (int)((w->tick ^ (w->tick >> 3)) % (uint64_t)w->n_ent);
        w->ent[idx].hp -= 1.25f;
        w->hit_until = w->clock;
    }
    for (int i = 0; i < w->n_ent; i++) {
        if (w->ent[i].hp <= 0.0f && w->ent[i].state == 1) w->ent[i].state = 2;
        w->ent[i].solid = (w->ent[i].pos[1] >= 0.5f);
    }
}

static void world_play(const Scene* sc, World* w, int ticks) {
    world_init(sc, w);
    for (int i = 0; i < ticks; i++) world_step(sc, w);
}
static void world_advance(const Scene* sc, World* w, int ticks) {
    for (int i = 0; i < ticks; i++) world_step(sc, w);
}

/* --------------------------------------------------------- cell mapping */

static int cell_count(const World* w) {
    return MF_COUNT + EF_COUNT * w->n_ent + EMF_COUNT * w->n_emit
         + PF_COUNT * w->n_part + w->n_ent;   /* trailing: derived anim frames */
}
static int off_emit(const World* w) { return MF_COUNT + EF_COUNT * w->n_ent; }
static int off_part(const World* w) { return off_emit(w) + EMF_COUNT * w->n_emit; }
static int off_anim(const World* w) { return off_part(w) + PF_COUNT * w->n_part; }

static uint32_t f2u(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }
static float u2f(uint32_t u) { float f; memcpy(&f, &u, 4); return f; }

static int world_to_cells(const World* w, const Scene* sc, uint32_t* out, int cap) {
    int n = cell_count(w);
    if (n > cap || w->n_ent > MAX_ENTS || w->n_emit > MAX_EMIT || w->n_part > MAX_PART) return -1;
    memset(out, 0, sizeof(uint32_t) * (size_t)n);
    out[MF_TICK_LO] = (uint32_t)(w->tick & 0xffffffffu);
    out[MF_TICK_HI] = (uint32_t)(w->tick >> 32);
    out[MF_CLOCK] = f2u(w->clock);
    out[MF_RNG] = (uint32_t)w->rng;
    out[MF_HIT_UNTIL] = f2u(w->hit_until);
    out[MF_N_ENT] = (uint32_t)w->n_ent;
    out[MF_N_EMIT] = (uint32_t)w->n_emit;
    out[MF_N_PART] = (uint32_t)w->n_part;
    for (int i = 0; i < w->n_ent; i++) {
        const Entity* e = &w->ent[i];
        uint32_t* c = out + MF_COUNT + EF_COUNT * i;
        c[EF_POSX] = f2u(e->pos[0]); c[EF_POSY] = f2u(e->pos[1]); c[EF_POSZ] = f2u(e->pos[2]);
        c[EF_VELX] = f2u(e->vel[0]); c[EF_VELY] = f2u(e->vel[1]); c[EF_VELZ] = f2u(e->vel[2]);
        c[EF_YAW] = f2u(e->yaw); c[EF_HP] = f2u(e->hp);
        c[EF_ANIM_T] = f2u(e->anim_t); c[EF_ANIM_V] = f2u(e->anim_v);
        c[EF_ANIM_KEYS] = (uint32_t)e->anim_keys;
        c[EF_STATE] = (uint32_t)e->state; c[EF_SOLID] = (uint32_t)e->solid;
    }
    for (int i = 0; i < w->n_emit; i++) {
        uint32_t* c = out + off_emit(w) + EMF_COUNT * i;
        c[EMF_ID] = (uint32_t)w->emit[i].id;
        c[EMF_LEFT] = (uint32_t)w->emit[i].left;
        c[EMF_PACE] = f2u(w->emit[i].pace);
        c[EMF_PRIO] = (uint32_t)w->emit[i].prio;
    }
    for (int i = 0; i < w->n_part; i++) {
        uint32_t* c = out + off_part(w) + PF_COUNT * i;
        c[PF_PLAYER] = (uint32_t)w->part[i].player;
        c[PF_EMITTER] = (uint32_t)w->part[i].emitter;
        c[PF_TTL] = f2u(w->part[i].ttl);
    }
    for (int i = 0; i < w->n_ent; i++)
        out[off_anim(w) + i] = f2u(anim_frame(sc, i, w->ent[i].anim_t));
    return n;
}

static int cells_to_world(const Scene* sc, const uint32_t* c, int n, World* w, char* err, size_t ecap) {
    if (n < MF_COUNT) { snprintf(err, ecap, "too_short"); return -1; }
    int ne = (int)c[MF_N_ENT], nm = (int)c[MF_N_EMIT], np = (int)c[MF_N_PART];
    if (ne < 0 || ne > MAX_ENTS || nm < 0 || nm > MAX_EMIT || np < 0 || np > MAX_PART) {
        snprintf(err, ecap, "bad_counts"); return -1;
    }
    if (n != MF_COUNT + EF_COUNT * ne + EMF_COUNT * nm + PF_COUNT * np + ne) {
        snprintf(err, ecap, "bad_shape"); return -1;
    }
    memset(w, 0, sizeof(*w));
    w->tick = ((uint64_t)c[MF_TICK_HI] << 32) | (uint64_t)c[MF_TICK_LO];
    w->clock = u2f(c[MF_CLOCK]);
    w->rng = (int32_t)c[MF_RNG];
    w->hit_until = u2f(c[MF_HIT_UNTIL]);
    w->n_ent = ne; w->n_emit = nm; w->n_part = np;
    for (int i = 0; i < ne; i++) {
        const uint32_t* q = c + MF_COUNT + EF_COUNT * i;
        Entity* e = &w->ent[i];
        e->pos[0] = u2f(q[EF_POSX]); e->pos[1] = u2f(q[EF_POSY]); e->pos[2] = u2f(q[EF_POSZ]);
        e->vel[0] = u2f(q[EF_VELX]); e->vel[1] = u2f(q[EF_VELY]); e->vel[2] = u2f(q[EF_VELZ]);
        e->yaw = u2f(q[EF_YAW]); e->hp = u2f(q[EF_HP]);
        e->anim_t = u2f(q[EF_ANIM_T]); e->anim_v = u2f(q[EF_ANIM_V]);
        e->anim_keys = (int)q[EF_ANIM_KEYS];
        e->state = (int)q[EF_STATE]; e->solid = (int)q[EF_SOLID];
        if (e->anim_keys != (i < sc->n_ent ? sc->ents[i].nkey : 0)) {
            snprintf(err, ecap, "scene_keys_mismatch"); return -1;
        }
    }
    for (int i = 0; i < nm; i++) {
        const uint32_t* q = c + off_emit(w) + EMF_COUNT * i;
        w->emit[i].id = (int)q[EMF_ID];
        w->emit[i].left = (int)q[EMF_LEFT];
        w->emit[i].pace = u2f(q[EMF_PACE]);
        w->emit[i].prio = (int)q[EMF_PRIO];
    }
    for (int i = 0; i < np; i++) {
        const uint32_t* q = c + off_part(w) + PF_COUNT * i;
        w->part[i].player = (int)q[PF_PLAYER];
        w->part[i].emitter = (int)q[PF_EMITTER];
        w->part[i].ttl = u2f(q[PF_TTL]);
    }
    /* derived fields must agree with the truth they are computed from */
    for (int i = 0; i < ne; i++) {
        float frame = anim_frame(sc, i, w->ent[i].anim_t);
        if (f2u(frame) != c[off_anim(w) + i]) {
            snprintf(err, ecap, "derived_anim_mismatch"); return -1;
        }
        int solid = (w->ent[i].pos[1] >= 0.5f);
        if (solid != w->ent[i].solid) {
            snprintf(err, ecap, "derived_solid_mismatch"); return -1;
        }
    }
    return 0;
}

static const char* cell_name(const World* w, int idx, char* buf, size_t cap) {
    static const char* mn[MF_COUNT] = {
        "tick.lo", "tick.hi", "clock", "rng", "hit_until", "n_ent", "n_emit", "n_part"
    };
    static const char* en[EF_COUNT] = {
        "pos.x", "pos.y", "pos.z", "vel.x", "vel.y", "vel.z",
        "yaw", "hp", "anim_t", "anim_v", "anim_keys", "state", "solid"
    };
    static const char* emn[EMF_COUNT] = { "id", "left", "pace", "prio" };
    static const char* pn[PF_COUNT] = { "player", "emitter", "ttl" };

    if (idx < MF_COUNT) { snprintf(buf, cap, "%s", mn[idx]); return buf; }
    idx -= MF_COUNT;
    if (idx < EF_COUNT * w->n_ent) {
        snprintf(buf, cap, "ent.%d.%s", idx / EF_COUNT, en[idx % EF_COUNT]);
        return buf;
    }
    idx -= EF_COUNT * w->n_ent;
    if (idx < EMF_COUNT * w->n_emit) {
        snprintf(buf, cap, "emitter.%d.%s", idx / EMF_COUNT, emn[idx % EMF_COUNT]);
        return buf;
    }
    idx -= EMF_COUNT * w->n_emit;
    if (idx < PF_COUNT * w->n_part) {
        snprintf(buf, cap, "particle.%d.%s", idx / PF_COUNT, pn[idx % PF_COUNT]);
        return buf;
    }
    idx -= PF_COUNT * w->n_part;
    if (idx < w->n_ent) {
        snprintf(buf, cap, "ent.%d.anim.frame", idx);
        return buf;
    }
    snprintf(buf, cap, "cell.%d", idx);
    return buf;
}

/* field-by-field comparison, reading through the cell mapping */
static int compare_worlds(const World* a, const World* b, const Scene* sc, int max_report) {
    uint32_t ca[MAX_CELLS], cb[MAX_CELLS];
    int na = world_to_cells(a, sc, ca, MAX_CELLS);
    int nb = world_to_cells(b, sc, cb, MAX_CELLS);
    if (na < 0 || nb < 0) { printf("compare_overflow=1\n"); return 1; }
    if (na != nb) {
        printf("shape_mismatch=%d,%d\n", na, nb);
        return 1;
    }
    int bad = 0;
    char name[64];
    for (int i = 0; i < na; i++) {
        if (ca[i] == cb[i]) continue;
        bad++;
        if (bad <= max_report)
            printf("mismatch_field=%s a=0x%08x b=0x%08x\n",
                   cell_name(a, i, name, sizeof name), ca[i], cb[i]);
    }
    return bad;
}

static uint32_t world_hash(const World* w, const Scene* sc) {
    uint32_t c[MAX_CELLS];
    int n = world_to_cells(w, sc, c, MAX_CELLS);
    uint32_t h = 2166136261u;
    for (int i = 0; i < n; i++) { h ^= c[i]; h *= 16777619u; h ^= h >> 13; }
    return h;
}

/* --------------------------------------------------------------- package */

typedef struct { unsigned char* data; size_t len, cap; } Buf;

static void buf_grow(Buf* b, size_t extra) {
    if (b->len + extra <= b->cap) return;
    size_t cap = b->cap ? b->cap : 256;
    while (cap < b->len + extra) cap *= 2;
    unsigned char* p = (unsigned char*)realloc(b->data, cap);
    if (!p) { fprintf(stderr, "oom\n"); exit(1); }
    b->data = p; b->cap = cap;
}
static void buf_bytes(Buf* b, const void* p, size_t n) {
    if (n == 0) return;
    buf_grow(b, n);
    memcpy(b->data + b->len, p, n);
    b->len += n;
}
static void buf_u8(Buf* b, uint8_t v) { buf_bytes(b, &v, 1); }
static void buf_u16(Buf* b, uint16_t v) {
    unsigned char p[2] = { (unsigned char)(v & 0xff), (unsigned char)(v >> 8) };
    buf_bytes(b, p, 2);
}
static void buf_u32(Buf* b, uint32_t v) {
    unsigned char p[4] = { (unsigned char)(v & 0xff), (unsigned char)((v >> 8) & 0xff),
                           (unsigned char)((v >> 16) & 0xff), (unsigned char)(v >> 24) };
    buf_bytes(b, p, 4);
}
static void buf_free(Buf* b) { free(b->data); b->data = NULL; b->len = b->cap = 0; }

typedef struct { const unsigned char* p; size_t n, off; } Reader;
static int rd_bytes(Reader* r, void* out, size_t n) {
    if (r->off + n > r->n) return -1;
    memcpy(out, r->p + r->off, n);
    r->off += n;
    return 0;
}
static int rd_u8(Reader* r, uint8_t* v) { return rd_bytes(r, v, 1); }
static int rd_u16(Reader* r, uint16_t* v) {
    unsigned char p[2];
    if (rd_bytes(r, p, 2)) return -1;
    *v = (uint16_t)(p[0] | (p[1] << 8));
    return 0;
}
static int rd_u32(Reader* r, uint32_t* v) {
    unsigned char p[4];
    if (rd_bytes(r, p, 4)) return -1;
    *v = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return 0;
}

static uint32_t fnv(uint32_t h, const void* p, size_t n) {
    const unsigned char* b = (const unsigned char*)p;
    for (size_t i = 0; i < n; i++) { h ^= b[i]; h *= 16777619u; h ^= h >> 13; }
    return h;
}

#define PKG_MAGIC "MGSC"
#define PKG_VERSION 1
#define PKG_FLAG_DELTA 0x0001

enum { CH_META = 0, CH_ENT, CH_EMIT, CH_PART, CH_ANIM, CH_COUNT };

static uint32_t scene_digest(const Scene* sc) {
    uint32_t h = 2166136261u;
    h = fnv(h, sc->name, strlen(sc->name));
    h = fnv(h, &sc->dt, 4);
    h = fnv(h, &sc->seed, 4);
    h = fnv(h, &sc->player, 4);
    h = fnv(h, &sc->n_ent, 4);
    h = fnv(h, &sc->n_emit, 4);
    for (int i = 0; i < sc->n_ent; i++) {
        const EntDef* d = &sc->ents[i];
        h = fnv(h, d->name, strlen(d->name));
        h = fnv(h, d->pos, sizeof d->pos);
        h = fnv(h, &d->hp, 4);
        h = fnv(h, &d->anim_v, 4);
        h = fnv(h, &d->state, 4);
        h = fnv(h, &d->atk, 4);
        h = fnv(h, &d->nkey, 4);
        for (int k = 0; k < d->nkey; k++) {
            h = fnv(h, &d->keys[k].t, 4);
            h = fnv(h, &d->keys[k].v, 4);
            h = fnv(h, &d->keys[k].ease, 4);
        }
    }
    for (int i = 0; i < sc->n_emit; i++)
        h = fnv(h, &sc->emits[i], sizeof sc->emits[i]);
    h = fnv(h, &sc->n_hit, 4);
    for (int i = 0; i < sc->n_hit; i++) h = fnv(h, &sc->hit_ticks[i], 4);
    return h;
}

typedef struct { int id; int off; int count; } Slice;

static int build_slices(const uint32_t* meta, Slice* s, int* total) {
    int ne = (int)meta[MF_N_ENT], nm = (int)meta[MF_N_EMIT], np = (int)meta[MF_N_PART];
    if (ne < 0 || ne > MAX_ENTS || nm < 0 || nm > MAX_EMIT || np < 0 || np > MAX_PART) return -1;
    int o = 0;
    s[CH_META].id = CH_META; s[CH_META].off = o; s[CH_META].count = MF_COUNT; o += MF_COUNT;
    s[CH_ENT].id = CH_ENT; s[CH_ENT].off = o; s[CH_ENT].count = EF_COUNT * ne; o += EF_COUNT * ne;
    s[CH_EMIT].id = CH_EMIT; s[CH_EMIT].off = o; s[CH_EMIT].count = EMF_COUNT * nm; o += EMF_COUNT * nm;
    s[CH_PART].id = CH_PART; s[CH_PART].off = o; s[CH_PART].count = PF_COUNT * np; o += PF_COUNT * np;
    s[CH_ANIM].id = CH_ANIM; s[CH_ANIM].off = o; s[CH_ANIM].count = ne; o += ne;
    if (o > MAX_CELLS) return -1;
    *total = o;
    return 0;
}

/* encode: when delta != 0, every chunk copies what the full version would
   write, then marks the cells that differ from the base. The base checkpoints
   elsewhere in the array are the authoritative stored cells, so the delta
   reproduces a full save byte for byte after being applied to the base. */
static int pkg_encode(const Scene* sc, int delta, const uint32_t* base, int base_n,
                      const uint32_t* tgt, int tgt_n, Buf* out, int* changed_out) {
    Slice ts[CH_COUNT], bs[CH_COUNT];
    int ttotal = 0, btotal = 0;
    if (build_slices(tgt, ts, &ttotal) || ttotal != tgt_n) return -1;
    if (delta) {
        if (!base || base_n < MF_COUNT) return -1;
        if (build_slices(base, bs, &btotal) || btotal != base_n) return -1;
    }
    int changed = 0;
    Buf body = {0};
    buf_u16(&body, PKG_VERSION);
    buf_u16(&body, delta ? PKG_FLAG_DELTA : 0);
    size_t name_len = strlen(sc->name);
    buf_u16(&body, (uint16_t)name_len);
    buf_bytes(&body, sc->name, name_len);
    buf_u32(&body, scene_digest(sc));
    /* provenance so a delta can never be applied to the wrong base */
    uint32_t base_hash = 0;
    uint64_t base_tick = 0;
    if (delta) {
        base_hash = 2166136261u;
        for (int i = 0; i < base_n; i++) base_hash = fnv(base_hash, &base[i], 4);
        base_tick = ((uint64_t)base[MF_TICK_HI] << 32) | (uint64_t)base[MF_TICK_LO];
    }
    buf_u32(&body, base_hash);
    buf_u32(&body, (uint32_t)(base_tick & 0xffffffffu));
    buf_u32(&body, (uint32_t)(base_tick >> 32));
    buf_u16(&body, CH_COUNT);

    for (int ci = 0; ci < CH_COUNT; ci++) {
        Buf payload = {0};
        if (delta) {
            int n = ts[ci].count;
            int bm_bytes = (n + 7) / 8;
            unsigned char bm[ (MAX_CELLS + 7) / 8 ];
            memset(bm, 0, (size_t)bm_bytes);
            Buf vals = {0};
            int nchange = 0;
            for (int i = 0; i < n; i++) {
                uint32_t tv = tgt[ts[ci].off + i];
                int in_base = base && i < bs[ci].count;
                uint32_t bv = in_base ? base[bs[ci].off + i] : 0;
                if (in_base && bv == tv) continue;
                bm[i / 8] |= (unsigned char)(1u << (i % 8));
                buf_u32(&vals, tv);
                nchange++;
            }
            changed += nchange;
            /* payload: cell count, one changed-bit per cell, then the values */
            buf_u32(&payload, (uint32_t)n);
            buf_bytes(&payload, bm, (size_t)bm_bytes);
            buf_bytes(&payload, vals.data, vals.len);
            buf_free(&vals);
        } else {
            for (int i = 0; i < ts[ci].count; i++) buf_u32(&payload, tgt[ts[ci].off + i]);
        }
        buf_u8(&body, (uint8_t)ts[ci].id);
        buf_u8(&body, (uint8_t)(delta ? 1 : 0));
        buf_u16(&body, 0);
        buf_u32(&body, (uint32_t)ts[ci].count);
        buf_u32(&body, (uint32_t)payload.len);
        buf_u32(&body, fnv(2166136261u, payload.data, payload.len));
        buf_bytes(&body, payload.data, payload.len);
        buf_free(&payload);
    }

    buf_bytes(out, PKG_MAGIC, 4);
    buf_bytes(out, body.data, body.len);
    buf_u32(out, fnv(0x51ed270bu, out->data, out->len));
    buf_free(&body);
    if (changed_out) *changed_out = changed;
    return 0;
}

typedef struct {
    int id;
    int enc;              /* 0 full, 1 diff */
    uint32_t count;
    uint32_t len;
    const unsigned char* payload;
} ChunkIn;

static int apply_chunk_cells(const ChunkIn* in, const uint32_t* base, int base_n,
                             int base_off, int base_count, uint32_t* dst, int dst_count) {
    if (dst_count < 0 || (int)in->count != dst_count) return -1;
    if (base && base_n >= base_off + base_count) {
        int copy = base_count < dst_count ? base_count : dst_count;
        memcpy(dst, base + base_off, sizeof(uint32_t) * (size_t)copy);
    } else {
        memset(dst, 0, sizeof(uint32_t) * (size_t)dst_count);
    }
    if (in->enc == 0) {
        if (in->len != (uint32_t)dst_count * 4) return -1;
        Reader r = { in->payload, in->len, 0 };
        for (int i = 0; i < dst_count; i++) {
            if (rd_u32(&r, &dst[i])) return -1;
        }
        if (r.off != r.n) return -1;
        return 0;
    }
    if (in->enc == 1) {
        Reader r = { in->payload, in->len, 0 };
        uint32_t ncell = 0;
        if (rd_u32(&r, &ncell)) return -1;
        if ((int)ncell != dst_count) return -1;
        if (in->count != (uint32_t)dst_count) return -1;
        size_t bm_bytes = (dst_count + 7u) / 8u;
        if (r.off + bm_bytes > r.n) return -1;
        const unsigned char* bm = r.p + r.off;
        r.off += bm_bytes;
        int nchange = 0;
        for (int i = 0; i < dst_count; i++) if (bm[i / 8] & (1u << (i % 8))) nchange++;
        if (r.n != r.off + (size_t)nchange * 4) return -1;
        int k = 0;
        for (int i = 0; i < dst_count; i++) {
            if (!(bm[i / 8] & (1u << (i % 8)))) continue;
            if (rd_u32(&r, &dst[i])) return -1;
            k++;
        }
        return k == nchange ? 0 : -1;
    }
    return -1;
}

/* decode full or delta package into the canonical cell vector */
static int pkg_decode(const unsigned char* data, size_t len, const Scene* sc,
                      const uint32_t* base, int base_n,
                      uint32_t* out, int cap, int* out_n, char* err, size_t ecap) {
    if (!data || len < 4 + 2 + 2 + 2) { snprintf(err, ecap, "too_short"); return -1; }
    if (memcmp(data, PKG_MAGIC, 4) != 0) { snprintf(err, ecap, "bad_magic"); return -1; }
    if (len < 12) { snprintf(err, ecap, "too_short"); return -1; }
    uint32_t want_hash;
    Reader tail = { data + len - 4, 4, 0 };
    if (rd_u32(&tail, &want_hash)) { snprintf(err, ecap, "too_short"); return -1; }
    if (fnv(0x51ed270bu, data, len - 4) != want_hash) { snprintf(err, ecap, "bad_package_hash"); return -1; }

    Reader r = { data + 4, len - 8, 0 };   /* body, excluding magic and trailing hash */
    uint16_t ver = 0, flags = 0, name_len = 0;
    if (rd_u16(&r, &ver) || rd_u16(&r, &flags) || rd_u16(&r, &name_len)) {
        snprintf(err, ecap, "bad_header"); return -1;
    }
    if (ver != PKG_VERSION) { snprintf(err, ecap, "bad_version"); return -1; }
    if (flags & ~PKG_FLAG_DELTA) { snprintf(err, ecap, "bad_flags"); return -1; }
    const char* name = (const char*)(data + 4 + r.off);
    if (r.off + name_len > r.n) { snprintf(err, ecap, "bad_header"); return -1; }
    r.off += name_len;
    uint32_t digest = 0;
    if (rd_u32(&r, &digest)) { snprintf(err, ecap, "bad_header"); return -1; }
    if (digest != scene_digest(sc)) { snprintf(err, ecap, "scene_mismatch"); return -1; }
    if (name_len != strlen(sc->name) || memcmp(name, sc->name, name_len) != 0) {
        snprintf(err, ecap, "scene_name_mismatch"); return -1;
    }
    uint32_t base_hash = 0, base_tick_lo = 0, base_tick_hi = 0;
    if (rd_u32(&r, &base_hash) || rd_u32(&r, &base_tick_lo) || rd_u32(&r, &base_tick_hi)) {
        snprintf(err, ecap, "bad_header"); return -1;
    }
    uint16_t nchunk = 0;
    if (rd_u16(&r, &nchunk)) { snprintf(err, ecap, "bad_header"); return -1; }
    if (nchunk != CH_COUNT) { snprintf(err, ecap, "bad_chunks"); return -1; }

    ChunkIn ch[CH_COUNT];
    int seen[CH_COUNT] = {0};
    for (int i = 0; i < CH_COUNT; i++) {
        uint8_t id = 0, enc = 0;
        uint16_t res = 0;
        uint32_t count = 0, plen = 0, phash = 0;
        if (rd_u8(&r, &id) || rd_u8(&r, &enc) || rd_u16(&r, &res) ||
            rd_u32(&r, &count) || rd_u32(&r, &plen) || rd_u32(&r, &phash)) {
            snprintf(err, ecap, "bad_chunk_header"); return -1;
        }
        if (id >= CH_COUNT || seen[id]) { snprintf(err, ecap, "bad_chunk_id"); return -1; }
        if (enc > 1) { snprintf(err, ecap, "bad_chunk_enc"); return -1; }
        if (r.off + plen > r.n) { snprintf(err, ecap, "bad_chunk_len"); return -1; }
        if (fnv(2166136261u, data + 4 + r.off, plen) != phash) {
            snprintf(err, ecap, "bad_chunk_hash"); return -1;
        }
        ch[id].id = id; ch[id].enc = enc; ch[id].count = count;
        ch[id].len = plen; ch[id].payload = data + 4 + r.off;
        seen[id] = 1;
        r.off += plen;
    }
    if (r.off != r.n) { snprintf(err, ecap, "trailing_bytes"); return -1; }
    for (int i = 0; i < CH_COUNT; i++)
        if (!seen[i]) { snprintf(err, ecap, "missing_chunk"); return -1; }

    int is_delta = (flags & PKG_FLAG_DELTA) != 0;
    if (is_delta && (!base || base_n < MF_COUNT)) { snprintf(err, ecap, "delta_without_base"); return -1; }
    if (!is_delta && base_hash != 0) { snprintf(err, ecap, "unexpected_base"); return -1; }

    Slice bs[CH_COUNT];
    int btotal = 0;
    if (is_delta) {
        if (build_slices(base, bs, &btotal) || btotal != base_n) {
            snprintf(err, ecap, "bad_base"); return -1;
        }
        uint32_t h = 2166136261u;
        for (int i = 0; i < base_n; i++) h = fnv(h, &base[i], 4);
        uint64_t btick = ((uint64_t)base_tick_hi << 32) | (uint64_t)base_tick_lo;
        if (h != base_hash) { snprintf(err, ecap, "delta_base_hash_mismatch"); return -1; }
        if (btick != (((uint64_t)base[MF_TICK_HI] << 32) | (uint64_t)base[MF_TICK_LO])) {
            snprintf(err, ecap, "delta_base_tick_mismatch"); return -1;
        }
    }

    /* meta first: it carries the entity/emitter/particle counts */
    if (ch[CH_META].count != MF_COUNT) { snprintf(err, ecap, "bad_meta"); return -1; }
    int bmeta_off = is_delta ? bs[CH_META].off : 0;
    int bmeta_count = is_delta ? bs[CH_META].count : 0;
    if (apply_chunk_cells(&ch[CH_META], is_delta ? base : NULL, is_delta ? base_n : 0,
                          bmeta_off, bmeta_count, out, MF_COUNT)) {
        snprintf(err, ecap, "bad_meta_payload"); return -1;
    }

    uint32_t meta[MF_COUNT];
    memcpy(meta, out, sizeof meta);
    Slice ts[CH_COUNT];
    int ttotal = 0;
    if (build_slices(meta, ts, &ttotal)) { snprintf(err, ecap, "bad_counts"); return -1; }
    if (ttotal > cap) { snprintf(err, ecap, "state_too_large"); return -1; }

    for (int ci = CH_ENT; ci < CH_COUNT; ci++) {
        if (ch[ci].count != (uint32_t)ts[ci].count) { snprintf(err, ecap, "chunk_count_mismatch"); return -1; }
        int boff = is_delta ? bs[ci].off : 0;
        int bcnt = is_delta ? bs[ci].count : 0;
        if (apply_chunk_cells(&ch[ci], is_delta ? base : NULL, is_delta ? base_n : 0,
                              boff, bcnt, out + ts[ci].off, ts[ci].count)) {
            snprintf(err, ecap, "bad_chunk_payload"); return -1;
        }
    }
    *out_n = ttotal;
    return 0;
}

/* inflate a package into a World, then let cells_to_world check derived data */
static int pkg_restore(const unsigned char* data, size_t len, const Scene* sc,
                       const uint32_t* base, int base_n, World* w, char* err, size_t ecap) {
    uint32_t cells[MAX_CELLS];
    int n = 0;
    if (pkg_decode(data, len, sc, base, base_n, cells, MAX_CELLS, &n, err, ecap)) return -1;
    return cells_to_world(sc, cells, n, w, err, ecap);
}

static void pkg_reseal(Buf* b) {
    if (b->len < 4) return;
    uint32_t h = fnv(0x51ed270bu, b->data, b->len - 4);
    b->data[b->len - 4] = (unsigned char)(h & 0xff);
    b->data[b->len - 3] = (unsigned char)((h >> 8) & 0xff);
    b->data[b->len - 2] = (unsigned char)((h >> 16) & 0xff);
    b->data[b->len - 1] = (unsigned char)(h >> 24);
}

static int pkg_is_delta(const Buf* b) {
    if (b->len < 8) return 0;
    Reader r = { b->data + 4, 4, 0 };
    uint16_t ver = 0, flags = 0;
    if (rd_u16(&r, &ver) || rd_u16(&r, &flags)) return 0;
    return (flags & PKG_FLAG_DELTA) != 0;
}

/* walk the package bytes without trusting the encoder: count stored values */
static int pkg_count_stored_values(const unsigned char* data, size_t len) {
    if (len < 12 || memcmp(data, PKG_MAGIC, 4) != 0) return -1;
    Reader r = { data + 4, len - 8, 0 };
    uint16_t ver = 0, flags = 0, name_len = 0;
    if (rd_u16(&r, &ver) || rd_u16(&r, &flags) || rd_u16(&r, &name_len)) return -1;
    r.off += name_len;
    uint32_t ignore = 0;
    if (rd_u32(&r, &ignore) || rd_u32(&r, &ignore) || rd_u32(&r, &ignore) || rd_u32(&r, &ignore))
        return -1;
    uint16_t nchunk = 0;
    if (rd_u16(&r, &nchunk)) return -1;
    int total = 0;
    for (int c = 0; c < nchunk; c++) {
        uint8_t id = 0, enc = 0;
        uint16_t res = 0;
        uint32_t cnt = 0, plen = 0, phash = 0;
        if (rd_u8(&r, &id) || rd_u8(&r, &enc) || rd_u16(&r, &res) ||
            rd_u32(&r, &cnt) || rd_u32(&r, &plen) || rd_u32(&r, &phash)) return -1;
        if (r.off + plen > r.n) return -1;
        if (enc == 0) {
            total += (int)cnt;
            r.off += plen;
        } else if (enc == 1) {
            Reader p = { data + 4 + r.off, plen, 0 };
            uint32_t ncell = 0;
            if (rd_u32(&p, &ncell)) return -1;
            size_t bm_bytes = (ncell + 7u) / 8u;
            if (p.off + bm_bytes > p.n) return -1;
            for (size_t i = 0; i < bm_bytes; i++)
                for (int bit = 0; bit < 8; bit++)
                    if (p.p[p.off + i] & (1u << bit)) total++;
            r.off += plen;
        } else {
            return -1;
        }
    }
    return total;
}

/* --------------------------------------------------------------- helpers */

static double now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e6 + (double)ts.tv_nsec / 1e3;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* build a full checkpoint buffer for a World */
static int save_full(const World* w, const Scene* sc, Buf* out) {
    uint32_t cells[MAX_CELLS];
    int n = world_to_cells(w, sc, cells, MAX_CELLS);
    if (n < 0) return -1;
    out->data = NULL; out->len = out->cap = 0;
    return pkg_encode(sc, 0, NULL, 0, cells, n, out, NULL);
}
static int save_delta(const World* base, const World* target, const Scene* sc, Buf* out, int* changed) {
    uint32_t bc[MAX_CELLS], tc[MAX_CELLS];
    int bn = world_to_cells(base, sc, bc, MAX_CELLS);
    int tn = world_to_cells(target, sc, tc, MAX_CELLS);
    if (bn < 0 || tn < 0) return -1;
    out->data = NULL; out->len = out->cap = 0;
    return pkg_encode(sc, 1, bc, bn, tc, tn, out, changed);
}

/* count how many cells differ between two Worlds, for the "only changes" assert */
static int diff_cells(const World* a, const World* b, const Scene* sc) {
    uint32_t ca[MAX_CELLS], cb[MAX_CELLS];
    int na = world_to_cells(a, sc, ca, MAX_CELLS);
    int nb = world_to_cells(b, sc, cb, MAX_CELLS);
    int n = na < nb ? na : nb;
    int d = 0;
    for (int i = 0; i < n; i++) if (ca[i] != cb[i]) d++;
    return d;
}

/* ---------------------------------------------------------------- tests */

int main(void) {
    Scene sc = kSandbox;
    World w60, w90, w120, w150;
    world_play(&sc, &w60, 60);
    world_play(&sc, &w90, 90);
    world_play(&sc, &w120, 120);
    world_play(&sc, &w150, 150);

    Buf full90 = {0}, full120 = {0}, delta = {0}, same = {0}, delta_long = {0};
    check("save_full_90", save_full(&w90, &sc, &full90) == 0);
    check("save_full_120", save_full(&w120, &sc, &full120) == 0);

    /* ---- full checkpoint restore must equal a fresh replay, field by field */
    World r90;
    char err[96];
    check("restore_full_90", pkg_restore(full90.data, full90.len, &sc, NULL, 0, &r90, err, sizeof err) == 0);
    int bad = compare_worlds(&r90, &w90, &sc, 8);
    printf("restore_full_mismatches=%d\n", bad);
    check("restore_full_exact", bad == 0);
    check("restore_full_hash", world_hash(&r90, &sc) == world_hash(&w90, &sc));

    /* a restored world re-saves to the same bytes */
    Buf again = {0};
    check("resave_ok", save_full(&r90, &sc, &again) == 0);
    check("resave_identical", again.len == full90.len &&
          memcmp(again.data, full90.data, again.len) == 0);
    buf_free(&again);

    /* ---- continue from the checkpoint: no drift against straight replay */
    World cont = r90;
    world_advance(&sc, &cont, 30);
    int drift = compare_worlds(&cont, &w120, &sc, 8);
    printf("continue_drift_fields=%d\n", drift);
    check("continue_no_drift", drift == 0);

    /* ---- delta 90 -> 120: restore from base + changes, exact again */
    int changed = -1;
    check("save_delta_90_120", save_delta(&w90, &w120, &sc, &delta, &changed) == 0);
    printf("delta_changed_cells=%d\n", changed);
    int expected_diff = diff_cells(&w90, &w120, &sc);
    printf("delta_diff_cells_expected=%d\n", expected_diff);
    check("delta_changed_is_exact", changed == expected_diff);
    int stored = pkg_count_stored_values(delta.data, delta.len);
    printf("delta_stored_values_walked=%d\n", stored);
    check("delta_only_changes_stored", stored == expected_diff);
    check("delta_is_delta", pkg_is_delta(&delta));

    uint32_t base_cells[MAX_CELLS];
    int base_n = world_to_cells(&w90, &sc, base_cells, MAX_CELLS);
    World rd;
    check("restore_delta_90_120", base_n > 0 &&
          pkg_restore(delta.data, delta.len, &sc, base_cells, base_n, &rd, err, sizeof err) == 0);
    int dbad = compare_worlds(&rd, &w120, &sc, 8);
    printf("restore_delta_mismatches=%d\n", dbad);
    check("restore_delta_exact", dbad == 0);

    /* ---- delta of an unchanged state stores no cells */
    check("save_delta_same", save_delta(&w90, &w90, &sc, &same, &changed) == 0);
    printf("delta_same_changed_cells=%d\n", changed);
    check("delta_same_zero_cells", changed == 0);
    check("delta_same_far_smaller", same.len < full90.len);

    /* ---- a longer gap still only stores changes, and still restores exactly */
    check("save_delta_60_150", save_delta(&w60, &w150, &sc, &delta_long, &changed) == 0);
    World r150;
    int n60 = world_to_cells(&w60, &sc, base_cells, MAX_CELLS);
    check("restore_delta_60_150", n60 > 0 &&
          pkg_restore(delta_long.data, delta_long.len, &sc, base_cells, n60, &r150, err, sizeof err) == 0);
    int lbad = compare_worlds(&r150, &w150, &sc, 8);
    printf("restore_delta_long_mismatches=%d\n", lbad);
    check("restore_delta_long_exact", lbad == 0);

    /* ---- rollback then continue again from the delta-restored world */
    World cont2 = r150;
    world_advance(&sc, &cont2, 20);
    World w170;
    world_play(&sc, &w170, 170);
    int drift2 = compare_worlds(&cont2, &w170, &sc, 8);
    printf("delta_continue_drift_fields=%d\n", drift2);
    check("delta_continue_no_drift", drift2 == 0);

    /* ---- a one-tick delta stores only the handful of cells that moved */
    World w61;
    world_play(&sc, &w61, 61);
    int ch1 = -1;
    Buf delta1 = {0};
    check("save_delta_60_61", save_delta(&w60, &w61, &sc, &delta1, &ch1) == 0);
    printf("delta_one_tick_changed_cells=%d\n", ch1);
    printf("delta_one_tick_bytes=%zu\n", delta1.len);
    check("delta_one_tick_small", delta1.len < full90.len * 3 / 4);

    /* ---- size report */
    int cells = cell_count(&w90);
    printf("checkpoint_cells=%d\n", cells);
    printf("checkpoint_bytes=%zu\n", full90.len);
    printf("full120_bytes=%zu\n", full120.len);
    printf("cells_per_checkpoint=%d\n", cell_count(&w120));
    printf("bytes_per_cell=%.2f\n", (double)full90.len / (double)cells);
    printf("delta_bytes=%zu\n", delta.len);
    printf("delta_same_bytes=%zu\n", same.len);
    printf("delta_long_bytes=%zu\n", delta_long.len);
    printf("delta_saving_pct=%.1f\n", 100.0 * (1.0 - (double)delta.len / (double)full120.len));
    check("delta_smaller_than_full", delta.len < full120.len);

    /* ---- restore timing: full inflate and delta apply */
    const int REPS = 1000;
    volatile uint64_t sink = 0;
    base_n = world_to_cells(&w90, &sc, base_cells, MAX_CELLS);
    /* warm up so the first allocation is not billed to one of the paths */
    for (int i = 0; i < 50; i++) {
        World t;
        pkg_restore(full90.data, full90.len, &sc, NULL, 0, &t, err, sizeof err);
        pkg_restore(delta.data, delta.len, &sc, base_cells, base_n, &t, err, sizeof err);
    }
    double t0 = now_us();
    for (int i = 0; i < REPS; i++) {
        World t;
        if (pkg_restore(full90.data, full90.len, &sc, NULL, 0, &t, err, sizeof err)) { check("timing_full_ok", 0); break; }
        sink += t.tick;
    }
    double t_full = (now_us() - t0) / REPS;
    t0 = now_us();
    for (int i = 0; i < REPS; i++) {
        World t;
        if (pkg_restore(delta.data, delta.len, &sc, base_cells, base_n, &t, err, sizeof err)) { check("timing_delta_ok", 0); break; }
        sink += t.part[0].ttl != 0.0f;
    }
    double t_delta = (now_us() - t0) / REPS;
    printf("restore_us_full=%.1f\n", t_full);
    printf("restore_us_delta=%.1f\n", t_delta);
    printf("restore_reps=%d\n", REPS);
    printf("restore_sink=%llu\n", (unsigned long long)sink);

    /* ---- comparison catches a tampered field (bug detector works) */
    World tampered = r90;
    tampered.ent[1].hp = nextafterf(tampered.ent[1].hp, 1e9f);
    int tbad = compare_worlds(&tampered, &w90, &sc, 4);
    printf("tampered_mismatches=%d\n", tbad);
    check("tamper_detected", tbad == 1);

    /* ---- rejection paths */
    Buf badpkg = {0};
    buf_bytes(&badpkg, full90.data, full90.len);
    badpkg.data[badpkg.len - 10] ^= 0x01;
    World junk;
    check("reject_corrupt_byte",
          pkg_restore(badpkg.data, badpkg.len, &sc, NULL, 0, &junk, err, sizeof err) != 0);
    printf("reject_corrupt_reason=%s\n", err);
    buf_free(&badpkg);

    check("reject_truncated",
          pkg_restore(full90.data, full90.len - 1, &sc, NULL, 0, &junk, err, sizeof err) != 0);
    printf("reject_truncated_reason=%s\n", err);

    Buf badver = {0};
    buf_bytes(&badver, full90.data, full90.len);
    badver.data[4] = 99;             /* version low byte */
    pkg_reseal(&badver);
    check("reject_bad_version",
          pkg_restore(badver.data, badver.len, &sc, NULL, 0, &junk, err, sizeof err) != 0);
    printf("reject_bad_version_reason=%s\n", err);
    buf_free(&badver);

    /* delta without the matching base is refused */
    check("reject_delta_no_base",
          pkg_restore(delta.data, delta.len, &sc, NULL, 0, &junk, err, sizeof err) != 0);
    printf("reject_delta_no_base_reason=%s\n", err);

    /* delta against the wrong base is refused by base provenance */
    World rwrong;
    int nw = world_to_cells(&w120, &sc, base_cells, MAX_CELLS);
    check("reject_wrong_base", nw > 0 &&
          pkg_restore(delta.data, delta.len, &sc, base_cells, nw, &rwrong, err, sizeof err) != 0);
    printf("reject_wrong_base_reason=%s\n", err);

    /* a package whose derived cells disagree with their truth is refused:
       the cell vector is decoded directly, then the derived cells edited */
    uint32_t edited[MAX_CELLS];
    int en = 0;
    uint32_t b90[MAX_CELLS];
    int b90n = world_to_cells(&w90, &sc, b90, MAX_CELLS);
    check("decode_edit_prep", b90n > 0 &&
          pkg_decode(full90.data, full90.len, &sc, NULL, 0, edited, MAX_CELLS, &en, err, sizeof err) == 0);
    World fixed;
    uint32_t saved_solid = edited[MF_COUNT + EF_COUNT * 1 + EF_SOLID];
    edited[MF_COUNT + EF_COUNT * 1 + EF_SOLID] ^= 1u;
    check("reject_derived_solid",
          cells_to_world(&sc, edited, en, &fixed, err, sizeof err) != 0);
    printf("reject_derived_solid_reason=%s\n", err);
    edited[MF_COUNT + EF_COUNT * 1 + EF_SOLID] = saved_solid;
    uint32_t saved_keys = edited[MF_COUNT + EF_COUNT * 2 + EF_ANIM_KEYS];
    edited[MF_COUNT + EF_COUNT * 2 + EF_ANIM_KEYS] = 0u;
    check("reject_scene_key_count",
          cells_to_world(&sc, edited, en, &fixed, err, sizeof err) != 0);
    printf("reject_scene_key_count_reason=%s\n", err);
    edited[MF_COUNT + EF_COUNT * 2 + EF_ANIM_KEYS] = saved_keys;
    uint32_t saved_frame = edited[off_anim(&w90)];
    edited[off_anim(&w90)] = f2u(u2f(saved_frame) + 0.25f);
    check("reject_derived_anim_frame",
          cells_to_world(&sc, edited, en, &fixed, err, sizeof err) != 0);
    printf("reject_derived_anim_reason=%s\n", err);
    check("edited_back_ok", cells_to_world(&sc, b90, b90n, &fixed, err, sizeof err) == 0);

    /* package built for another scene content is rejected */
    Scene other = kSandbox;
    other.seed = 99;
    Buf otherpkg = {0};
    World otherworld;
    world_play(&other, &otherworld, 10);
    check("save_other_scene", save_full(&otherworld, &other, &otherpkg) == 0);
    check("reject_scene_mismatch",
          pkg_restore(otherpkg.data, otherpkg.len, &sc, NULL, 0, &junk, err, sizeof err) != 0);
    printf("reject_scene_reason=%s\n", err);
    buf_free(&otherpkg);

    printf("all_ok=%d\n", failures == 0);
    buf_free(&full90); buf_free(&full120); buf_free(&delta);
    buf_free(&same); buf_free(&delta_long); buf_free(&delta1);
    return failures == 0 ? 0 : 1;
}
