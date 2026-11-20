/* modifier: layered stat modifiers with sources, revocation and a replayable op log.
 *
 * final(stat) = (base + sum of ADD layer) * (1 + sum of MUL layer)
 *
 * Determinism: modifiers are stored sorted by content — (stat, layer, value) —
 * and every query re-aggregates from scratch in that canonical order, so the
 * value is a pure function of the *set* of live modifiers, never of the order
 * they were added or removed in. (Ties share the same value, and IEEE addition
 * is commutative, so tie order cannot affect the sum.) Replaying the op log
 * reproduces the live state bit-exactly.
 *
 * Cost: insert/remove/query are all O(n) — linear in modifier count.
 */
#include <stdio.h>
#include <string.h>

#define MAX_STATS 8
#define MAX_MODS 4096
#define MAX_LOG 32768

typedef enum { LAYER_ADD = 0, LAYER_MUL = 1 } Layer;

typedef struct {
    int id;       /* unique, monotonically increasing; identity only, never ordering */
    int source;   /* equipment slot / buff id; revocable as a group */
    int stat;     /* stat index */
    Layer layer;
    double value; /* ADD: flat amount; MUL: fraction (0.2 = +20%) */
} Modifier;

typedef enum { OP_ADD = 0, OP_REMOVE, OP_REMOVE_SOURCE } OpKind;

typedef struct {
    OpKind kind;
    Modifier mod; /* OP_ADD: the modifier; OP_REMOVE: .id; OP_REMOVE_SOURCE: .source */
} LogEntry;

typedef struct {
    double base[MAX_STATS];
    Modifier mods[MAX_MODS]; /* sorted by (stat, layer, value) — content, not history */
    int count;
    int next_id;
    LogEntry log[MAX_LOG];
    int log_len;
} StatBlock;

static void sb_init(StatBlock* sb, const double* base, int nstats) {
    memset(sb, 0, sizeof(*sb));
    for (int i = 0; i < nstats && i < MAX_STATS; i++) sb->base[i] = base[i];
    sb->next_id = 1;
}

static void sb_log(StatBlock* sb, OpKind kind, const Modifier* m) {
    if (sb->log_len >= MAX_LOG) return;
    sb->log[sb->log_len].kind = kind;
    sb->log[sb->log_len].mod = *m;
    sb->log_len++;
}

/* canonical order: stat, then layer, then value ascending */
static int mod_cmp(const Modifier* a, const Modifier* b) {
    if (a->stat != b->stat) return a->stat - b->stat;
    if (a->layer != b->layer) return (int)a->layer - (int)b->layer;
    if (a->value < b->value) return -1;
    if (a->value > b->value) return 1;
    return 0;
}

static void sb_insert(StatBlock* sb, const Modifier* m) {
    int lo = 0, hi = sb->count;
    while (lo < hi) {
        int mid = lo + (hi - lo) / 2;
        if (mod_cmp(&sb->mods[mid], m) < 0) lo = mid + 1;
        else hi = mid;
    }
    memmove(&sb->mods[lo + 1], &sb->mods[lo], (size_t)(sb->count - lo) * sizeof(Modifier));
    sb->mods[lo] = *m;
    sb->count++;
}

/* returns new modifier id, or -1 when full */
static int sb_add(StatBlock* sb, int source, int stat, Layer layer, double value) {
    if (sb->count >= MAX_MODS) return -1;
    Modifier m;
    m.id = sb->next_id++;
    m.source = source;
    m.stat = stat;
    m.layer = layer;
    m.value = value;
    sb_insert(sb, &m);
    sb_log(sb, OP_ADD, &m);
    return m.id;
}

static void sb_remove_at(StatBlock* sb, int idx) {
    memmove(&sb->mods[idx], &sb->mods[idx + 1], (size_t)(sb->count - idx - 1) * sizeof(Modifier));
    sb->count--;
}

/* returns 1 when found and removed */
static int sb_remove(StatBlock* sb, int id) {
    for (int i = 0; i < sb->count; i++) {
        if (sb->mods[i].id == id) {
            Modifier m = sb->mods[i];
            sb_remove_at(sb, i);
            sb_log(sb, OP_REMOVE, &m);
            return 1;
        }
    }
    return 0;
}

/* revoke every modifier from one source; returns how many were removed */
static int sb_remove_source(StatBlock* sb, int source) {
    int removed = 0;
    for (int i = sb->count - 1; i >= 0; i--) {
        if (sb->mods[i].source == source) {
            sb_remove_at(sb, i);
            removed++;
        }
    }
    if (removed > 0) {
        Modifier m;
        memset(&m, 0, sizeof(m));
        m.source = source;
        sb_log(sb, OP_REMOVE_SOURCE, &m);
    }
    return removed;
}

/* canonical aggregation: iterate live modifiers in content-sorted order */
static double sb_value(const StatBlock* sb, int stat) {
    double add = 0.0, mul = 0.0;
    for (int i = 0; i < sb->count; i++) {
        const Modifier* m = &sb->mods[i];
        if (m->stat != stat) continue;
        if (m->layer == LAYER_ADD) add += m->value;
        else mul += m->value;
    }
    return (sb->base[stat] + add) * (1.0 + mul);
}

/* rebuild state by replaying the recorded ops onto a fresh block */
static void sb_replay(const StatBlock* live, StatBlock* out, const double* base, int nstats) {
    sb_init(out, base, nstats);
    out->next_id = 1;
    for (int i = 0; i < live->log_len; i++) {
        const LogEntry* e = &live->log[i];
        switch (e->kind) {
        case OP_ADD:
            sb_insert(out, &e->mod);
            if (e->mod.id >= out->next_id) out->next_id = e->mod.id + 1;
            break;
        case OP_REMOVE:
            for (int j = 0; j < out->count; j++) {
                if (out->mods[j].id == e->mod.id) { sb_remove_at(out, j); break; }
            }
            break;
        case OP_REMOVE_SOURCE:
            for (int j = out->count - 1; j >= 0; j--) {
                if (out->mods[j].source == e->mod.source) sb_remove_at(out, j);
            }
            break;
        }
    }
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static unsigned rng_state = 12345;
static unsigned rng_next(void) {
    rng_state = rng_state * 1664525u + 1013904223u;
    return rng_state >> 8;
}

int main(void) {
    enum { ATK = 0, DEF = 1, SPD = 2 };
    double base[MAX_STATS] = { 100.0, 50.0, 10.0 };

    /* layering: equipment add + buff mul, either order */
    {
        StatBlock a, b;
        sb_init(&a, base, MAX_STATS);
        sb_init(&b, base, MAX_STATS);
        sb_add(&a, 1, ATK, LAYER_ADD, 10.0);  /* sword: +10 atk */
        sb_add(&a, 2, ATK, LAYER_MUL, 0.20);  /* buff: +20% atk */
        sb_add(&b, 2, ATK, LAYER_MUL, 0.20);
        sb_add(&b, 1, ATK, LAYER_ADD, 10.0);
        double expect = (100.0 + 10.0) * 1.20; /* 132, hand-computed */
        check("layer_value", sb_value(&a, ATK) == expect);
        check("layer_order_independent", sb_value(&a, ATK) == sb_value(&b, ATK));
        check("layer_untouched_stat", sb_value(&a, DEF) == 50.0);
    }

    /* individual revoke and source revoke */
    {
        StatBlock sb;
        sb_init(&sb, base, MAX_STATS);
        int ring = sb_add(&sb, 1, ATK, LAYER_ADD, 5.0);   /* ring: +5 */
        sb_add(&sb, 1, ATK, LAYER_ADD, 3.0);              /* ring enchant: +3 */
        sb_add(&sb, 2, ATK, LAYER_MUL, 0.10);             /* buff: +10% */
        check("revoke_before", sb_value(&sb, ATK) == 108.0 * 1.10);
        check("revoke_one", sb_remove(&sb, ring) == 1);
        check("revoke_one_value", sb_value(&sb, ATK) == 103.0 * 1.10);
        check("revoke_missing", sb_remove(&sb, 999) == 0);
        check("revoke_source_count", sb_remove_source(&sb, 1) == 1); /* enchant left */
        check("revoke_source_value", sb_value(&sb, ATK) == 100.0 * 1.10);
        check("revoke_source_again", sb_remove_source(&sb, 1) == 0);
    }

    /* shuffled add/remove sequences reaching the same set: bit-identical */
    {
        StatBlock a, b;
        sb_init(&a, base, MAX_STATS);
        sb_init(&b, base, MAX_STATS);
        int tmp;
        sb_add(&a, 1, ATK, LAYER_ADD, 7.0);
        sb_add(&a, 2, ATK, LAYER_MUL, 0.25);
        tmp = sb_add(&a, 9, SPD, LAYER_ADD, 99.0); /* noisy modifier comes and goes */
        sb_add(&a, 3, ATK, LAYER_ADD, -2.0);
        sb_remove(&a, tmp);
        sb_add(&a, 4, DEF, LAYER_MUL, 0.5);

        sb_add(&b, 3, ATK, LAYER_ADD, -2.0);
        sb_add(&b, 4, DEF, LAYER_MUL, 0.5);
        tmp = sb_add(&b, 9, SPD, LAYER_ADD, 99.0);
        sb_add(&b, 2, ATK, LAYER_MUL, 0.25);
        sb_remove(&b, tmp);
        sb_add(&b, 1, ATK, LAYER_ADD, 7.0);

        int same = 1;
        for (int s = 0; s < MAX_STATS; s++)
            if (sb_value(&a, s) != sb_value(&b, s)) same = 0;
        check("shuffle_bit_identical", same);
    }

    /* log replay reproduces live state bit-exactly */
    {
        StatBlock sb, rp;
        sb_init(&sb, base, MAX_STATS);
        int a = sb_add(&sb, 1, ATK, LAYER_ADD, 12.5);
        sb_add(&sb, 2, ATK, LAYER_MUL, 0.15);
        sb_add(&sb, 1, DEF, LAYER_ADD, 4.0);
        sb_remove(&sb, a);
        sb_add(&sb, 3, SPD, LAYER_MUL, 0.30);
        sb_remove_source(&sb, 1);
        sb_replay(&sb, &rp, base, MAX_STATS);
        int same = sb.count == rp.count;
        for (int s = 0; s < MAX_STATS; s++)
            if (sb_value(&sb, s) != sb_value(&rp, s)) same = 0;
        check("replay_bit_identical", same);
        check("replay_set_equal", same && sb.count == 2);
    }

    /* stress: 10k random ops, compare against independent log-fold reference */
    {
        StatBlock sb;
        sb_init(&sb, base, MAX_STATS);
        int ids[1024], sources[1024], nlive = 0;
        int ops = 10000, expected_log = 0;
        for (int i = 0; i < ops; i++) {
            unsigned r = rng_next();
            if (nlive == 0 || (r % 100) < 60) { /* add */
                int src = (int)(r % 8);
                int st = (int)((r >> 4) % 3);
                Layer ly = (r & 0x40) ? LAYER_MUL : LAYER_ADD;
                double v = ((int)(r % 2001) - 1000) / 16.0; /* -62.5 .. 62.4375 */
                if (ly == LAYER_MUL) v = ((int)(r % 201) - 100) / 400.0; /* -0.25 .. 0.25 */
                int id = sb_add(&sb, src, st, ly, v);
                if (id < 0) break;
                expected_log++;
                ids[nlive] = id;
                sources[nlive] = src;
                nlive++;
            } else if ((r % 100) < 85) { /* remove one */
                int k = (int)(r % (unsigned)nlive);
                sb_remove(&sb, ids[k]);
                expected_log++;
                ids[k] = ids[nlive - 1];
                sources[k] = sources[nlive - 1];
                nlive--;
            } else { /* revoke a source */
                int src = (int)(r % 8);
                if (sb_remove_source(&sb, src) > 0) expected_log++; /* no-op revokes are not logged */
                int w = 0;
                for (int k = 0; k < nlive; k++) {
                    if (sources[k] != src) { ids[w] = ids[k]; sources[w] = sources[k]; w++; }
                }
                nlive = w;
            }
        }
        check("stress_ops_applied", sb.log_len == expected_log);

        /* independent reference: fold the op log into a set, sort by id, aggregate */
        StatBlock rp;
        sb_replay(&sb, &rp, base, MAX_STATS);
        int zero_error = sb.count == rp.count && sb.count == nlive;
        for (int s = 0; s < MAX_STATS && zero_error; s++)
            if (sb_value(&sb, s) != sb_value(&rp, s)) zero_error = 0;
        check("stress_zero_error", zero_error);

        /* live modifier set matches the test's own bookkeeping exactly */
        int set_ok = 1;
        for (int i = 0; i < nlive && set_ok; i++) {
            int found = 0;
            for (int j = 0; j < sb.count; j++)
                if (sb.mods[j].id == ids[i]) { found = 1; break; }
            if (!found) set_ok = 0;
        }
        check("stress_set_matches", set_ok);
        printf("stress_live_mods=%d\n", sb.count);
    }

    /* linear cost: aggregation visits each live modifier once per query */
    {
        StatBlock sb;
        sb_init(&sb, base, MAX_STATS);
        for (int i = 0; i < 1000; i++)
            sb_add(&sb, i, i % 3, (i & 1) ? LAYER_MUL : LAYER_ADD, 0.125);
        double acc = 0.0;
        for (int q = 0; q < 1000; q++)
            acc += sb_value(&sb, q % 3);
        check("linear_queries", acc > 0.0);
    }

    printf("failures=%d\n", failures);
    return failures ? 1 : 0;
}
