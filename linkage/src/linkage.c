/* linkage: level-state linkage between switches and doors.
 *
 * Nodes are either switches (external inputs) or derived nodes (doors/gates).
 * Every derived node owns exactly one rule that combines any number of input
 * nodes with ANY / ALL / AT_LEAST(n), plus per-input negation.  A switch may
 * feed any number of rules, and a rule may read any number of switches and
 * derived nodes (many-to-many).
 *
 * link_finalize() topologically validates the rule graph.  A cycle makes it
 * fail and flags every node that is cyclic or blocked behind a cycle;
 * link_tick() then refuses to run.  Evaluation is a single pass over the
 * precomputed order, so a cyclic setup can never spin in an eval loop.
 *
 * Each tick evaluates in topological order, then publishes every state change
 * since the previous tick to all subscribers: one callback per
 * (changed node, subscriber) pair, i.e. linear in the subscriber count.
 */
#include <stdio.h>
#include <string.h>

#define LINK_MAX_NODES 24
#define LINK_MAX_RULES 24
#define LINK_MAX_INPUTS 8
#define LINK_MAX_SUBS 8
#define LINK_MAX_CYCLE 24
#define LINK_MAX_EDGES (LINK_MAX_RULES * LINK_MAX_INPUTS)
#define LINK_NAME_LEN 16

typedef enum { LINK_SWITCH = 0, LINK_DOOR, LINK_GATE } LinkKind;
typedef enum { LINK_ANY = 0, LINK_ALL, LINK_AT_LEAST } LinkCombine;

typedef struct {
    int src;        /* source node id */
    int negate;     /* read !state(src) */
} LinkInput;

typedef struct {
    int target;
    LinkCombine combine;
    int threshold;  /* AT_LEAST only */
    LinkInput in[LINK_MAX_INPUTS];
    int in_count;
} LinkRule;

typedef struct {
    char name[LINK_NAME_LEN];
    LinkKind kind;
    int state;
    int rule;       /* owned rule, or -1 */
    int order;      /* topological index, or -1 */
    int in_cycle;   /* cyclic, or downstream of a cycle */
} LinkNode;

typedef void (*LinkCallback)(void* user, int node, int state, int frame);

typedef struct {
    LinkCallback cb;
    void* user;
} LinkSub;

typedef struct {
    LinkNode nodes[LINK_MAX_NODES];
    int node_count;
    LinkRule rules[LINK_MAX_RULES];
    int rule_count;
    LinkSub subs[LINK_MAX_SUBS];
    int sub_count;

    int topo[LINK_MAX_NODES];
    int topo_count;
    int cyc[LINK_MAX_CYCLE];
    int cycle_count;
    int finalized;

    int published[LINK_MAX_NODES];  /* last state sent to subscribers */
    int frame;
    long callbacks;                 /* broadcast callbacks delivered */
    long eval_steps;                /* rule inputs evaluated */
} Link;

static void link_init(Link* l) {
    memset(l, 0, sizeof(*l));
}

static int link_add_node(Link* l, const char* name, LinkKind kind) {
    if (!l || !name || l->node_count >= LINK_MAX_NODES) return -1;
    LinkNode* n = &l->nodes[l->node_count];
    memset(n, 0, sizeof(*n));
    int i = 0;
    for (; name[i] && i + 1 < LINK_NAME_LEN; i++) n->name[i] = name[i];
    n->name[i] = '\0';
    n->kind = kind;
    n->rule = -1;
    n->order = -1;
    l->finalized = 0;
    return l->node_count++;
}

/* One rule per derived target; adding a second one is a conflict. */
static int link_add_rule(Link* l, int target, LinkCombine combine, int threshold) {
    if (!l || target < 0 || target >= l->node_count) return -1;
    LinkNode* t = &l->nodes[target];
    if (t->kind == LINK_SWITCH || t->rule >= 0) return -1;
    if (combine != LINK_ANY && combine != LINK_ALL && combine != LINK_AT_LEAST) return -1;
    if (combine == LINK_AT_LEAST && threshold < 0) return -1;
    if (l->rule_count >= LINK_MAX_RULES) return -1;
    LinkRule* r = &l->rules[l->rule_count];
    memset(r, 0, sizeof(*r));
    r->target = target;
    r->combine = combine;
    r->threshold = threshold;
    t->rule = l->rule_count;
    l->finalized = 0;
    return l->rule_count++;
}

static int link_add_input(Link* l, int rule, int src, int negate) {
    if (!l || rule < 0 || rule >= l->rule_count) return -1;
    if (src < 0 || src >= l->node_count) return -1;
    LinkRule* r = &l->rules[rule];
    if (r->in_count >= LINK_MAX_INPUTS) return -1;
    for (int i = 0; i < r->in_count; i++) {
        if (r->in[i].src == src) return -1;     /* duplicate source */
    }
    r->in[r->in_count].src = src;
    r->in[r->in_count].negate = negate ? 1 : 0;
    r->in_count++;
    l->finalized = 0;
    return 0;
}

static int link_subscribe(Link* l, LinkCallback cb, void* user) {
    if (!l || !cb || l->sub_count >= LINK_MAX_SUBS) return -1;
    LinkSub* s = &l->subs[l->sub_count];
    s->cb = cb;
    s->user = user;
    return l->sub_count++;
}

/* Kahn over the rule graph.  Any node left without a topological position is
 * either part of a cycle or depends on one; all are reported. */
static int link_finalize(Link* l) {
    if (!l) return -1;
    l->finalized = 0;
    l->topo_count = 0;
    l->cycle_count = 0;
    for (int i = 0; i < l->node_count; i++) {
        l->nodes[i].order = -1;
        l->nodes[i].in_cycle = 0;
    }

    int indeg[LINK_MAX_NODES];
    int edge_from[LINK_MAX_EDGES];
    int edge_to[LINK_MAX_EDGES];
    int edge_count = 0;
    memset(indeg, 0, sizeof(indeg));
    for (int r = 0; r < l->rule_count; r++) {
        LinkRule* rule = &l->rules[r];
        for (int i = 0; i < rule->in_count; i++) {
            if (edge_count >= LINK_MAX_EDGES) return -1;
            edge_from[edge_count] = rule->in[i].src;
            edge_to[edge_count] = rule->target;
            edge_count++;
            indeg[rule->target]++;
        }
    }

    int queue[LINK_MAX_NODES];
    int qh = 0, qt = 0;
    for (int i = 0; i < l->node_count; i++) {
        if (indeg[i] == 0) queue[qt++] = i;
    }
    while (qh < qt) {
        int u = queue[qh++];
        l->nodes[u].order = l->topo_count;
        l->topo[l->topo_count++] = u;
        for (int e = 0; e < edge_count; e++) {
            if (edge_from[e] != u) continue;
            if (--indeg[edge_to[e]] == 0) queue[qt++] = edge_to[e];
        }
    }

    if (l->topo_count != l->node_count) {
        for (int i = 0; i < l->node_count; i++) {
            if (l->nodes[i].order >= 0) continue;
            l->nodes[i].in_cycle = 1;
            if (l->cycle_count < LINK_MAX_CYCLE) l->cyc[l->cycle_count] = i;
            l->cycle_count++;
        }
        return -1;
    }

    l->finalized = 1;
    return 0;
}

static int link_in_cycle(const Link* l, int node) {
    if (!l || node < 0 || node >= l->node_count) return 0;
    return l->nodes[node].in_cycle;
}

static int link_cycle_nodes(const Link* l, int* out, int max_out) {
    if (!l) return 0;
    int n = l->cycle_count < max_out ? l->cycle_count : max_out;
    for (int i = 0; i < n; i++) out[i] = l->cyc[i];
    return n;
}

static int link_set_switch(Link* l, int node, int value) {
    if (!l || node < 0 || node >= l->node_count) return -1;
    LinkNode* n = &l->nodes[node];
    if (n->kind != LINK_SWITCH) return -1;
    int prev = n->state;
    n->state = value ? 1 : 0;
    return prev;
}

static int link_state(const Link* l, int node) {
    if (!l || node < 0 || node >= l->node_count) return -1;
    return l->nodes[node].state;
}

static int link_eval_rule(Link* l, const LinkRule* r) {
    int satisfied = 0;
    for (int i = 0; i < r->in_count; i++) {
        int v = l->nodes[r->in[i].src].state;
        if (r->in[i].negate) v = !v;
        satisfied += v;
        l->eval_steps++;
    }
    switch (r->combine) {
    case LINK_ALL: return satisfied == r->in_count;
    case LINK_AT_LEAST: return satisfied >= r->threshold;
    case LINK_ANY:
    default: return satisfied > 0;
    }
}

/* One deterministic evaluation pass, then one broadcast pass.
 * Returns the number of changed nodes, or -1 if the graph is not usable. */
static int link_tick(Link* l) {
    if (!l || !l->finalized || l->cycle_count > 0) return -1;

    for (int i = 0; i < l->topo_count; i++) {
        LinkNode* n = &l->nodes[l->topo[i]];
        if (n->kind == LINK_SWITCH || n->rule < 0) continue;
        n->state = link_eval_rule(l, &l->rules[n->rule]);
    }

    l->frame++;
    int changes = 0;
    for (int i = 0; i < l->node_count; i++) {
        int state = l->nodes[i].state;
        if (state == l->published[i]) continue;
        l->published[i] = state;
        changes++;
        for (int s = 0; s < l->sub_count; s++) {
            l->subs[s].cb(l->subs[s].user, i, state, l->frame);
            l->callbacks++;
        }
    }
    return changes;
}

/* ------------------------------- self-test ------------------------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

typedef struct {
    int calls;
    int last_node;
    int last_state;
    int last_frame;
} Probe;

static void probe_cb(void* user, int node, int state, int frame) {
    Probe* p = (Probe*)user;
    p->calls++;
    p->last_node = node;
    p->last_state = state;
    p->last_frame = frame;
}

/* Scene indices */
enum { A, B, C, D1, D2, G, D3, D4, D5, SCENE_NODES };

/* Independent hand derivation of the same rules:
 *   D1 = A
 *   D2 = A && B
 *   G  = B || C
 *   D3 = !A && G
 *   D4 = at least 2 of A,B,C
 *   D5 = D1 && D4
 */
static void manual_states(int a, int b, int c, int* out) {
    for (int i = 0; i < SCENE_NODES; i++) out[i] = 0;
    int d1 = a;
    int d2 = a && b;
    int g = b || c;
    int d3 = !a && g;
    int d4 = (a + b + c) >= 2;
    int d5 = d1 && d4;
    out[A] = a; out[B] = b; out[C] = c;
    out[D1] = d1; out[D2] = d2; out[G] = g; out[D3] = d3; out[D4] = d4; out[D5] = d5;
}

static void build_scene(Link* l) {
    link_init(l);
    link_add_node(l, "A", LINK_SWITCH);
    link_add_node(l, "B", LINK_SWITCH);
    link_add_node(l, "C", LINK_SWITCH);
    link_add_node(l, "D1", LINK_DOOR);
    link_add_node(l, "D2", LINK_DOOR);
    link_add_node(l, "G", LINK_GATE);
    link_add_node(l, "D3", LINK_DOOR);
    link_add_node(l, "D4", LINK_DOOR);
    link_add_node(l, "D5", LINK_DOOR);

    int r;
    r = link_add_rule(l, D1, LINK_ANY, 0);
    link_add_input(l, r, A, 0);

    r = link_add_rule(l, D2, LINK_ALL, 0);
    link_add_input(l, r, A, 0);
    link_add_input(l, r, B, 0);

    r = link_add_rule(l, G, LINK_ANY, 0);
    link_add_input(l, r, B, 0);
    link_add_input(l, r, C, 0);

    r = link_add_rule(l, D3, LINK_ALL, 0);
    link_add_input(l, r, A, 1);
    link_add_input(l, r, G, 0);

    r = link_add_rule(l, D4, LINK_AT_LEAST, 2);
    link_add_input(l, r, A, 0);
    link_add_input(l, r, B, 0);
    link_add_input(l, r, C, 0);

    r = link_add_rule(l, D5, LINK_ALL, 0);
    link_add_input(l, r, D1, 0);
    link_add_input(l, r, D4, 0);
}

int main(void) {
    /* --- many-to-many, conditional combinations, manual equivalence --- */
    Link scene;
    build_scene(&scene);
    check("scene_finalize", link_finalize(&scene) == 0);
    check("scene_rule_count", scene.rule_count == 6);
    check("scene_topology", scene.nodes[D1].order < scene.nodes[D5].order &&
                            scene.nodes[D4].order < scene.nodes[D5].order &&
                            scene.nodes[G].order < scene.nodes[D3].order);

    Probe probe;
    memset(&probe, 0, sizeof(probe));
    check("scene_subscribe", link_subscribe(&scene, probe_cb, &probe) == 0);

    int expected[SCENE_NODES];
    int prev[SCENE_NODES];
    int mask_ok[8];
    memset(prev, 0, sizeof(prev));
    int total_expected_changes = 0;
    int counts_ok = 1;
    for (int mask = 0; mask < 8; mask++) {
        int a = (mask >> 0) & 1, b = (mask >> 1) & 1, c = (mask >> 2) & 1;
        link_set_switch(&scene, A, a);
        link_set_switch(&scene, B, b);
        link_set_switch(&scene, C, c);
        int changed = link_tick(&scene);

        manual_states(a, b, c, expected);
        int want_changes = 0;
        mask_ok[mask] = 1;
        for (int i = 0; i < SCENE_NODES; i++) {
            if (link_state(&scene, i) != expected[i]) mask_ok[mask] = 0;
            if (expected[i] != prev[i]) want_changes++;
            prev[i] = expected[i];
        }
        if (changed != want_changes) counts_ok = 0;
        total_expected_changes += want_changes;
    }
    char name[48];
    for (int mask = 0; mask < 8; mask++) {
        snprintf(name, sizeof(name), "manual_match_mask_%d", mask);
        check(name, mask_ok[mask]);
    }
    check("change_counts_match", counts_ok);
    check("callbacks_linear_single_sub", scene.callbacks == total_expected_changes);
    check("probe_same_count", probe.calls == (int)scene.callbacks);
    check("eval_steps_linear", scene.eval_steps == 8L * 12L);  /* 12 inputs obey */
    check("repeat_tick_idle", link_tick(&scene) == 0);
    check("idle_no_broadcast", probe.calls == (int)scene.callbacks);

    /* --- per-frame batching: several writes collapse into one diff --- */
    Link batch;
    link_init(&batch);
    link_add_node(&batch, "S", LINK_SWITCH);
    link_add_node(&batch, "D", LINK_DOOR);
    int r = link_add_rule(&batch, 1, LINK_ANY, 0);
    link_add_input(&batch, r, 0, 0);
    check("batch_finalize", link_finalize(&batch) == 0);
    Probe bp;
    memset(&bp, 0, sizeof(bp));
    link_subscribe(&batch, probe_cb, &bp);
    check("batch_first_idle", link_tick(&batch) == 0);
    link_set_switch(&batch, 0, 1);
    link_set_switch(&batch, 0, 0);
    check("batch_collapsed", link_tick(&batch) == 0 && bp.calls == 0);
    link_set_switch(&batch, 0, 1);
    check("batch_two_changes", link_tick(&batch) == 2 && bp.calls == 2);
    link_set_switch(&batch, 0, 1);
    check("batch_no_duplicate", link_tick(&batch) == 0 && bp.calls == 2);
    link_set_switch(&batch, 0, 1);
    link_set_switch(&batch, 0, 0);
    link_set_switch(&batch, 0, 1);
    check("batch_return_to_published", link_tick(&batch) == 0 && bp.calls == 2);
    link_set_switch(&batch, 0, 0);
    check("batch_off_frame", link_tick(&batch) == 2 && bp.calls == 4);
    check("batch_frame_counted", bp.last_frame == batch.frame);

    /* --- broadcast cost scales only with subscribers --- */
    int scaling_ok = 1;
    int eval_ok = 1;
    for (int s = 1; s <= LINK_MAX_SUBS; s++) {
        Link l;
        build_scene(&l);
        if (link_finalize(&l) != 0) scaling_ok = 0;
        Probe probes[LINK_MAX_SUBS];
        memset(probes, 0, sizeof(probes));
        for (int i = 0; i < s; i++) {
            if (link_subscribe(&l, probe_cb, &probes[i]) < 0) scaling_ok = 0;
        }
        link_set_switch(&l, A, 1);
        long before = l.callbacks;
        int changed = link_tick(&l);
        /* A and D1 flip; every subscriber sees each event exactly once. */
        if (changed != 2) scaling_ok = 0;
        if (l.callbacks - before != (long)changed * s) scaling_ok = 0;
        for (int i = 0; i < s; i++) if (probes[i].calls != 2) scaling_ok = 0;
        if (l.eval_steps != 12) eval_ok = 0;   /* broadcast does not add eval work */
        if (link_tick(&l) != 0 || l.callbacks != before + (long)changed * s) scaling_ok = 0;
    }
    check("broadcast_linear_in_subscribers", scaling_ok);
    check("eval_independent_of_subscribers", eval_ok);

    /* --- cycle detection: reject instead of looping --- */
    Link cycle;
    link_init(&cycle);
    int x = link_add_node(&cycle, "X", LINK_DOOR);
    int y = link_add_node(&cycle, "Y", LINK_DOOR);
    int w = link_add_node(&cycle, "W", LINK_DOOR);
    r = link_add_rule(&cycle, x, LINK_ANY, 0);
    link_add_input(&cycle, r, y, 1);            /* X = !Y */
    r = link_add_rule(&cycle, y, LINK_ANY, 0);
    link_add_input(&cycle, r, x, 1);            /* Y = !X */
    r = link_add_rule(&cycle, w, LINK_ANY, 0);
    link_add_input(&cycle, r, x, 0);            /* W depends on the cycle */
    Probe cp;
    memset(&cp, 0, sizeof(cp));
    link_subscribe(&cycle, probe_cb, &cp);
    check("cycle_rejected", link_finalize(&cycle) == -1);
    int cyc[LINK_MAX_CYCLE];
    int cyc_n = link_cycle_nodes(&cycle, cyc, LINK_MAX_CYCLE);
    check("cycle_nodes_reported", cyc_n >= 3);
    check("cycle_flags_x", link_in_cycle(&cycle, x));
    check("cycle_flags_y", link_in_cycle(&cycle, y));
    check("cycle_flags_downstream", link_in_cycle(&cycle, w));
    check("cycle_tick_refused", link_tick(&cycle) == -1);
    check("cycle_no_broadcast", cp.calls == 0);

    Link self;
    link_init(&self);
    int z = link_add_node(&self, "Z", LINK_GATE);
    r = link_add_rule(&self, z, LINK_ANY, 0);
    link_add_input(&self, r, z, 1);             /* Z = !Z */
    check("self_cycle_rejected", link_finalize(&self) == -1);
    check("self_cycle_flag", link_in_cycle(&self, z));
    check("self_cycle_tick_refused", link_tick(&self) == -1);

    /* --- configuration errors and re-finalize --- */
    Link bad;
    link_init(&bad);
    int s0 = link_add_node(&bad, "S0", LINK_SWITCH);
    int d0 = link_add_node(&bad, "D0", LINK_DOOR);
    int g0 = link_add_node(&bad, "G0", LINK_GATE);
    check("reject_rule_on_switch", link_add_rule(&bad, s0, LINK_ANY, 0) == -1);
    r = link_add_rule(&bad, d0, LINK_ANY, 0);
    check("reject_duplicate_target_rule", link_add_rule(&bad, d0, LINK_ALL, 0) == -1);
    check("reject_input_out_of_range", link_add_input(&bad, r, 99, 0) == -1);
    check("accept_input", link_add_input(&bad, r, s0, 0) == 0);
    check("reject_duplicate_input", link_add_input(&bad, r, s0, 1) == -1);
    check("reject_negative_threshold", link_add_rule(&bad, g0, LINK_AT_LEAST, -1) == -1);
    check("bad_finalize", link_finalize(&bad) == 0);
    check("bad_first_tick", link_tick(&bad) == 0);
    r = link_add_rule(&bad, g0, LINK_ANY, 0);
    link_add_input(&bad, r, d0, 0);
    check("mutation_invalidates_finalize", link_tick(&bad) == -1);
    check("refinalize", link_finalize(&bad) == 0);
    link_set_switch(&bad, s0, 1);
    check("refinalize_ticks", link_tick(&bad) == 3);  /* S0, D0, G0 */

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
