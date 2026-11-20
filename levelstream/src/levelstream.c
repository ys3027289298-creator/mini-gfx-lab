/* level-stream: dependency-aware background streaming for level entry.
 *
 * The level manifest is a DAG. Resolution maps dependency names to indices,
 * rejects unknown names and cycles, and computes the transitive closure of
 * the level roots: only that closure gates playability. A fixed worker pool
 * streams assets in bottom-level order (how much load time an asset unlocks
 * downstream), so the critical path is worked on first; an asset becomes
 * eligible only once all of its dependencies are resident. The gate stays
 * blocked until the whole closure is resident and names whatever is still
 * pending (or missing).
 *
 * The playable digest is assembled in topological order, so it matches an
 * eager full preload even though assets finish in a different order. Assets
 * outside the closure (ambient art/audio) load only after the gate opens and
 * can never change the playable digest.
 *
 * Memory model: a resident asset holds size_kb. An in-flight asset holds a
 * staging buffer proportional to its read progress plus a fixed per-worker
 * decode context, so peak resident bytes rise with the number of concurrent
 * workers. peak_entry_kb is the peak up to the moment the gate opens.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <limits.h>

#define MAX_ASSETS 32
#define MAX_DEPS 6
#define MAX_WORKERS 8
#define NAME_LEN 24
#define FRAME_MS 16
#define STREAM_OVERHEAD_KB 512
#define MAX_FRAMES 100000

enum { AST_PENDING = 0, AST_LOADING = 1, AST_READY = 2 };

typedef struct {
    char name[NAME_LEN];
    int size_kb;
    int load_ms;
    int priority;                   /* configured priority, tie-break only */
    int bottom_level;               /* load_ms of the longest chain it gates */
    int root;                       /* level root: seeds the required closure */
    int dep_count;
    char dep_name[MAX_DEPS][NAME_LEN];
    int dep_idx[MAX_DEPS];          /* filled by resolve(), -1 when unknown */
} Asset;

typedef struct {
    Asset a[MAX_ASSETS];
    int count;
} Manifest;

typedef struct {
    int ok;
    int missing;                    /* unknown dependency references */
    int cycle;
    char error[512];
    char missing_names[256];
    char cycle_names[256];
    uint64_t required;              /* bitmask, transitive closure of roots */
    int topo[MAX_ASSETS];           /* dependencies before dependents */
    int topo_len;
} Resolve;

typedef struct {
    int state[MAX_ASSETS];
    int remaining_ms[MAX_ASSETS];
    long long start_ms[MAX_ASSETS];
    long long finish_ms[MAX_ASSETS];
    int order[MAX_ASSETS];          /* completion order */
    int order_len;
    int workers;
    int frames;
    long long gate_ms;              /* frame end at which the closure is ready */
    int gate_frame;
    long long last_required_ready_ms;
    int ambient_ready_at_gate;
    int resident_kb;                /* fully loaded */
    int peak_kb;                    /* over the whole run */
    int peak_entry_kb;              /* up to the moment the gate opens */
    long long total_ms;
} Stream;

typedef struct {
    int blocked;
    int missing;
    int pending;
    char names[512];
} Gate;

static int failures = 0;

static void compute_bottom_levels(Manifest* m, const Resolve* r);

static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void names_add(char* buf, size_t cap, const char* name) {
    size_t n = strlen(buf);
    if (n + 1 >= cap) return;
    if (n > 0) buf[n++] = ',';
    size_t room = cap - 1 - n;
    size_t len = strlen(name);
    if (len > room) len = room;
    memcpy(buf + n, name, len);
    buf[n + len] = '\0';
}

static void manifest_init(Manifest* m) {
    memset(m, 0, sizeof(*m));
}

static int m_add(Manifest* m, const char* name, int size_kb, int load_ms,
                 int priority, int root) {
    if (m->count >= MAX_ASSETS) return -1;
    Asset* a = &m->a[m->count];
    snprintf(a->name, sizeof(a->name), "%s", name);
    a->size_kb = size_kb > 0 ? size_kb : 1;
    a->load_ms = load_ms > 0 ? load_ms : 1;
    a->priority = priority;
    a->root = root;
    a->dep_count = 0;
    for (int d = 0; d < MAX_DEPS; d++) a->dep_idx[d] = -1;
    return m->count++;
}

static void m_dep(Manifest* m, int idx, const char* dep_name) {
    if (idx < 0 || idx >= m->count) return;
    Asset* a = &m->a[idx];
    if (a->dep_count >= MAX_DEPS) return;
    snprintf(a->dep_name[a->dep_count], NAME_LEN, "%s", dep_name);
    a->dep_count++;
}

static int m_find(const Manifest* m, const char* name) {
    for (int i = 0; i < m->count; i++)
        if (strcmp(m->a[i].name, name) == 0) return i;
    return -1;
}

static int is_required(const Resolve* r, int i) {
    return (int)((r->required >> i) & 1u);
}

/* DFS cycle hunt; records the first back-edge path by name */
static int cycle_dfs(Manifest* m, int u, int* color, int* parent, Resolve* r) {
    color[u] = 1;
    const Asset* a = &m->a[u];
    for (int d = 0; d < a->dep_count; d++) {
        int v = a->dep_idx[d];
        if (v < 0) continue;
        if (color[v] == 1) {
            int path[MAX_ASSETS], n = 0, x = u;
            while (x != -1 && x != v && n < MAX_ASSETS) {
                path[n++] = x;
                x = parent[x];
            }
            names_add(r->cycle_names, sizeof(r->cycle_names), m->a[v].name);
            for (int k = n - 1; k >= 0; k--)
                names_add(r->cycle_names, sizeof(r->cycle_names), m->a[path[k]].name);
            r->cycle = 1;
            return 1;
        }
        if (color[v] == 0) {
            parent[v] = u;
            if (cycle_dfs(m, v, color, parent, r)) return 1;
        }
    }
    color[u] = 2;
    return 0;
}

static void resolve(Manifest* m, Resolve* r) {    memset(r, 0, sizeof(*r));
    r->required = 0;

    /* dependency names -> indices; unknown names are the blocking condition */
    for (int i = 0; i < m->count; i++) {
        Asset* a = &m->a[i];
        for (int d = 0; d < a->dep_count; d++) {
            int v = m_find(m, a->dep_name[d]);
            a->dep_idx[d] = v;
            if (v < 0) {
                r->missing++;
                names_add(r->missing_names, sizeof(r->missing_names), a->dep_name[d]);
            }
        }
    }

    int color[MAX_ASSETS] = {0}, parent[MAX_ASSETS];
    for (int i = 0; i < MAX_ASSETS; i++) parent[i] = -1;
    for (int i = 0; i < m->count && !r->cycle; i++)
        if (color[i] == 0) cycle_dfs(m, i, color, parent, r);

    /* transitive closure from level roots */
    int roots = 0;
    int queue[MAX_ASSETS], qh = 0, qt = 0;
    for (int i = 0; i < m->count; i++) {
        if (!m->a[i].root) continue;
        roots++;
        r->required |= (uint64_t)1u << i;
        queue[qt++] = i;
    }
    while (qh < qt) {
        int u = queue[qh++];
        const Asset* a = &m->a[u];
        for (int d = 0; d < a->dep_count; d++) {
            int v = a->dep_idx[d];
            if (v < 0) continue;
            if (!((r->required >> v) & 1u)) {
                r->required |= (uint64_t)1u << v;
                queue[qt++] = v;
            }
        }
    }

    /* deterministic topological order over the required set */
    int indeg[MAX_ASSETS] = {0}, done[MAX_ASSETS] = {0};
    int req_count = 0;
    for (int i = 0; i < m->count; i++) {
        if (!is_required(r, i)) continue;
        req_count++;
        const Asset* a = &m->a[i];
        for (int d = 0; d < a->dep_count; d++) {
            int v = a->dep_idx[d];
            if (v >= 0 && is_required(r, v)) indeg[i]++;
        }
    }
    r->topo_len = 0;
    for (int k = 0; k < m->count; k++) {
        int pick = -1;
        for (int i = 0; i < m->count; i++)
            if (is_required(r, i) && !done[i] && indeg[i] == 0) { pick = i; break; }
        if (pick < 0) break;
        done[pick] = 1;
        r->topo[r->topo_len++] = pick;
        for (int j = 0; j < m->count; j++) {
            if (!is_required(r, j) || done[j]) continue;
            const Asset* b = &m->a[j];
            for (int d = 0; d < b->dep_count; d++)
                if (b->dep_idx[d] == pick) indeg[j]--;
        }
    }

    r->ok = (r->missing == 0) && (r->cycle == 0) && (roots > 0) &&
            (r->topo_len == req_count);
    if (r->missing) snprintf(r->error, sizeof(r->error), "missing dependency: %s", r->missing_names);
    else if (r->cycle) snprintf(r->error, sizeof(r->error), "dependency cycle: %s", r->cycle_names);
    else if (roots == 0) snprintf(r->error, sizeof(r->error), "no level root");
    else if (r->topo_len != req_count) snprintf(r->error, sizeof(r->error), "unresolved dependency graph");
    else snprintf(r->error, sizeof(r->error), "ok");

    if (r->ok) compute_bottom_levels(m, r);
}

static int required_count(const Resolve* r, int count) {
    int n = 0;
    for (int i = 0; i < count; i++) if (is_required(r, i)) n++;
    return n;
}

static int critical_path_ms(const Manifest* m, const Resolve* r) {
    int cp[MAX_ASSETS] = {0}, best = 0;
    for (int k = 0; k < r->topo_len; k++) {
        int i = r->topo[k], dep_max = 0;
        const Asset* a = &m->a[i];
        for (int d = 0; d < a->dep_count; d++) {
            int v = a->dep_idx[d];
            if (v >= 0 && cp[v] > dep_max) dep_max = cp[v];
        }
        cp[i] = dep_max + a->load_ms;
        if (cp[i] > best) best = cp[i];
    }
    return best;
}

/* bottom level: load_ms of the longest chain from an asset up to a root.
   Assets with a larger bottom level sit on the critical path, so scheduling
   them first pulls the gate as close to the critical path as possible. */
static void compute_bottom_levels(Manifest* m, const Resolve* r) {
    for (int i = 0; i < m->count; i++) m->a[i].bottom_level = m->a[i].load_ms;
    for (int k = r->topo_len - 1; k >= 0; k--) {
        int i = r->topo[k];
        int chain = m->a[i].load_ms;
        for (int j = 0; j < m->count; j++) {
            if (!is_required(r, j)) continue;
            const Asset* b = &m->a[j];
            for (int d = 0; d < b->dep_count; d++) {
                if (b->dep_idx[d] == i && m->a[j].bottom_level > chain)
                    chain = m->a[j].bottom_level;
            }
        }
        m->a[i].bottom_level = chain;
    }
}

static void stream_init(Stream* s, int workers) {
    memset(s, 0, sizeof(*s));
    for (int i = 0; i < MAX_ASSETS; i++) {
        s->state[i] = AST_PENDING;
        s->start_ms[i] = -1;
        s->finish_ms[i] = -1;
    }
    s->workers = workers < 1 ? 1 : (workers > MAX_WORKERS ? MAX_WORKERS : workers);
    s->gate_ms = -1;
}

static int required_ready(const Stream* s, const Manifest* m, const Resolve* r) {
    for (int i = 0; i < m->count; i++)
        if (is_required(r, i) && s->state[i] != AST_READY) return 0;
    return 1;
}

static int all_ready(const Stream* s, const Manifest* m) {
    for (int i = 0; i < m->count; i++)
        if (s->state[i] != AST_READY) return 0;
    return 1;
}

static void sample_mem(Stream* s, const Manifest* m) {
    int stage = 0, in_flight = 0;
    for (int i = 0; i < m->count; i++) {
        if (s->state[i] != AST_LOADING) continue;
        const Asset* a = &m->a[i];
        int done_ms = a->load_ms - s->remaining_ms[i];
        stage += (int)((long long)a->size_kb * done_ms / a->load_ms);
        in_flight++;
    }
    int mem = s->resident_kb + stage + in_flight * STREAM_OVERHEAD_KB;
    if (mem > s->peak_kb) s->peak_kb = mem;
    if (s->gate_ms < 0 && mem > s->peak_entry_kb) s->peak_entry_kb = mem;
}

/* eligible = pending, in the closure before the gate, deps all resident.
   Bottom level dominates, configured priority breaks ties, then index. */
static int pick_asset(const Manifest* m, const Resolve* r, const Stream* s, int gate_open) {
    int best = -1, best_level = INT_MIN, best_prio = INT_MIN;
    for (int i = 0; i < m->count; i++) {
        if (s->state[i] != AST_PENDING) continue;
        if (!gate_open && !is_required(r, i)) continue;
        const Asset* a = &m->a[i];
        int deps_ok = 1;
        for (int d = 0; d < a->dep_count; d++) {
            int v = a->dep_idx[d];
            if (v < 0 || s->state[v] != AST_READY) { deps_ok = 0; break; }
        }
        if (!deps_ok) continue;
        if (a->bottom_level > best_level ||
            (a->bottom_level == best_level && a->priority > best_prio)) {
            best_level = a->bottom_level;
            best_prio = a->priority;
            best = i;
        }
    }
    return best;
}

static void stream_step(Stream* s, const Manifest* m, const Resolve* r) {
    long long now = (long long)s->frames * FRAME_MS;
    int gate_open = required_ready(s, m, r);

    int loading = 0;
    for (int i = 0; i < m->count; i++) if (s->state[i] == AST_LOADING) loading++;
    while (loading < s->workers) {
        int i = pick_asset(m, r, s, gate_open);
        if (i < 0) break;
        s->state[i] = AST_LOADING;
        s->remaining_ms[i] = m->a[i].load_ms;
        s->start_ms[i] = now;
        loading++;
    }
    sample_mem(s, m);

    /* every worker spends up to FRAME_MS of the frame; completions are
       replayed in finish-time order so peaks and the completion log are
       deterministic regardless of manifest order */
    int fin_idx[MAX_ASSETS], fin_spend[MAX_ASSETS], fin_n = 0;
    for (int i = 0; i < m->count; i++) {
        if (s->state[i] != AST_LOADING) continue;
        int spend = s->remaining_ms[i] < FRAME_MS ? s->remaining_ms[i] : FRAME_MS;
        s->remaining_ms[i] -= spend;
        if (s->remaining_ms[i] == 0) {
            fin_idx[fin_n] = i;
            fin_spend[fin_n] = spend;
            fin_n++;
        }
    }
    for (int i = 0; i < fin_n; i++)
        for (int j = i + 1; j < fin_n; j++)
            if (fin_spend[j] < fin_spend[i] ||
                (fin_spend[j] == fin_spend[i] && fin_idx[j] < fin_idx[i])) {
                int t = fin_idx[i]; fin_idx[i] = fin_idx[j]; fin_idx[j] = t;
                t = fin_spend[i]; fin_spend[i] = fin_spend[j]; fin_spend[j] = t;
            }
    for (int k = 0; k < fin_n; k++) {
        int i = fin_idx[k];
        s->state[i] = AST_READY;
        s->resident_kb += m->a[i].size_kb;
        s->finish_ms[i] = now + fin_spend[k];
        s->order[s->order_len++] = i;
        sample_mem(s, m);
    }

    s->frames++;
    if (s->gate_ms < 0 && required_ready(s, m, r)) {
        s->gate_ms = now + FRAME_MS;      /* first frame boundary that can draw */
        s->gate_frame = s->frames;
        long long last = 0;
        for (int i = 0; i < m->count; i++)
            if (is_required(r, i) && s->finish_ms[i] > last) last = s->finish_ms[i];
        s->last_required_ready_ms = last;
        int ambient = 0;
        for (int i = 0; i < m->count; i++)
            if (!is_required(r, i) && s->state[i] == AST_READY) ambient++;
        s->ambient_ready_at_gate = ambient;
    }
}

static void stream_run(Stream* s, const Manifest* m, const Resolve* r) {
    int guard = MAX_FRAMES;
    while (!all_ready(s, m) && guard-- > 0) stream_step(s, m, r);
    s->total_ms = (long long)s->frames * FRAME_MS;
}

static void gate_status(const Manifest* m, const Resolve* r, const Stream* s, Gate* g) {
    memset(g, 0, sizeof(*g));
    if (!r->ok) {
        g->blocked = 1;
        g->missing = r->missing;
        snprintf(g->names, sizeof(g->names), "%s",
                 r->missing ? r->missing_names : r->cycle_names);
        return;
    }
    int pending = 0;
    for (int i = 0; i < m->count; i++) {
        if (!is_required(r, i)) continue;
        if (s && s->state[i] == AST_READY) continue;
        pending++;
        names_add(g->names, sizeof(g->names), m->a[i].name);
    }
    g->pending = pending;
    g->blocked = pending > 0;
}

static uint64_t hash_bytes(uint64_t h, const void* data, size_t n) {
    const unsigned char* p = (const unsigned char*)data;
    for (size_t i = 0; i < n; i++) {
        h ^= p[i];
        h *= 1099511628211ull;
    }
    return h;
}

/* stand-in for decoded content: real loaders would hash the decoded bytes */
static uint64_t hash_asset(uint64_t h, const Asset* a) {
    uint32_t kb = (uint32_t)a->size_kb;
    h = hash_bytes(h, a->name, strlen(a->name));
    h = hash_bytes(h, &kb, sizeof(kb));
    return h;
}

static uint64_t eager_digest(const Manifest* m, const Resolve* r) {
    uint64_t h = 1469598103934665603ull;
    for (int k = 0; k < r->topo_len; k++) {
        int i = r->topo[k];
        if (is_required(r, i)) h = hash_asset(h, &m->a[i]);
    }
    return h;
}

/* completion_order = 0: assemble in topological order (matches eager).
   completion_order = 1: assemble in load-completion order (the trap). */
static uint64_t streamed_digest(const Manifest* m, const Resolve* r,
                                const Stream* s, int completion_order) {
    uint64_t h = 1469598103934665603ull;
    if (completion_order) {
        for (int k = 0; k < s->order_len; k++) {
            int i = s->order[k];
            if (is_required(r, i) && s->state[i] == AST_READY)
                h = hash_asset(h, &m->a[i]);
        }
        return h;
    }
    for (int k = 0; k < r->topo_len; k++) {
        int i = r->topo[k];
        if (is_required(r, i) && s->state[i] == AST_READY)
            h = hash_asset(h, &m->a[i]);
    }
    return h;
}

static int deps_respected(const Manifest* m, const Resolve* r, const Stream* s) {
    for (int i = 0; i < m->count; i++) {
        if (!is_required(r, i)) continue;
        if (s->state[i] != AST_READY || s->start_ms[i] < 0) return 0;
        const Asset* a = &m->a[i];
        for (int d = 0; d < a->dep_count; d++) {
            int v = a->dep_idx[d];
            if (v < 0) return 0;
            if (s->state[v] != AST_READY) return 0;
            if (s->start_ms[i] < s->finish_ms[v]) return 0;
        }
    }
    return 1;
}

static void build_level(Manifest* m) {
    manifest_init(m);
    int level = m_add(m, "level_city", 256, 300, 100, 1);
    int geo = m_add(m, "zone_geo", 128, 150, 90, 0);
    int light = m_add(m, "zone_light", 128, 150, 90, 0);
    int audio = m_add(m, "zone_audio", 64, 100, 60, 0);
    int hud = m_add(m, "ui_hud", 64, 100, 95, 0);
    int nav = m_add(m, "nav_grid", 256, 300, 85, 0);
    int shader = m_add(m, "shader_std", 128, 200, 88, 0);
    int ground = m_add(m, "tex_ground", 4096, 600, 80, 0);
    int bldg = m_add(m, "tex_bldg", 3072, 500, 78, 0);
    int sky = m_add(m, "tex_sky", 2048, 400, 70, 0);
    int font = m_add(m, "font_main", 96, 150, 65, 0);
    int amb = m_add(m, "sfx_amb", 1024, 450, 55, 0);

    /* level_city needs its zones; each zone pulls in its own content */
    m_dep(m, level, "zone_geo");
    m_dep(m, level, "zone_light");
    m_dep(m, level, "zone_audio");
    m_dep(m, level, "ui_hud");
    m_dep(m, level, "nav_grid");
    m_dep(m, geo, "tex_ground");
    m_dep(m, geo, "tex_bldg");
    m_dep(m, light, "shader_std");
    m_dep(m, light, "tex_sky");
    m_dep(m, audio, "sfx_amb");
    m_dep(m, hud, "font_main");
    m_dep(m, bldg, "shader_std");

    /* not reachable from the root: background preload after the gate opens */
    m_add(m, "tex_decal", 512, 400, 20, 0);
    m_add(m, "vfx_smoke", 256, 300, 15, 0);
    m_add(m, "sfx_music", 768, 500, 10, 0);
    (void)level; (void)geo; (void)light; (void)audio; (void)hud; (void)nav;
    (void)shader; (void)ground; (void)bldg; (void)sky; (void)font; (void)amb;
}

static int sum_load_ms(const Manifest* m, const Resolve* r, int required_only) {
    int sum = 0;
    for (int i = 0; i < m->count; i++)
        if (!required_only || is_required(r, i)) sum += m->a[i].load_ms;
    return sum;
}

int main(void) {
    Manifest m;
    Resolve r;

    build_level(&m);
    resolve(&m, &r);

    int req = required_count(&r, m.count);
    int resident_required = 0, resident_all = 0;
    for (int i = 0; i < m.count; i++) {
        resident_all += m.a[i].size_kb;
        if (is_required(&r, i)) resident_required += m.a[i].size_kb;
    }

    printf("assets=%d required=%d frame_ms=%d\n", m.count, req, FRAME_MS);
    printf("resolve=%s\n", r.error);
    check("resolution_ok", r.ok);
    check("required_count", req == 12);
    check("critical_path_known", critical_path_ms(&m, &r) > 0);

    check("missing_dep_none", r.missing == 0);
    check("cycle_none", r.cycle == 0);

    Stream s;
    stream_init(&s, 4);
    Gate g;
    gate_status(&m, &r, &s, &g);
    check("gate_blocked_at_entry", g.blocked && g.pending == req);
    check("gate_names_pending", strstr(g.names, "level_city") != NULL);

    stream_run(&s, &m, &r);
    gate_status(&m, &r, &s, &g);
    check("gate_open_after_stream", !g.blocked && g.pending == 0);
    check("ambient_not_ready_at_gate", s.ambient_ready_at_gate == 0);

    int eager_required = sum_load_ms(&m, &r, 1);
    int eager_full = sum_load_ms(&m, &r, 0);
    uint64_t eager = eager_digest(&m, &r);
    int c1 = m_find(&m, "ui_hud");
    int c2 = m_find(&m, "zone_audio");

    printf("eager_required_ms=%d eager_full_ms=%d\n", eager_required, eager_full);
    printf("workers=4 first_playable_ms=%lld gate_frame=%d last_ready_ms=%lld\n",
           s.gate_ms, s.gate_frame, s.last_required_ready_ms);
    printf("critical_path_ms=%d total_ms=%lld\n", critical_path_ms(&m, &r), s.total_ms);
    printf("peak_entry_kb=%d peak_total_kb=%d\n", s.peak_entry_kb, s.peak_kb);
    printf("resident_required_kb=%d resident_all_kb=%d\n", resident_required, resident_all);

    check("streaming_beats_eager", s.gate_ms > 0 && s.gate_ms < eager_required);
    check("near_critical_path", s.gate_ms <= critical_path_ms(&m, &r) + 3 * FRAME_MS);
    check("all_assets_loaded", all_ready(&s, &m));
    check("deps_respected", deps_respected(&m, &r, &s));
    check("state_matches_eager", streamed_digest(&m, &r, &s, 0) == eager);
    printf("playable_digest=%016llx\n", (unsigned long long)eager);
    check("completion_order_differs", streamed_digest(&m, &r, &s, 1) != eager);

    /* single worker: priority order is observable and dependency waits hold */
    Stream s1;
    stream_init(&s1, 1);
    stream_run(&s1, &m, &r);
    check("single_worker_slower", s1.gate_ms > s.gate_ms);
    check("critical_path_asset_first",
          s1.start_ms[m_find(&m, "tex_ground")] < s1.start_ms[c1]);
    check("deps_before_dependents",
          s1.start_ms[c2] >= s1.finish_ms[m_find(&m, "sfx_amb")]);
    check("single_worker_deps_respected", deps_respected(&m, &r, &s1));

    /* peak memory vs worker count: more workers = more in-flight staging.
       peak_entry_kb is windowed (up to the gate), so a higher worker count
       can shrink it by opening the gate sooner; it is still bounded by the
       closure's residency plus one asset's staged bytes plus per-worker
       overhead. peak_total_kb is the number to size the budget from. */
    int conc[4] = { 1, 2, 4, 8 };
    int peak_total_monotonic = 1, prev_peak = -1, peak1 = 0, peak8 = 0;
    int entry_ok[4];
    int max_asset_kb = 0;
    for (int i = 0; i < m.count; i++)
        if (m.a[i].size_kb > max_asset_kb) max_asset_kb = m.a[i].size_kb;
    printf("concurrency,first_playable_ms,peak_entry_kb,peak_total_kb,total_ms\n");
    for (int k = 0; k < 4; k++) {
        Stream w;
        stream_init(&w, conc[k]);
        stream_run(&w, &m, &r);
        printf("%d,%lld,%d,%d,%lld\n", conc[k], w.gate_ms, w.peak_entry_kb, w.peak_kb, w.total_ms);
        if (prev_peak >= 0 && w.peak_kb < prev_peak) peak_total_monotonic = 0;
        prev_peak = w.peak_kb;
        if (k == 0) peak1 = w.peak_kb;
        if (k == 3) peak8 = w.peak_kb;
        entry_ok[k] = w.peak_entry_kb <= resident_required + conc[k] * STREAM_OVERHEAD_KB + max_asset_kb;
    }
    check("peak_total_monotonic", peak_total_monotonic);
    check("peak_grows_with_workers", peak8 > peak1);
    check("peak_entry_within_bound",
          entry_ok[0] && entry_ok[1] && entry_ok[2] && entry_ok[3]);
    printf("peak_total_growth_kb=%d\n", peak8 - peak1);

    /* missing dependency: resolve fails and the gate names it */
    Manifest broken;
    Resolve br;
    manifest_init(&broken);
    int broot = m_add(&broken, "level_missing", 128, 200, 100, 1);
    int bzone = m_add(&broken, "zone_x", 64, 100, 90, 0);
    m_dep(&broken, broot, "zone_x");
    m_dep(&broken, bzone, "tex_missing");
    resolve(&broken, &br);
    Stream bs;
    stream_init(&bs, 2);
    Gate bg;
    gate_status(&broken, &br, &bs, &bg);
    printf("missing_case=%s\n", br.error);
    check("missing_dep_rejected", !br.ok && br.missing == 1);
    check("missing_dep_named", strstr(br.missing_names, "tex_missing") != NULL);
    check("missing_dep_blocks_gate", bg.blocked && strstr(bg.names, "tex_missing") != NULL);

    /* cycle: rejected with the participating names */
    Manifest cyc;
    Resolve cr;
    manifest_init(&cyc);
    int ca = m_add(&cyc, "cyc_a", 64, 100, 100, 1);
    int cb = m_add(&cyc, "cyc_b", 64, 100, 90, 0);
    m_dep(&cyc, ca, "cyc_b");
    m_dep(&cyc, cb, "cyc_a");
    resolve(&cyc, &cr);
    printf("cycle_case=%s\n", cr.error);
    check("cycle_rejected", !cr.ok && cr.cycle);
    check("cycle_named", strstr(cr.cycle_names, "cyc_a") != NULL &&
                         strstr(cr.cycle_names, "cyc_b") != NULL);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
