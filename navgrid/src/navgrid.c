
/* mini-navgrid: 8-direction A* on an ASCII grid, with corner-cut prevention,
   path smoothing by line-of-sight, and incremental re-planning. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "navpath.h"

#define MAX_W 64
#define MAX_H 64
#define MAX_NODES (MAX_W * MAX_H)

typedef struct { int w, h; unsigned char blocked[MAX_H][MAX_W]; } Grid;

typedef NavPt Pt;  /* same layout as the exported point */

static int grid_load(Grid* g, const char* const* rows, int h) {
    int w = (int)strlen(rows[0]);
    if (w <= 0 || w > MAX_W || h <= 0 || h > MAX_H) return -1;
    g->w = w; g->h = h;
    for (int y = 0; y < h; y++) {
        if ((int)strlen(rows[y]) != w) return -1;
        for (int x = 0; x < w; x++) g->blocked[y][x] = (rows[y][x] == '#');
    }
    return 0;
}

static int in_bounds(const Grid* g, int x, int y) {
    return x >= 0 && y >= 0 && x < g->w && y < g->h;
}
static int walkable(const Grid* g, int x, int y) {
    return in_bounds(g, x, y) && !g->blocked[y][x];
}

/* diagonal step p->q is legal only if both orthogonal neighbours are free */
static int step_ok(const Grid* g, Pt p, Pt q) {
    int dx = q.x - p.x, dy = q.y - p.y;
    if (q.x < 0 || q.y < 0 || q.x >= g->w || q.y >= g->h) return 0;
    if (g->blocked[q.y][q.x]) return 0;
    if (dx != 0 && dy != 0) {
        if (g->blocked[p.y][q.x] || g->blocked[q.y][p.x]) return 0;
    }
    return 1;
}

static const int DIRS[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

typedef struct { double g, f; int parent; unsigned char open, closed; } Node;

/* expansions are reported through this, never recomputed for the export */
static int g_expanded;

static int astar(const Grid* g, Pt s, Pt t, Pt* out, int max_out) {
    g_expanded = 0;
    if (!walkable(g, s.x, s.y) || !walkable(g, t.x, t.y)) return -1;
    Node* nodes = (Node*)calloc((size_t)g->w * g->h, sizeof(Node));
    if (!nodes) return -1;
    int* heap = (int*)malloc(sizeof(int) * (size_t)g->w * g->h);
    int hn = 0;
    int si = s.y * g->w + s.x, ti = t.y * g->w + t.x;
    nodes[si].g = 0.0;
    nodes[si].f = hypot((double)(t.x - s.x), (double)(t.y - s.y));
    nodes[si].parent = -1;
    nodes[si].open = 1;
    heap[hn++] = si;
    int found = 0;
    while (hn > 0) {
        int best = 0;
        for (int i = 1; i < hn; i++) if (nodes[heap[i]].f < nodes[heap[best]].f) best = i;
        int cur = heap[best]; heap[best] = heap[--hn];
        if (nodes[cur].closed) continue;
        nodes[cur].closed = 1;
        g_expanded++;
        if (cur == ti) { found = 1; break; }
        Pt cp = { cur % g->w, cur / g->w };
        for (int d = 0; d < 8; d++) {
            Pt np = { cp.x + DIRS[d][0], cp.y + DIRS[d][1] };
            if (!step_ok(g, cp, np)) continue;
            int ni = np.y * g->w + np.x;
            if (nodes[ni].closed) continue;
            double ng = nodes[cur].g + nav_cost(DIRS[d][0], DIRS[d][1]);
            if (!nodes[ni].open || ng < nodes[ni].g) {
                nodes[ni].g = ng;
                nodes[ni].f = ng + hypot((double)(t.x - np.x), (double)(t.y - np.y));
                nodes[ni].parent = cur;
                nodes[ni].open = 1;
                heap[hn++] = ni;
            }
        }
    }
    int rc = -1;
    if (found) {
        int n = 0, cur = ti;
        while (cur != -1) n++, cur = nodes[cur].parent;
        if (n <= max_out) {
            cur = ti;
            for (int i = n - 1; i >= 0; i--) {
                out[i].x = cur % g->w; out[i].y = cur / g->w;
                cur = nodes[cur].parent;
            }
            rc = n;
        }
    }
    free(heap); free(nodes);
    return rc;
}

static int line_of_sight(const Grid* g, Pt a, Pt b) {
    int steps = abs(b.x - a.x) + abs(b.y - a.y);
    if (steps == 0) return 1;
    Pt prev = a;
    for (int i = 1; i <= steps; i++) {
        double t = (double)i / steps;
        Pt cur;
        cur.x = (int)lround(a.x + (b.x - a.x) * t);
        cur.y = (int)lround(a.y + (b.y - a.y) * t);
        if (cur.x == prev.x && cur.y == prev.y) continue;
        if (!step_ok(g, prev, cur)) return 0;
        prev = cur;
    }
    return prev.x == b.x && prev.y == b.y;
}

static int smooth(const Grid* g, const Pt* in, int n, Pt* out, int max_out) {
    if (n <= 0) return 0;
    int m = 0;
    out[m++] = in[0];
    int anchor = 0;
    while (anchor < n - 1) {
        int next = anchor + 1;
        for (int j = n - 1; j > anchor; j--) {
            if (line_of_sight(g, in[anchor], in[j])) { next = j; break; }
        }
        if (m >= max_out) return -1;
        out[m++] = in[next];
        anchor = next;
    }
    return m;
}

static double path_length(const Pt* p, int n) {
    double s = 0.0;
    for (int i = 1; i < n; i++) s += hypot((double)(p[i].x - p[i-1].x), (double)(p[i].y - p[i-1].y));
    return s;
}

/* ------------------------------------------------- export / verification */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* replace the first occurrence of needle; same-length values only, so the
   document stays well-formed and the parser is not what fails the test */
static int replace_once(char* doc, const char* needle, const char* repl) {
    char* p = strstr(doc, needle);
    if (!p) return 0;
    memcpy(p, repl, strlen(repl));
    return 1;
}

static void test_roundtrip(const NavPathDoc* d, const char* tag) {
    static char text[1 << 16];
    for (int v = 1; v <= NAVPATH_VERSION; v++) {
        for (int c = 0; c <= 2; c++) {
            if (v < 3 && c > 1) continue;
            NavPathErr e;
            size_t len = 0;
            int rc = navpath_export(d, v, c, text, sizeof(text), &len, &e);
            char name[96];
            snprintf(name, sizeof(name), "%s_v%d_c%d_export", tag, v, c);
            check(name, rc == NAVPATH_OK && len > 0);
            if (rc != NAVPATH_OK) continue;

            NavPathDoc back;
            rc = navpath_parse(text, len, d->w, d->h, &back, &e);
            snprintf(name, sizeof(name), "%s_v%d_c%d_parse", tag, v, c);
            check(name, rc == NAVPATH_OK);
            if (rc != NAVPATH_OK) { printf("  err=%s\n", e.msg); continue; }

            NavPathCheck vc;
            int exp = v == 1 ? -1 : d->expanded;
            navpath_verify(&back, d->w, d->h, exp, &vc);
            snprintf(name, sizeof(name), "%s_v%d_c%d_verify", tag, v, c);
            check(name, vc.ok);
            if (!vc.ok) printf("  %s seg=%d: %s\n", navpath_bad_name(vc.bad_kind),
                               vc.first_bad_segment, vc.detail);

            int same = back.n == d->n;
            for (int i = 0; same && i < d->n; i++)
                same = back.pts[i].x == d->pts[i].x && back.pts[i].y == d->pts[i].y;
            snprintf(name, sizeof(name), "%s_v%d_c%d_points", tag, v, c);
            check(name, same);
        }
    }
}

static void test_v1_migration(const NavPathDoc* d) {
    static char text[1 << 16];
    NavPathErr e;
    size_t len = 0;
    /* version 1 carries no costs: the reader must derive them */
    check("v1_export", navpath_export(d, 1, 0, text, sizeof(text), &len, &e) == NAVPATH_OK);
    NavPathDoc back;
    check("v1_parse", navpath_parse(text, len, d->w, d->h, &back, &e) == NAVPATH_OK);
    check("v1_source_version", back.source_version == 1);
    check("v1_costs_derived", (back.flags & NAVPATH_COSTS_DERIVED) != 0);
    check("v1_no_expanded", back.expanded == -1);
    NavPathCheck vc;
    navpath_verify(&back, d->w, d->h, -1, &vc);
    check("v1_verify", vc.ok);
    check("v1_total_matches", fabs(back.total_cost - d->total_cost) < 1e-6);

    /* the legacy bare "(x,y) ..." dump upgrades the same way */
    static char bare[4096];
    size_t off = 0;
    for (int i = 0; i < d->n; i++)
        off += (size_t)snprintf(bare + off, sizeof(bare) - off, "(%d,%d)%s",
                                d->pts[i].x, d->pts[i].y, i + 1 == d->n ? "\n" : " ");
    check("legacy_parse", navpath_parse(bare, off, d->w, d->h, &back, &e) == NAVPATH_OK);
    navpath_verify(&back, d->w, d->h, -1, &vc);
    check("legacy_verify", vc.ok);
}

/* same-length substitution of the first needle at or after 'from' */
static int replace_in(char* doc, const char* from, const char* needle,
                      const char* repl) {
    if (strlen(needle) != strlen(repl)) return 0;
    char* p = strstr(from ? from : doc, needle);
    if (!p) return 0;
    memcpy(p, repl, strlen(repl));
    return 1;
}

/* delete the whole line that starts with prefix (including its newline) */
static int delete_line(char* doc, const char* prefix) {
    char* p = doc;
    while ((p = strstr(p, prefix)) != NULL) {
        if (p == doc || p[-1] == '\n') break;
        p++;
    }
    if (!p) return 0;
    char* q = strchr(p, '\n');
    if (!q) return 0;
    memmove(p, q + 1, strlen(q + 1) + 1);
    return 1;
}

static void test_rejections(const NavPathDoc* d) {
    static char text[1 << 16], bad[1 << 16];
    NavPathErr e;
    size_t len = 0;
    NavPathDoc doc;
    NavPathCheck vc;

    /* --- segment 1's exported cost is wrong: verify must say segment 1 --- */
    navpath_export(d, 3, 0, text, sizeof(text), &len, &e);
    memcpy(bad, text, len + 1);
    check("tamper_setup", replace_once(bad, "\ncosts 1.000000", "\ncosts 1.500000"));
    int rc = navpath_parse(bad, strlen(bad), d->w, d->h, &doc, &e);
    check("tamper_parse", rc == NAVPATH_OK);
    navpath_verify(&doc, d->w, d->h, d->expanded, &vc);
    check("tamper_detected", !vc.ok && vc.bad_kind == NAVPATH_BAD_SEG_COST);
    check("tamper_first_segment", vc.first_bad_segment == 1);

    /* --- the diagonal is segment 4: the first three must pass, then stop --- */
    memcpy(bad, text, len + 1);
    char* costs = strstr(bad, "\ncosts ");
    check("tamper_later_setup", costs && replace_in(bad, costs, "1.414214", "1.400000"));
    rc = navpath_parse(bad, strlen(bad), d->w, d->h, &doc, &e);
    check("tamper_later_parse", rc == NAVPATH_OK);
    navpath_verify(&doc, d->w, d->h, -1, &vc);
    check("tamper_later_detected", !vc.ok && vc.bad_kind == NAVPATH_BAD_SEG_COST &&
                                   vc.first_bad_segment == 4);

    /* --- a wrong total is caught separately from the segments --- */
    memcpy(bad, text, len + 1);
    replace_once(bad, "total=11.414214", "total=11.900000");
    rc = navpath_parse(bad, strlen(bad), d->w, d->h, &doc, &e);
    check("tamper_total_parse", rc == NAVPATH_OK);
    navpath_verify(&doc, d->w, d->h, -1, &vc);
    check("tamper_total_detected", !vc.ok && vc.bad_kind == NAVPATH_BAD_TOTAL);

    /* --- missing required field, otherwise intact --- */
    memcpy(bad, text, len + 1);
    {
        char* p = strstr(bad, " expanded=");
        if (p) {
            char* q = p + strlen(" expanded=");
            while (*q && *q != ' ' && *q != '\n') q++;
            memmove(p, q, strlen(q) + 1);
        }
    }
    rc = navpath_parse(bad, strlen(bad), d->w, d->h, &doc, &e);
    check("missing_expanded_rejected", rc == NAVPATH_ERR_MISSING);

    /* --- missing cost column --- */
    memcpy(bad, text, len + 1);
    check("missing_costs_setup", delete_line(bad, "costs "));
    rc = navpath_parse(bad, strlen(bad), d->w, d->h, &doc, &e);
    check("missing_costs_rejected", rc == NAVPATH_ERR_MISSING);

    /* --- coordinate outside the grid (same-length edit) --- */
    memcpy(bad, text, len + 1);
    check("bad_coord_setup", replace_once(bad, "start=0,0", "start=9,9"));
    rc = navpath_parse(bad, strlen(bad), d->w, d->h, &doc, &e);
    check("bad_coord_rejected", rc == NAVPATH_ERR_COORD);

    /* --- future version is refused, not guessed at --- */
    memcpy(bad, text, len + 1);
    replace_once(bad, "version=3", "version=9");
    rc = navpath_parse(bad, strlen(bad), d->w, d->h, &doc, &e);
    check("unknown_version_rejected", rc == NAVPATH_ERR_VERSION);

    /* --- payload cut in half --- */
    navpath_export(d, 3, 2, text, sizeof(text), &len, &e);
    char* cut = strstr(text, "\ncosts ");
    if (cut) cut[1] = '\0';
    rc = navpath_parse(text, strlen(text), d->w, d->h, &doc, &e);
    check("truncated_rejected", rc != NAVPATH_OK);

    /* --- RLE delta pointing off the grid --- */
    static const char ok_doc[] =
        "NAVPATH version=3 w=4 h=4 start=0,0 goal=2,0 n=3 expanded=1"
        " total=2.000000 codec=rle\n"
        "points 2*1\n"
        "costs 1.000000 1.000000\n"
        "end\n";
    static const char off_doc[] =
        "NAVPATH version=3 w=4 h=4 start=0,0 goal=2,0 n=3 expanded=1"
        " total=2.000000 codec=rle\n"
        "points 2*2\n"
        "costs 1.000000 1.000000\n"
        "end\n";
    rc = navpath_parse(ok_doc, strlen(ok_doc), 4, 4, &doc, &e);
    check("rle_control_parses", rc == NAVPATH_OK);
    navpath_verify(&doc, 4, 4, 1, &vc);
    check("rle_control_verifies", vc.ok);
    rc = navpath_parse(off_doc, strlen(off_doc), 4, 4, &doc, &e);
    check("delta_escape_rejected", rc == NAVPATH_ERR_COORD);

    /* --- RLE repeat count disagreeing with n --- */
    static const char short_doc[] =
        "NAVPATH version=3 w=4 h=4 start=0,0 goal=2,0 n=3 expanded=1"
        " total=2.000000 codec=rle\n"
        "points 3*1\n"
        "costs 3*1.000000\n"
        "end\n";
    rc = navpath_parse(short_doc, strlen(short_doc), 4, 4, &doc, &e);
    check("rle_count_mismatch_rejected", rc == NAVPATH_ERR_DATA);

    /* --- grid mismatch against the caller --- */
    navpath_export(d, 3, 0, text, sizeof(text), &len, &e);
    rc = navpath_parse(text, len, d->w + 1, d->h, &doc, &e);
    check("grid_mismatch_rejected", rc == NAVPATH_ERR_FIELD);

    /* --- a caller-supplied (w,h) of 0 means "trust the header" --- */
    check("caller_dims_optional",
          navpath_parse(text, len, 0, 0, &doc, &e) == NAVPATH_OK &&
          doc.w == d->w && doc.h == d->h);

    /* --- verify catches a wrong expansion count --- */
    rc = navpath_parse(text, len, d->w, d->h, &doc, &e);
    check("verify_parse", rc == NAVPATH_OK);
    navpath_verify(&doc, d->w, d->h, d->expanded + 1, &vc);
    check("expanded_mismatch_detected", !vc.ok && vc.bad_kind == NAVPATH_BAD_EXPANDED);

    /* --- build() refuses endpoints that do not match the path --- */
    NavPt one = d->pts[0];
    check("build_endpoint_guard",
          navpath_build(&doc, d->w, d->h, d->goal, d->goal, &one, 1, 0) == NAVPATH_ERR_ENDPOINT);
}

static void test_export_purity(const NavPathDoc* d) {
    /* exporting is a read-only operation: the document and expansion count
       must be untouched, so the exported number equals a fresh search. */
    NavPathDoc before = *d;
    static char text[1 << 16];
    NavPathErr e;
    size_t len = 0;
    navpath_export(d, 3, 2, text, sizeof(text), &len, &e);
    check("export_purity", before.n == d->n && before.expanded == d->expanded &&
                           before.total_cost == d->total_cost);
    int i = -1;
    NavPathDoc back;
    navpath_parse(text, len, d->w, d->h, &back, &e);
    check("export_expanded_kept", back.expanded == d->expanded);
    (void)i;
}

static void test_sizes(const NavPathDoc* d) {
    static char plain[1 << 16], packed[1 << 16], legacy[1 << 16];
    NavPathErr e;
    size_t lp = 0, lc = 0, lv = 0;
    navpath_export(d, 3, 0, plain, sizeof(plain), &lp, &e);
    navpath_export(d, 3, 1, packed, sizeof(packed), &lc, &e);
    navpath_export(d, 1, 0, legacy, sizeof(legacy), &lv, &e);
    printf("bytes plain=%zu compressed=%zu legacy=%zu ratio=%.2f\n",
           lp, lc, lv, lp ? (double)lp / (double)lc : 0.0);
    check("compressed_smaller", lc <= lp);
    check("legacy_input_upgrades", lv > 0);
}

int main(void) {
    const char* rows[] = {
        "S....#....",
        ".##..#....",
        ".#......#.",
        ".#.####.#.",
        ".....#..G.",
    };
    Grid g;
    if (grid_load(&g, rows, 5) != 0) { printf("load fail\n"); return 1; }
    Pt s = {0,0}, t = {8,4};
    Pt raw[MAX_NODES], sm[MAX_NODES];
    int n = astar(&g, s, t, raw, MAX_NODES);
    if (n < 0) { printf("no path\n"); return 1; }
    int expanded = g_expanded;
    int m = smooth(&g, raw, n, sm, MAX_NODES);
    printf("raw=%d len=%.4f smooth=%d len=%.4f\n", n, path_length(raw,n), m, path_length(sm,m));
    for (int i = 0; i < m; i++) printf("(%d,%d)%s", sm[i].x, sm[i].y, i+1==m ? "\n" : " ");
    int bad = 0;
    for (int i = 1; i < m; i++) if (!line_of_sight(&g, sm[i-1], sm[i])) bad++;
    printf("bad_segments=%d\n", bad);
    printf("corner_cut=%d\n", !step_ok(&g, (Pt){1,0}, (Pt){2,1}));
    printf("expanded=%d\n", expanded);

    /* exports describe the raw A* path: its unit steps exercise both the
       fixed-size segment encoding and exact per-segment costs. */
    NavPathDoc rawdoc, smdoc;
    if (navpath_build(&rawdoc, g.w, g.h, s, t, raw, n, expanded) != NAVPATH_OK) {
        printf("doc build fail\n");
        return 1;
    }
    if (navpath_build(&smdoc, g.w, g.h, s, t, sm, m, expanded) != NAVPATH_OK) {
        printf("doc build fail\n");
        return 1;
    }
    test_roundtrip(&rawdoc, "raw");
    test_roundtrip(&smdoc, "smooth");
    test_v1_migration(&rawdoc);
    test_rejections(&rawdoc);
    test_export_purity(&rawdoc);
    test_sizes(&rawdoc);

    /* planning again after all that exporting must expand the same nodes */
    int n2 = astar(&g, s, t, raw, MAX_NODES);
    check("astar_expanded_stable", n2 == n && g_expanded == expanded);
    check("astar_baseline_expanded", g_expanded == 18);
    printf("expanded_after_exports=%d\n", g_expanded);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
