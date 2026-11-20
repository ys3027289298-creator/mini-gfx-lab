/* navpath: versioned path export, migration and self-verification. */
#include "navpath.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NAV_DOC_BYTES (NAVPATH_MAX_PTS * 32 + 2048)
#define NAV_COST_TOL 1e-6   /* costs are written with 6 decimals */

static const int NAV_DIRS[8][2] = {
    {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}
};

double nav_cost(int dx, int dy) {
    if (dx == 0 && dy == 0) return 0.0;
    return hypot((double)dx, (double)dy);
}

/* ---------------------------------------------------------------- errors */

static int fail(NavPathErr* err, int code, int line, const char* msg) {
    if (err) {
        err->code = code;
        err->line = line;
        snprintf(err->msg, sizeof(err->msg), "%s", msg);
    }
    return code;
}

const char* navpath_strerror(int code) {
    switch (code) {
    case NAVPATH_OK:           return "ok";
    case NAVPATH_ERR_ARG:      return "bad argument";
    case NAVPATH_ERR_FORMAT:   return "malformed document";
    case NAVPATH_ERR_VERSION:  return "unsupported version";
    case NAVPATH_ERR_MISSING:  return "missing field";
    case NAVPATH_ERR_FIELD:    return "bad field";
    case NAVPATH_ERR_COORD:    return "illegal coordinate";
    case NAVPATH_ERR_ENDPOINT: return "endpoint mismatch";
    case NAVPATH_ERR_COUNT:    return "count mismatch";
    case NAVPATH_ERR_DATA:     return "corrupt payload";
    case NAVPATH_ERR_CAPACITY: return "buffer too small";
    default:                   return "unknown error";
    }
}

const char* navpath_bad_name(int kind) {
    switch (kind) {
    case NAVPATH_BAD_NONE:      return "none";
    case NAVPATH_BAD_ENDPOINT:  return "endpoint";
    case NAVPATH_BAD_COORD:     return "coordinate";
    case NAVPATH_BAD_SEG_COST:  return "segment_cost";
    case NAVPATH_BAD_TOTAL:     return "total_cost";
    case NAVPATH_BAD_EXPANDED:  return "expanded";
    default:                    return "unknown";
    }
}

/* --------------------------------------------------------------- helpers */

static int parse_int(const char* s, int* out) {
    if (!s || !*s) return 0;
    char* end = NULL;
    errno = 0;
    long v = strtol(s, &end, 10);
    if (errno || !end || *end != '\0' || v < INT_MIN || v > INT_MAX) return 0;
    *out = (int)v;
    return 1;
}

static int parse_uint(const char* s, int* out) {
    if (!s || !*s) return 0;
    for (const char* p = s; *p; p++) if (!isdigit((unsigned char)*p)) return 0;
    char* end = NULL;
    errno = 0;
    long v = strtol(s, &end, 10);
    if (errno || *end != '\0' || v < 0 || v > INT_MAX) return 0;
    *out = (int)v;
    return 1;
}

static int parse_double_strict(const char* s, double* out) {
    if (!s || !*s) return 0;
    char* end = NULL;
    errno = 0;
    double v = strtod(s, &end);
    if (errno || !end || *end != '\0' || !isfinite(v)) return 0;
    *out = v;
    return 1;
}

/* "x,y" with integer components */
static int parse_pt_token(const char* tok, NavPt* p) {
    const char* comma = strchr(tok, ',');
    if (!comma) return 0;
    char a[32];
    size_t l = (size_t)(comma - tok);
    if (l == 0 || l >= sizeof(a) || strlen(comma + 1) >= sizeof(a)) return 0;
    memcpy(a, tok, l);
    a[l] = '\0';
    return parse_int(a, &p->x) && parse_int(comma + 1, &p->y);
}

static int in_grid(int x, int y, int w, int h) {
    return x >= 0 && y >= 0 && x < w && y < h;
}

/* split a mutable line on blanks; returns token count or -1 on overflow */
static int split(char* line, char** tok, int max) {
    int n = 0;
    char* p = line;
    while (*p) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        if (n >= max) return -1;
        tok[n++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = '\0';
    }
    return n;
}

/* ------------------------------------------------------------- document */

int navpath_build(NavPathDoc* d, int w, int h, NavPt start, NavPt goal,
                  const NavPt* pts, int n, int expanded) {
    if (!d || !pts) return NAVPATH_ERR_ARG;
    if (w <= 0 || h <= 0 || n <= 0 || n > NAVPATH_MAX_PTS) return NAVPATH_ERR_ARG;
    if (!in_grid(start.x, start.y, w, h) || !in_grid(goal.x, goal.y, w, h))
        return NAVPATH_ERR_COORD;
    memset(d, 0, sizeof(*d));
    d->version = NAVPATH_VERSION;
    d->source_version = NAVPATH_VERSION;
    d->w = w; d->h = h;
    d->start = start; d->goal = goal;
    d->n = n;
    d->expanded = expanded;
    for (int i = 0; i < n; i++) {
        if (!in_grid(pts[i].x, pts[i].y, w, h)) return NAVPATH_ERR_COORD;
        d->pts[i] = pts[i];
    }
    if (pts[0].x != start.x || pts[0].y != start.y) return NAVPATH_ERR_ENDPOINT;
    if (pts[n-1].x != goal.x || pts[n-1].y != goal.y) return NAVPATH_ERR_ENDPOINT;
    d->seg_cost[0] = 0.0;
    double total = 0.0;
    for (int i = 1; i < n; i++) {
        double c = nav_cost(pts[i].x - pts[i-1].x, pts[i].y - pts[i-1].y);
        d->seg_cost[i] = c;
        total += c;
    }
    d->total_cost = total;
    d->flags = NAVPATH_HAS_COSTS | (expanded >= 0 ? NAVPATH_HAS_EXPANDED : 0);
    return NAVPATH_OK;
}

/* --------------------------------------------------------------- output */

typedef struct { char* b; size_t cap, len; int ovf; } Out;

static void out_fmt(Out* o, const char* fmt, ...) {
    if (o->ovf) return;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(o->b + o->len, o->cap - o->len, fmt, ap);
    va_end(ap);
    if (w < 0 || (size_t)w >= o->cap - o->len) { o->ovf = 1; return; }
    o->len += (size_t)w;
}

static void out_str(Out* o, const char* s) {
    size_t l = strlen(s);
    if (o->ovf || o->len + l + 1 > o->cap) { o->ovf = 1; return; }
    memcpy(o->b + o->len, s, l + 1);
    o->len += l;
}

/* unit step -> '1'..'8', anything longer stays "dx,dy" */
static void fmt_delta(const NavPt* a, const NavPt* b, char* buf, size_t cap) {
    int dx = b->x - a->x, dy = b->y - a->y;
    for (int i = 0; i < 8; i++) {
        if (NAV_DIRS[i][0] == dx && NAV_DIRS[i][1] == dy) {
            snprintf(buf, cap, "%d", i + 1);
            return;
        }
    }
    snprintf(buf, cap, "%d,%d", dx, dy);
}

/* write tokens with a "<repeat>*" prefix when a run repeats */
static void emit_rle(Out* o, char* const* tok, int n) {
    for (int i = 0; i < n; ) {
        int j = i + 1;
        while (j < n && strcmp(tok[j], tok[i]) == 0) j++;
        int rep = j - i;
        if (rep > 1) out_fmt(o, "%d*%s ", rep, tok[i]);
        else out_fmt(o, "%s ", tok[i]);
        i = j;
    }
}

static void render_points_v1(Out* o, const NavPathDoc* d) {
    for (int i = 0; i < d->n; i++)
        out_fmt(o, "(%d,%d)%s", d->pts[i].x, d->pts[i].y, i + 1 == d->n ? "" : " ");
}

static void render_points_plain(Out* o, const NavPathDoc* d) {
    for (int i = 0; i < d->n; i++)
        out_fmt(o, "%d,%d%s", d->pts[i].x, d->pts[i].y, i + 1 == d->n ? "" : " ");
}

/* segment costs only: n-1 values, seg_cost[0] is the implicit anchor */
static void render_costs_plain(Out* o, const NavPathDoc* d) {
    for (int i = 1; i < d->n; i++)
        out_fmt(o, "%.6f%s", d->seg_cost[i], i + 1 == d->n ? "" : " ");
}

static int render(const NavPathDoc* d, int version, int rle, Out* o) {
    if (version < 1 || version > NAVPATH_VERSION) return NAVPATH_ERR_VERSION;
    if (rle && version < 3) return NAVPATH_ERR_ARG;
    if (d->n <= 0 || d->n > NAVPATH_MAX_PTS) return NAVPATH_ERR_COUNT;

    if (version == 1) {
        /* legacy layout: no costs, no total, no expansion count */
        out_fmt(o, "NAVPATH version=1 w=%d h=%d start=%d,%d goal=%d,%d n=%d\n",
                d->w, d->h, d->start.x, d->start.y, d->goal.x, d->goal.y, d->n);
        out_str(o, "points ");
        render_points_v1(o, d);
        out_str(o, "\nend\n");
        return NAVPATH_OK;
    }

    out_fmt(o, "NAVPATH version=%d w=%d h=%d start=%d,%d goal=%d,%d n=%d"
               " expanded=%d total=%.6f",
            version, d->w, d->h, d->start.x, d->start.y, d->goal.x, d->goal.y,
            d->n, d->expanded, d->total_cost);
    if (rle) out_str(o, " codec=rle");
    out_str(o, "\n");

    if (!rle) {
        out_str(o, "points ");
        render_points_plain(o, d);
        out_str(o, "\ncosts ");
        render_costs_plain(o, d);
        out_str(o, "\n");
    } else {
        static char tok[NAVPATH_MAX_PTS][32];
        static char* ptr[NAVPATH_MAX_PTS];
        int m = 0;
        for (int i = 1; i < d->n; i++) {
            fmt_delta(&d->pts[i-1], &d->pts[i], tok[m], sizeof(tok[m]));
            ptr[m] = tok[m];
            m++;
        }
        out_str(o, "points ");
        emit_rle(o, ptr, m);
        out_str(o, "\n");
        m = 0;
        for (int i = 1; i < d->n; i++) {
            snprintf(tok[m], sizeof(tok[m]), "%.6f", d->seg_cost[i]);
            ptr[m] = tok[m];
            m++;
        }
        out_str(o, "costs ");
        emit_rle(o, ptr, m);
        out_str(o, "\n");
    }
    out_str(o, "end\n");
    return NAVPATH_OK;
}

int navpath_export(const NavPathDoc* d, int version, int compress,
                   char* out, size_t cap, size_t* out_len, NavPathErr* err) {
    if (!d || !out || cap == 0) return fail(err, NAVPATH_ERR_ARG, 0, "bad argument");
    if (version < 1 || version > NAVPATH_VERSION)
        return fail(err, NAVPATH_ERR_VERSION, 0, "unsupported export version");
    if (compress < 0 || compress > 2)
        return fail(err, NAVPATH_ERR_ARG, 0, "compress must be 0, 1 or 2");
    if (version < 3 && compress > 1)
        return fail(err, NAVPATH_ERR_ARG, 0, "forced compression needs version 3");
    if (d->n <= 0 || d->n > NAVPATH_MAX_PTS)
        return fail(err, NAVPATH_ERR_COUNT, 0, "path has no points");
    if (d->expanded < -1)
        return fail(err, NAVPATH_ERR_FIELD, 0, "invalid expansion count");

    /* two documents at most fit; static keeps ~260 KB off the stack */
    static char plain[NAV_DOC_BYTES], packed[NAV_DOC_BYTES];
    Out a = { plain, sizeof(plain), 0, 0 };
    int rc = render(d, version, 0, &a);
    if (rc) return fail(err, rc, 0, "render failed");
    if (a.ovf) return fail(err, NAVPATH_ERR_CAPACITY, 0, "document exceeds internal buffer");

    const char* chosen = plain;
    size_t len = a.len;
    if (version >= 3 && compress >= 1) {
        Out b = { packed, sizeof(packed), 0, 0 };
        rc = render(d, version, 1, &b);
        if (rc) return fail(err, rc, 0, "render failed");
        if (b.ovf) return fail(err, NAVPATH_ERR_CAPACITY, 0, "document exceeds internal buffer");
        if (compress == 2 || b.len < a.len) { chosen = packed; len = b.len; }
    }
    if (len + 1 > cap) return fail(err, NAVPATH_ERR_CAPACITY, 0, "output buffer too small");
    memcpy(out, chosen, len + 1);
    if (out_len) *out_len = len;
    return NAVPATH_OK;
}

/* ---------------------------------------------------------------- input */

typedef struct { const char* p; int line; } Cursor;

/* 1 = line read, 0 = end of input, -1 = line too long */
static int next_line(Cursor* c, char* out, size_t cap) {
    if (!*c->p) return 0;
    c->line++;
    size_t n = 0;
    while (*c->p && *c->p != '\n') {
        if (n + 1 >= cap) return -1;
        out[n++] = *c->p++;
    }
    if (*c->p == '\n') c->p++;
    while (n > 0 && (out[n-1] == '\r' || out[n-1] == ' ')) n--;
    out[n] = '\0';
    return 1;
}

/* parse key=value header; fills the presence flags it is handed */
static int parse_header(char* line, int lineno, NavPathDoc* d, int* codec_rle,
                        int* has_version, int* has_w, int* has_h, int* has_start,
                        int* has_goal, int* has_n, int* has_expanded, int* has_total,
                        NavPathErr* err) {
    char* tok[64];
    int nt = split(line, tok, 64);
    if (nt < 1 || strcmp(tok[0], "NAVPATH") != 0)
        return fail(err, NAVPATH_ERR_FORMAT, lineno, "missing NAVPATH header");
    for (int i = 1; i < nt; i++) {
        char* eq = strchr(tok[i], '=');
        if (!eq) return fail(err, NAVPATH_ERR_FORMAT, lineno, "malformed key=value");
        *eq = '\0';
        const char* k = tok[i];
        const char* v = eq + 1;
        int* seen = NULL;
        int bad = 0;
        if (strcmp(k, "version") == 0) {
            seen = has_version;
            if (!parse_int(v, &d->source_version)) bad = 1;
            else if (d->source_version < 1 || d->source_version > NAVPATH_VERSION) {
                char m[96];
                snprintf(m, sizeof(m), "unsupported version %d", d->source_version);
                return fail(err, NAVPATH_ERR_VERSION, lineno, m);
            }
        } else if (strcmp(k, "w") == 0) {
            seen = has_w; if (!parse_int(v, &d->w)) bad = 1;
        } else if (strcmp(k, "h") == 0) {
            seen = has_h; if (!parse_int(v, &d->h)) bad = 1;
        } else if (strcmp(k, "start") == 0) {
            seen = has_start; if (!parse_pt_token(v, &d->start)) bad = 1;
        } else if (strcmp(k, "goal") == 0) {
            seen = has_goal; if (!parse_pt_token(v, &d->goal)) bad = 1;
        } else if (strcmp(k, "n") == 0) {
            seen = has_n; if (!parse_int(v, &d->n)) bad = 1;
        } else if (strcmp(k, "expanded") == 0) {
            seen = has_expanded; if (!parse_int(v, &d->expanded)) bad = 1;
        } else if (strcmp(k, "total") == 0) {
            seen = has_total; if (!parse_double_strict(v, &d->total_cost)) bad = 1;
        } else if (strcmp(k, "codec") == 0) {
            if (*codec_rle) return fail(err, NAVPATH_ERR_FIELD, lineno, "duplicate codec");
            if (strcmp(v, "rle") != 0)
                return fail(err, NAVPATH_ERR_FIELD, lineno, "unknown codec");
            *codec_rle = 1;
            continue;
        } else {
            char m[96];
            snprintf(m, sizeof(m), "unknown field '%s'", k);
            return fail(err, NAVPATH_ERR_FIELD, lineno, m);
        }
        if (!seen) return fail(err, NAVPATH_ERR_FIELD, lineno, "field not valid in this version");
        if (*seen) return fail(err, NAVPATH_ERR_FIELD, lineno, "duplicate field");
        *seen = 1;
        if (bad) return fail(err, NAVPATH_ERR_FIELD, lineno, "malformed value");
    }
    return NAVPATH_OK;
}

/* v1 point list: "(x,y) (x,y) ..." */
static int parse_points_v1(char** tok, int nt, NavPathDoc* d, NavPathErr* err, int line) {
    if (nt != d->n) return fail(err, NAVPATH_ERR_COUNT, line, "point count does not match n");
    for (int i = 0; i < nt; i++) {
        const char* s = tok[i];
        size_t l = strlen(s);
        if (l < 5 || s[0] != '(' || s[l-1] != ')')
            return fail(err, NAVPATH_ERR_FORMAT, line, "point is not (x,y)");
        char inner[64];
        if (l - 2 >= sizeof(inner)) return fail(err, NAVPATH_ERR_FIELD, line, "point too long");
        memcpy(inner, s + 1, l - 2);
        inner[l-2] = '\0';
        if (!parse_pt_token(inner, &d->pts[i]))
            return fail(err, NAVPATH_ERR_FORMAT, line, "malformed point");
    }
    return NAVPATH_OK;
}

/* absolute "x,y" list */
static int parse_points_plain(char** tok, int nt, NavPathDoc* d, NavPathErr* err, int line) {
    if (nt != d->n) return fail(err, NAVPATH_ERR_COUNT, line, "point count does not match n");
    for (int i = 0; i < nt; i++)
        if (!parse_pt_token(tok[i], &d->pts[i]))
            return fail(err, NAVPATH_ERR_FORMAT, line, "malformed point");
    return NAVPATH_OK;
}

/* deltas from start, with <rep>*<token> run-length */
static int parse_points_rle(char** tok, int nt, NavPathDoc* d, NavPathErr* err, int line) {
    int count = 1;
    d->pts[0] = d->start;
    for (int i = 0; i < nt; i++) {
        char* s = tok[i];
        int rep = 1;
        char* star = strchr(s, '*');
        if (star) {
            *star = '\0';
            if (!parse_uint(s, &rep) || rep < 1)
                return fail(err, NAVPATH_ERR_FIELD, line, "bad repeat count");
            s = star + 1;
        }
        NavPt delta;
        if (s[0] >= '1' && s[0] <= '8' && s[1] == '\0') {
            int dir = s[0] - '1';
            delta.x = NAV_DIRS[dir][0];
            delta.y = NAV_DIRS[dir][1];
        } else if (!parse_pt_token(s, &delta)) {
            return fail(err, NAVPATH_ERR_FORMAT, line, "malformed delta token");
        }
        if (rep > NAVPATH_MAX_PTS || count + rep > NAVPATH_MAX_PTS)
            return fail(err, NAVPATH_ERR_COUNT, line, "repeat overflows point list");
        for (int k = 0; k < rep; k++) {
            NavPt prev = d->pts[count - 1];
            d->pts[count].x = prev.x + delta.x;
            d->pts[count].y = prev.y + delta.y;
            if (!in_grid(d->pts[count].x, d->pts[count].y, d->w, d->h)) {
                char m[96];
                snprintf(m, sizeof(m), "point %d leaves the grid", count);
                return fail(err, NAVPATH_ERR_COORD, line, m);
            }
            count++;
        }
    }
    if (count != d->n)
        return fail(err, NAVPATH_ERR_DATA, line, "decoded point count does not match n");
    return NAVPATH_OK;
}

static int parse_costs_plain(char** tok, int nt, NavPathDoc* d, NavPathErr* err, int line) {
    if (nt != d->n - 1) return fail(err, NAVPATH_ERR_COUNT, line, "cost count does not match n-1");
    for (int i = 0; i < nt; i++)
        if (!parse_double_strict(tok[i], &d->seg_cost[i + 1]))
            return fail(err, NAVPATH_ERR_FIELD, line, "malformed cost");
    return NAVPATH_OK;
}

static int parse_costs_rle(char** tok, int nt, NavPathDoc* d, NavPathErr* err, int line) {
    int count = 0;
    for (int i = 0; i < nt; i++) {
        char* s = tok[i];
        int rep = 1;
        char* star = strchr(s, '*');
        if (star) {
            *star = '\0';
            if (!parse_uint(s, &rep) || rep < 1)
                return fail(err, NAVPATH_ERR_FIELD, line, "bad repeat count");
            s = star + 1;
        }
        double c;
        if (!parse_double_strict(s, &c))
            return fail(err, NAVPATH_ERR_FIELD, line, "malformed cost");
        if (count + rep > d->n - 1)
            return fail(err, NAVPATH_ERR_COUNT, line, "cost list is too long");
        for (int k = 0; k < rep; k++) d->seg_cost[count++ + 1] = c;
    }
    if (count != d->n - 1) return fail(err, NAVPATH_ERR_COUNT, line, "cost count does not match n-1");
    return NAVPATH_OK;
}

/* old bare dump: whitespace-separated "(x,y)" tokens, optional "end" */
static int parse_legacy_bare(char* buf, int w, int h, NavPathDoc* d,
                             NavPathErr* err) {
    if (w <= 0 || h <= 0)
        return fail(err, NAVPATH_ERR_ARG, 0, "legacy dump needs grid dimensions");
    Cursor c = { buf, 0 };
    static char line[NAV_DOC_BYTES];
    char* tok[NAVPATH_MAX_PTS + 8];
    int n = 0, saw_end = 0;
    int r;
    while ((r = next_line(&c, line, sizeof(line))) > 0) {
        if (line[0] == '\0') continue;
        int nt = split(line, tok, NAVPATH_MAX_PTS + 8);
        if (nt < 0) return fail(err, NAVPATH_ERR_COUNT, c.line, "too many points");
        for (int i = 0; i < nt; i++) {
            if (strcmp(tok[i], "end") == 0) { saw_end = 1; break; }
            if (saw_end) return fail(err, NAVPATH_ERR_FORMAT, c.line, "trailing data");
            const char* s = tok[i];
            size_t l = strlen(s);
            if (l < 5 || s[0] != '(' || s[l-1] != ')')
                return fail(err, NAVPATH_ERR_FORMAT, c.line, "legacy point is not (x,y)");
            if (n >= NAVPATH_MAX_PTS)
                return fail(err, NAVPATH_ERR_COUNT, c.line, "too many points");
            char inner[64];
            if (l - 2 >= sizeof(inner))
                return fail(err, NAVPATH_ERR_FIELD, c.line, "point too long");
            memcpy(inner, s + 1, l - 2);
            inner[l-2] = '\0';
            if (!parse_pt_token(inner, &d->pts[n]))
                return fail(err, NAVPATH_ERR_FORMAT, c.line, "malformed point");
            if (!in_grid(d->pts[n].x, d->pts[n].y, w, h))
                return fail(err, NAVPATH_ERR_COORD, c.line, "point outside the grid");
            n++;
        }
    }
    if (r < 0) return fail(err, NAVPATH_ERR_FORMAT, c.line, "line too long");
    if (n < 1) return fail(err, NAVPATH_ERR_MISSING, 0, "legacy dump has no points");
    d->source_version = 1;
    d->w = w; d->h = h;
    d->n = n;
    d->start = d->pts[0];
    d->goal = d->pts[n-1];
    d->expanded = -1;
    return NAVPATH_OK;
}

/* Migration chain, applied in order after a document parses.
   v1 -> v2: derive per-segment costs and the total with nav_cost(); the
             expansion count is unknown, so it stays -1.
   v2 -> v3: the on-disk layout is unchanged; v3 only adds the optional RLE
             codec, and an expansion count may now be omitted. */
static void migrate_v1_to_v2(NavPathDoc* d) {
    d->seg_cost[0] = 0.0;
    double total = 0.0;
    for (int i = 1; i < d->n; i++) {
        double s = nav_cost(d->pts[i].x - d->pts[i-1].x,
                            d->pts[i].y - d->pts[i-1].y);
        d->seg_cost[i] = s;
        total += s;
    }
    d->total_cost = total;
    d->expanded = -1;
    d->flags |= NAVPATH_COSTS_DERIVED;
}

static void migrate_to_current(NavPathDoc* d) {
    if (d->source_version < 2) migrate_v1_to_v2(d);
    /* v2 -> v3 needs no data change */
    d->version = NAVPATH_VERSION;
}

int navpath_parse(const char* text, size_t len, int w, int h,
                  NavPathDoc* d, NavPathErr* err) {
    if (!text || !d) return fail(err, NAVPATH_ERR_ARG, 0, "bad argument");
    if (len == 0 || len > 1024 * 1024) return fail(err, NAVPATH_ERR_FORMAT, 0, "bad document size");
    memset(d, 0, sizeof(*d));
    d->expanded = -1;

    char* buf = (char*)malloc(len + 1);
    if (!buf) return fail(err, NAVPATH_ERR_ARG, 0, "out of memory");
    memcpy(buf, text, len);
    buf[len] = '\0';

    static char line[NAV_DOC_BYTES];
    static char* tok[NAVPATH_MAX_PTS + 8];
    int has_version = 0, has_w = 0, has_h = 0, has_start = 0, has_goal = 0;
    int has_n = 0, has_expanded = 0, has_total = 0, codec_rle = 0;
    int rc = NAVPATH_OK;
    int got_end = 0;

    const char* scan = buf;
    while (*scan == ' ' || *scan == '\t' || *scan == '\r' || *scan == '\n') scan++;
    if (*scan == '(') {
        rc = parse_legacy_bare(buf, w, h, d, err);
        if (rc) goto done;
        has_version = 1; /* implicit version 1 */
        goto migrate;
    }

    Cursor c = { buf, 0 };
    int r = next_line(&c, line, sizeof(line));
    if (r <= 0) { rc = fail(err, NAVPATH_ERR_FORMAT, 0, "empty document"); goto done; }
    if (r < 0) { rc = fail(err, NAVPATH_ERR_FORMAT, 0, "line too long"); goto done; }
    rc = parse_header(line, c.line, d, &codec_rle, &has_version, &has_w, &has_h,
                      &has_start, &has_goal, &has_n, &has_expanded, &has_total, err);
    if (rc) goto done;

    if (!has_version) { rc = fail(err, NAVPATH_ERR_MISSING, 1, "missing field 'version'"); goto done; }
    if (!has_w || !has_h) { rc = fail(err, NAVPATH_ERR_MISSING, 1, "missing field 'w'/'h'"); goto done; }
    if (!has_start || !has_goal) { rc = fail(err, NAVPATH_ERR_MISSING, 1, "missing 'start'/'goal'"); goto done; }
    if (!has_n) { rc = fail(err, NAVPATH_ERR_MISSING, 1, "missing field 'n'"); goto done; }
    if (d->source_version >= 2) {
        if (!has_expanded) { rc = fail(err, NAVPATH_ERR_MISSING, 1, "version >= 2 requires 'expanded'"); goto done; }
        if (!has_total) { rc = fail(err, NAVPATH_ERR_MISSING, 1, "version >= 2 requires 'total'"); goto done; }
    } else if (has_expanded || has_total) {
        rc = fail(err, NAVPATH_ERR_FIELD, 1, "version 1 does not carry cost fields"); goto done;
    }
    if (d->source_version < 3 && codec_rle) {
        rc = fail(err, NAVPATH_ERR_FIELD, 1, "codec is only valid for version 3"); goto done;
    }
    if (d->w <= 0 || d->h <= 0 || d->w > 4096 || d->h > 4096)
        { rc = fail(err, NAVPATH_ERR_FIELD, 1, "invalid grid size"); goto done; }
    if (d->n < 1 || d->n > NAVPATH_MAX_PTS)
        { rc = fail(err, NAVPATH_ERR_COUNT, 1, "invalid point count"); goto done; }
    if (!in_grid(d->start.x, d->start.y, d->w, d->h) ||
        !in_grid(d->goal.x, d->goal.y, d->w, d->h))
        { rc = fail(err, NAVPATH_ERR_COORD, 1, "start or goal outside the grid"); goto done; }
    if (w != 0 && h != 0 && (w != d->w || h != d->h))
        { rc = fail(err, NAVPATH_ERR_FIELD, 1, "grid size does not match the caller"); goto done; }

    /* points */
    r = next_line(&c, line, sizeof(line));
    if (r <= 0) { rc = fail(err, NAVPATH_ERR_MISSING, c.line, "missing points line"); goto done; }
    if (r < 0) { rc = fail(err, NAVPATH_ERR_FORMAT, c.line, "line too long"); goto done; }
    {
        int nt = split(line, tok, NAVPATH_MAX_PTS + 8);
        if (nt < 1) { rc = fail(err, NAVPATH_ERR_MISSING, c.line, "missing points line"); goto done; }
        if (nt < 0) { rc = fail(err, NAVPATH_ERR_COUNT, c.line, "too many points"); goto done; }
        if (strcmp(tok[0], "points") != 0)
            { rc = fail(err, NAVPATH_ERR_MISSING, c.line, "missing points payload"); goto done; }
        if (d->source_version == 1) rc = parse_points_v1(tok + 1, nt - 1, d, err, c.line);
        else if (codec_rle) rc = parse_points_rle(tok + 1, nt - 1, d, err, c.line);
        else rc = parse_points_plain(tok + 1, nt - 1, d, err, c.line);
        if (rc) goto done;
    }

    /* costs (v2+) */
    if (d->source_version >= 2) {
        r = next_line(&c, line, sizeof(line));
        if (r <= 0) { rc = fail(err, NAVPATH_ERR_MISSING, c.line, "missing costs line"); goto done; }
        if (r < 0) { rc = fail(err, NAVPATH_ERR_FORMAT, c.line, "line too long"); goto done; }
        int nt = split(line, tok, NAVPATH_MAX_PTS + 8);
        if (nt < 1) { rc = fail(err, NAVPATH_ERR_MISSING, c.line, "missing costs payload"); goto done; }
        if (nt < 0) { rc = fail(err, NAVPATH_ERR_COUNT, c.line, "too many costs"); goto done; }
        if (strcmp(tok[0], "costs") != 0)
            { rc = fail(err, NAVPATH_ERR_MISSING, c.line, "missing costs line"); goto done; }
        if (codec_rle) rc = parse_costs_rle(tok + 1, nt - 1, d, err, c.line);
        else rc = parse_costs_plain(tok + 1, nt - 1, d, err, c.line);
        if (rc) goto done;
    }

    /* end marker, then only blank lines */
    while ((r = next_line(&c, line, sizeof(line))) > 0) {
        if (line[0] == '\0') continue;
        int nt = split(line, tok, 4);
        if (!got_end && nt == 1 && strcmp(tok[0], "end") == 0) { got_end = 1; continue; }
        rc = fail(err, NAVPATH_ERR_FORMAT, c.line, got_end ? "trailing data" : "missing 'end'");
        goto done;
    }
    if (r < 0) { rc = fail(err, NAVPATH_ERR_FORMAT, c.line, "line too long"); goto done; }
    if (!got_end) { rc = fail(err, NAVPATH_ERR_MISSING, c.line, "missing 'end'"); goto done; }

migrate:
    migrate_to_current(d);
    if (d->expanded >= 0) d->flags |= NAVPATH_HAS_EXPANDED;
    d->flags |= NAVPATH_HAS_COSTS;
    if (codec_rle) d->flags |= NAVPATH_COMPRESSED;

done:
    free(buf);
    return rc;
}

/* ------------------------------------------------------------- verify */

static int check_fail(NavPathCheck* o, int kind, int seg, const char* fmt, ...) {
    o->ok = 0;
    o->bad_kind = kind;
    o->first_bad_segment = seg;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(o->detail, sizeof(o->detail), fmt, ap);
    va_end(ap);
    return 1;
}

int navpath_verify(const NavPathDoc* d, int w, int h, int expected_expanded,
                   NavPathCheck* out) {
    if (!out) return NAVPATH_ERR_ARG;
    memset(out, 0, sizeof(*out));
    out->first_bad_segment = -1;
    out->bad_kind = NAVPATH_BAD_NONE;
    if (!d) return check_fail(out, NAVPATH_BAD_COORD, -1, "no document");
    if (d->n < 1 || d->n > NAVPATH_MAX_PTS)
        return check_fail(out, NAVPATH_BAD_COORD, -1, "invalid point count %d", d->n);

    if (w == 0 && h == 0) { w = d->w; h = d->h; }
    if (w != d->w || h != d->h)
        return check_fail(out, NAVPATH_BAD_COORD, -1,
                          "grid mismatch: doc %dx%d, expected %dx%d", d->w, d->h, w, h);
    if (!(d->flags & NAVPATH_HAS_COSTS))
        return check_fail(out, NAVPATH_BAD_SEG_COST, 0, "document carries no segment costs");

    for (int i = 0; i < d->n; i++)
        if (!in_grid(d->pts[i].x, d->pts[i].y, w, h))
            return check_fail(out, NAVPATH_BAD_COORD, i,
                              "point %d (%d,%d) is outside %dx%d",
                              i, d->pts[i].x, d->pts[i].y, w, h);
    if (d->pts[0].x != d->start.x || d->pts[0].y != d->start.y)
        return check_fail(out, NAVPATH_BAD_ENDPOINT, 0,
                          "first point (%d,%d) != start (%d,%d)",
                          d->pts[0].x, d->pts[0].y, d->start.x, d->start.y);
    if (d->pts[d->n-1].x != d->goal.x || d->pts[d->n-1].y != d->goal.y)
        return check_fail(out, NAVPATH_BAD_ENDPOINT, d->n - 1,
                          "last point (%d,%d) != goal (%d,%d)",
                          d->pts[d->n-1].x, d->pts[d->n-1].y, d->goal.x, d->goal.y);
    if (fabs(d->seg_cost[0]) > NAV_COST_TOL)
        return check_fail(out, NAVPATH_BAD_SEG_COST, 0,
                          "segment 0 cost is %.6f, expected 0", d->seg_cost[0]);

    double total = 0.0;
    for (int i = 1; i < d->n; i++) {
        double want = nav_cost(d->pts[i].x - d->pts[i-1].x,
                               d->pts[i].y - d->pts[i-1].y);
        total += want;
        if (fabs(want - d->seg_cost[i]) > NAV_COST_TOL)
            return check_fail(out, NAVPATH_BAD_SEG_COST, i,
                              "segment %d (%d,%d)->(%d,%d): exported %.6f, recomputed %.6f",
                              i, d->pts[i-1].x, d->pts[i-1].y,
                              d->pts[i].x, d->pts[i].y,
                              d->seg_cost[i], want);
    }
    double tol = NAV_COST_TOL * (d->n > 1 ? d->n : 1);
    if (fabs(total - d->total_cost) > tol)
        return check_fail(out, NAVPATH_BAD_TOTAL, -1,
                          "total cost: exported %.6f, recomputed %.6f",
                          d->total_cost, total);
    if (expected_expanded >= 0) {
        if (d->expanded < 0)
            return check_fail(out, NAVPATH_BAD_EXPANDED, -1,
                              "expansion count missing, expected %d", expected_expanded);
        if (d->expanded != expected_expanded)
            return check_fail(out, NAVPATH_BAD_EXPANDED, -1,
                              "expanded nodes: exported %d, search %d",
                              d->expanded, expected_expanded);
    }
    out->ok = 1;
    return 0;
}
