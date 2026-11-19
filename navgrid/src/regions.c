/* regions.c: chunk-clipped connected-component labelling of walkable cells.

   Partition rule: regions are maximal 4-connected sets of walkable cells that
   never cross a render-chunk edge.  A region therefore belongs to exactly one
   chunk, which makes chunk residency closed under region residency (a chunk
   and all its regions can be loaded/unloaded together) and lets every region
   be culled or drawn as a single batch.

   Ids are canonical (min linear cell index + 1), so incremental relabelling
   can be compared cell-for-cell with a from-scratch partition. */
#include "regions.h"

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { FLOOD_ALL, FLOOD_MERGE, FLOOD_OLD };

/* Chunk-aligned half-open cell bounds: [x0,x1) x [y0,y1). */
static void chunk_bounds(const RegionGrid *g, int x, int y,
                         int *x0, int *y0, int *x1, int *y1) {
    int cx = x / g->chunk, cy = y / g->chunk;
    *x0 = cx * g->chunk; *y0 = cy * g->chunk;
    *x1 = *x0 + g->chunk; if (*x1 > g->w) *x1 = g->w;
    *y1 = *y0 + g->chunk; if (*y1 > g->h) *y1 = g->h;
}

/* In-chunk orthogonal neighbours of idx, written to out (max 4). */
static int neighbours(const RegionGrid *g, int idx, int out[4]) {
    int x = idx % g->w, y = idx / g->w;
    int x0, y0, x1, y1;
    chunk_bounds(g, x, y, &x0, &y0, &x1, &y1);
    int n = 0;
    if (x > x0) out[n++] = idx - 1;
    if (x + 1 < x1) out[n++] = idx + 1;
    if (y > y0) out[n++] = idx - g->w;
    if (y + 1 < y1) out[n++] = idx + g->w;
    return n;
}

/* Can the flood enter a cell that currently holds label `lab`?
     FLOOD_ALL:   any unlabelled walkable cell (full rebuild).
     FLOOD_MERGE: an unlabelled cell (label 0) or a cell of a region already
                  marked inactive, i.e. one being folded into the new region.
     FLOOD_OLD:   a cell still carrying the pre-split region id `ref`.
   Blocked cells never pass. */
static int flood_accept(const RegionGrid *g, int idx, int mode, int ref) {
    if (g->blocked[idx]) return 0;
    int lab = g->label[idx];
    switch (mode) {
    case FLOOD_ALL:   return lab == 0;
    case FLOOD_MERGE: return lab == 0 || (lab > 0 && !g->active[lab]);
    case FLOOD_OLD:   return lab == ref;
    }
    return 0;
}

/* Flood from `start` over in-chunk cells accepted by the mode, marking them
   `mark` and returning the component size.  g->queue holds the component on
   return.  `start` itself must be acceptable.

   `ref` is the old region id for FLOOD_OLD.  `mark` must be negative and
   unique within one operation: a split assigns each piece a distinct marker
   so that a piece which keeps the old region's minimum (and therefore its
   old id) is never mistaken for an unprocessed piece. */
static int flood(RegionGrid *g, int start, int x0, int y0, int x1, int y1,
                 int mode, int ref, int mark) {
    int *q = g->queue;
    int head = 0, tail = 0;
    q[tail++] = start;
    g->label[start] = mark;
    while (head < tail) {
        int cur = q[head++];
        int x = cur % g->w, y = cur / g->w;
        int nb[4], nn = 0;
        if (x > x0) nb[nn++] = cur - 1;
        if (x + 1 < x1) nb[nn++] = cur + 1;
        if (y > y0) nb[nn++] = cur - g->w;
        if (y + 1 < y1) nb[nn++] = cur + g->w;
        for (int i = 0; i < nn; i++) {
            int nidx = nb[i];
            if (flood_accept(g, nidx, mode, ref)) {
                g->label[nidx] = mark;
                q[tail++] = nidx;
            }
        }
    }
    return tail;
}

/* Give the queued component (negative marker) its canonical id and metadata. */
static int commit_component(RegionGrid *g, int count) {
    int *q = g->queue;
    int minidx = q[0];
    int minx = q[0] % g->w, maxx = minx;
    int miny = q[0] / g->w, maxy = miny;
    for (int i = 1; i < count; i++) {
        int idx = q[i];
        if (idx < minidx) minidx = idx;
        int x = idx % g->w, y = idx / g->w;
        if (x < minx) minx = x;
        if (x > maxx) maxx = x;
        if (y < miny) miny = y;
        if (y > maxy) maxy = y;
    }
    int id = minidx + 1;
    for (int i = 0; i < count; i++) g->label[q[i]] = id;
    RegionMeta *m = &g->meta[id];
    m->cells = count;
    m->minx = minx; m->miny = miny; m->maxx = maxx; m->maxy = maxy;
    g->active[id] = 1;
    return id;
}

int rg_init(RegionGrid *g, int w, int h, int chunk) {
    if (!g) return -1;
    memset(g, 0, sizeof(*g));
    if (w <= 0 || h <= 0 || chunk <= 0) return -1;
    if (w > INT_MAX / h) return -1;
    int n = w * h;
    g->w = w; g->h = h; g->n = n; g->chunk = chunk;
    g->cw = (w + chunk - 1) / chunk;
    g->ch = (h + chunk - 1) / chunk;
    g->blocked = (unsigned char *)calloc((size_t)n, 1);
    g->label = (int *)calloc((size_t)n, sizeof(int));
    g->active = (unsigned char *)calloc((size_t)n + 1, 1);
    g->meta = (RegionMeta *)calloc((size_t)n + 1, sizeof(RegionMeta));
    g->queue = (int *)malloc((size_t)n * sizeof(int));
    if (!g->blocked || !g->label || !g->active || !g->meta || !g->queue) {
        rg_free(g);
        return -1;
    }
    return 0;
}

void rg_free(RegionGrid *g) {
    if (!g) return;
    free(g->blocked); free(g->label); free(g->active);
    free(g->meta); free(g->queue); free(g->visit);
    memset(g, 0, sizeof(*g));
}

int rg_build(RegionGrid *g) {
    if (!g || !g->label || !g->queue) return -1;
    memset(g->label, 0, (size_t)g->n * sizeof(int));
    memset(g->active, 0, (size_t)g->n + 1);
    g->region_count = 0;
    g->last_relabel_cells = 0;
    for (int cy = 0; cy < g->ch; cy++) {
        int y0 = cy * g->chunk;
        int y1 = y0 + g->chunk; if (y1 > g->h) y1 = g->h;
        for (int cx = 0; cx < g->cw; cx++) {
            int x0 = cx * g->chunk;
            int x1 = x0 + g->chunk; if (x1 > g->w) x1 = g->w;
            for (int y = y0; y < y1; y++) {
                for (int x = x0; x < x1; x++) {
                    int idx = y * g->w + x;
                    if (g->blocked[idx] || g->label[idx] != 0) continue;
                    /* The seed is the component's minimum index (row-major
                       within the chunk), so the id is canonical. */
                    int count = flood(g, idx, x0, y0, x1, y1, FLOOD_ALL, 0, -1);
                    commit_component(g, count);
                    g->region_count++;
                    g->last_relabel_cells += count;
                }
            }
        }
    }
    g->built = 1;
    return g->region_count;
}

int rg_toggle(RegionGrid *g, int x, int y) {
    if (!g || !g->label || !g->built) return -1;
    if (x < 0 || y < 0 || x >= g->w || y >= g->h) return -1;
    int idx = y * g->w + x;
    int x0, y0, x1, y1;
    chunk_bounds(g, x, y, &x0, &y0, &x1, &y1);
    int nb[4];
    int nn = neighbours(g, idx, nb);

    if (g->blocked[idx]) {
        /* Open.  The cell joins every distinct region around it into one. */
        int merged[4], m = 0;
        for (int i = 0; i < nn; i++) {
            int id = g->label[nb[i]];
            if (id <= 0) continue;
            int dup = 0;
            for (int j = 0; j < m; j++) if (merged[j] == id) { dup = 1; break; }
            if (!dup) merged[m++] = id;
        }
        for (int i = 0; i < m; i++) g->active[merged[i]] = 0;
        g->blocked[idx] = 0;
        g->last_relabel_cells = 0;
        /* The cell is unlabelled, merged regions are inactive, and any other
           region is 4-separated from it, so FLOOD_MERGE collects exactly
           idx plus the merged regions. */
        int count = flood(g, idx, x0, y0, x1, y1, FLOOD_MERGE, 0, -1);
        commit_component(g, count);
        g->last_relabel_cells += count;
        g->region_count -= (m - 1);      /* m regions + new cell -> 1 */
    } else {
        /* Close.  The old region can split into pieces; every piece keeps a
           cell adjacent to idx, because any path between two pieces in the
           old region had to pass through idx.  Flood each piece under a
           distinct negative marker, then commit all pieces together: doing
           it in that order keeps a piece that reuses the old canonical id
           from being mistaken for an unprocessed piece by a later flood. */
        int old = g->label[idx];
        if (old <= 0) return -1;         /* label/blocked inconsistency */
        g->blocked[idx] = 1;
        g->label[idx] = 0;
        g->active[old] = 0;
        g->last_relabel_cells = 1;       /* the closed cell */

        int mark[4], id[4], count[4], pminx[4], pminy[4], pmaxx[4], pmaxy[4];
        int pieces = 0;
        for (int i = 0; i < nn; i++) {
            if (g->label[nb[i]] != old) continue;  /* blocked, other region,
                                                      or an earlier piece */
            int m = -(pieces + 2);
            int c = flood(g, nb[i], x0, y0, x1, y1, FLOOD_OLD, old, m);
            int mn = g->queue[0];
            int mnx = mn % g->w, mxx = mnx, mny = mn / g->w, mxy = mny;
            for (int k = 1; k < c; k++) {
                int cell = g->queue[k];
                if (cell < mn) mn = cell;
                int cx = cell % g->w, cy = cell / g->w;
                if (cx < mnx) mnx = cx;
                if (cx > mxx) mxx = cx;
                if (cy < mny) mny = cy;
                if (cy > mxy) mxy = cy;
            }
            mark[pieces] = m; id[pieces] = mn + 1; count[pieces] = c;
            pminx[pieces] = mnx; pminy[pieces] = mny;
            pmaxx[pieces] = mxx; pmaxy[pieces] = mxy;
            pieces++;
            g->last_relabel_cells += c;
        }
        for (int i = 0; i < pieces; i++) {
            RegionMeta *meta = &g->meta[id[i]];
            meta->cells = count[i];
            meta->minx = pminx[i]; meta->miny = pminy[i];
            meta->maxx = pmaxx[i]; meta->maxy = pmaxy[i];
            g->active[id[i]] = 1;
        }
        /* Rewrite the marked cells to their canonical ids.  Only the chunk
           that contains idx is scanned, and only marked cells are written;
           every other region in the chunk keeps its label untouched. */
        for (int y = y0; y < y1; y++) {
            for (int x = x0; x < x1; x++) {
                int cell = y * g->w + x;
                int lab = g->label[cell];
                for (int i = 0; i < pieces; i++) {
                    if (lab == mark[i]) { g->label[cell] = id[i]; break; }
                }
            }
        }
        g->region_count += (pieces - 1); /* one region -> `pieces` pieces */
    }
    return g->region_count;
}

int rg_region_at(const RegionGrid *g, int x, int y) {
    if (!g || !g->label || !g->built) return 0;
    if (x < 0 || y < 0 || x >= g->w || y >= g->h) return 0;
    if (g->blocked[y * g->w + x]) return 0;
    int id = g->label[y * g->w + x];
    return id > 0 ? id : 0;
}

void rg_stats(RegionGrid *g, RgStats *out) {
    memset(out, 0, sizeof(*out));
    if (!g || !g->label) return;
    out->chunks = g->cw * g->ch;
    int *occ = (int *)calloc((size_t)out->chunks, sizeof(int));
    int *per_chunk = (int *)calloc((size_t)out->chunks, sizeof(int));
    if (!occ || !per_chunk) { free(occ); free(per_chunk); return; }
    if (g->visit == NULL) {
        g->visit = (int *)calloc((size_t)g->n + 1, sizeof(int));
        g->visit_gen = 0;
    }
    int have_visit = g->visit != NULL;
    if (have_visit && ++g->visit_gen == 0) {
        memset(g->visit, 0, ((size_t)g->n + 1) * sizeof(int));
        g->visit_gen = 1;
    }
    for (int id = 1; id <= g->n; id++) {
        if (!g->active[id]) continue;
        out->regions++;
        if (g->meta[id].cells > out->max_region_cells) out->max_region_cells = g->meta[id].cells;
        out->region_cells += g->meta[id].cells;
    }
    for (int cy = 0; cy < g->ch; cy++) {
        int y0 = cy * g->chunk, y1 = y0 + g->chunk; if (y1 > g->h) y1 = g->h;
        for (int cx = 0; cx < g->cw; cx++) {
            int x0 = cx * g->chunk, x1 = x0 + g->chunk; if (x1 > g->w) x1 = g->w;
            int ch = cy * g->cw + cx;
            for (int y = y0; y < y1; y++) {
                for (int x = x0; x < x1; x++) {
                    int idx = y * g->w + x;
                    int id = g->label[idx];
                    if (g->blocked[idx] || id <= 0) continue;
                    occ[ch] = 1;
                    if (have_visit && g->visit[id] != (int)g->visit_gen) {
                        g->visit[id] = (int)g->visit_gen;
                        per_chunk[ch]++;
                    }
                }
            }
            if (per_chunk[ch] > out->max_regions_per_chunk)
                out->max_regions_per_chunk = per_chunk[ch];
        }
    }
    for (int i = 0; i < out->chunks; i++) out->occupied_chunks += occ[i] != 0;
    free(occ); free(per_chunk);
}

int rg_chunk_regions(RegionGrid *g, int cx, int cy, int *out, int max_out) {
    if (!g || !g->label || !g->built || !out || max_out <= 0) return -1;
    if (cx < 0 || cy < 0 || cx >= g->cw || cy >= g->ch) return -1;
    if (g->visit == NULL) {
        g->visit = (int *)calloc((size_t)g->n + 1, sizeof(int));
        if (!g->visit) return -1;
        g->visit_gen = 0;
    }
    if (++g->visit_gen == 0) {
        memset(g->visit, 0, ((size_t)g->n + 1) * sizeof(int));
        g->visit_gen = 1;
    }
    int x0 = cx * g->chunk, y0 = cy * g->chunk;
    int x1 = x0 + g->chunk; if (x1 > g->w) x1 = g->w;
    int y1 = y0 + g->chunk; if (y1 > g->h) y1 = g->h;
    int count = 0;
    for (int y = y0; y < y1; y++) {
        for (int x = x0; x < x1; x++) {
            int idx = y * g->w + x;
            int id = g->label[idx];
            if (g->blocked[idx] || id <= 0) continue;
            if (g->visit[id] == (int)g->visit_gen) continue;
            g->visit[id] = (int)g->visit_gen;
            if (count < max_out) out[count] = id;
            count++;
        }
    }
    return count > max_out ? -1 : count;
}

int rg_validate(const RegionGrid *g) {
    if (!g || !g->label || !g->built) { fprintf(stderr, "validate: not built\n"); return 0; }
    int n = g->n, ok = 1;
    int *min_idx = (int *)calloc((size_t)n + 1, sizeof(int)); /* min+1, 0 = unset */
    int *cnt = (int *)calloc((size_t)n + 1, sizeof(int));
    int *tminx = (int *)malloc(((size_t)n + 1) * sizeof(int));
    int *tminy = (int *)malloc(((size_t)n + 1) * sizeof(int));
    int *tmaxx = (int *)malloc(((size_t)n + 1) * sizeof(int));
    int *tmaxy = (int *)malloc(((size_t)n + 1) * sizeof(int));
    if (!min_idx || !cnt || !tminx || !tminy || !tmaxx || !tmaxy) ok = 0;
    for (int i = 0; ok && i <= n; i++) {
        tminx[i] = INT_MAX; tminy[i] = INT_MAX;
        tmaxx[i] = INT_MIN; tmaxy[i] = INT_MIN;
    }
    for (int y = 0; ok && y < g->h; y++) {
        for (int x = 0; ok && x < g->w; x++) {
            int idx = y * g->w + x, id = g->label[idx];
            if (g->blocked[idx]) {
                if (id != 0) { fprintf(stderr, "validate: blocked (%d,%d) labelled %d\n", x, y, id); ok = 0; break; }
                continue;
            }
            if (id <= 0 || id > n) { fprintf(stderr, "validate: walkable (%d,%d) bad id %d\n", x, y, id); ok = 0; break; }
            if (min_idx[id] == 0 || idx + 1 < min_idx[id]) min_idx[id] = idx + 1;
            cnt[id]++;
            if (x < tminx[id]) tminx[id] = x;
            if (x > tmaxx[id]) tmaxx[id] = x;
            if (y < tminy[id]) tminy[id] = y;
            if (y > tmaxy[id]) tmaxy[id] = y;
        }
    }
    int used = 0;
    for (int id = 1; ok && id <= n; id++) {
        if (cnt[id] == 0) {
            if (g->active[id]) { fprintf(stderr, "validate: active id %d empty\n", id); ok = 0; }
            continue;
        }
        used++;
        if (!g->active[id]) { fprintf(stderr, "validate: id %d in use but inactive\n", id); ok = 0; break; }
        if (min_idx[id] != id) { fprintf(stderr, "validate: id %d not canonical (min+1=%d)\n", id, min_idx[id]); ok = 0; break; }
        RegionMeta *m = &g->meta[id];
        if (m->cells != cnt[id] || m->minx != tminx[id] || m->miny != tminy[id] ||
            m->maxx != tmaxx[id] || m->maxy != tmaxy[id]) {
            fprintf(stderr, "validate: id %d metadata mismatch\n", id);
            ok = 0; break;
        }
    }
    if (ok && used != g->region_count) {
        fprintf(stderr, "validate: region_count %d != used %d\n", g->region_count, used);
        ok = 0;
    }
    /* Maximality inside a chunk; no region may cross a chunk edge. */
    for (int y = 0; ok && y < g->h; y++) {
        for (int x = 0; ok && x < g->w; x++) {
            int idx = y * g->w + x;
            if (g->blocked[idx]) continue;
            int id = g->label[idx];
            int cx = x / g->chunk, cy = y / g->chunk;
            const int dx[2] = {1, 0}, dy[2] = {0, 1};
            for (int d = 0; d < 2; d++) {
                int nx = x + dx[d], ny = y + dy[d];
                if (nx >= g->w || ny >= g->h) continue;
                int nidx = ny * g->w + nx;
                if (g->blocked[nidx]) continue;
                int same_chunk = (nx / g->chunk == cx) && (ny / g->chunk == cy);
                if (same_chunk && g->label[nidx] != id) {
                    fprintf(stderr, "validate: chunk-split edge (%d,%d)-(%d,%d)\n", x, y, nx, ny);
                    ok = 0; break;
                }
                if (!same_chunk && g->label[nidx] == id) {
                    fprintf(stderr, "validate: region %d crosses chunk at (%d,%d)\n", id, x, y);
                    ok = 0; break;
                }
            }
            if (!ok) break;
        }
    }
    free(min_idx); free(cnt); free(tminx); free(tminy); free(tmaxx); free(tmaxy);
    return ok;
}
