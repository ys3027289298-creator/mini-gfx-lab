
/* vcache-opt: post-transform vertex-cache optimization.
   Reorders triangles so that vertex references stay inside a simulated LRU
   post-transform vertex cache, using Forsyth-style vertex/triangle scoring
   with Tipsify's linear-time candidate rule (only the next live triangle of
   each cached vertex is considered). Only triangle order changes: every
   triangle keeps its original three indices in its original order, so winding
   and vertex data are untouched. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define VCACHE_K 16                /* simulated cache size, vertices */
#define LAST_TRI_SCORE 0.75f       /* vertices of the just-emitted triangle */
#define VALENCE_SCALE 2.0f         /* Forsyth valence boost constants */
#define VALENCE_POWER 0.5f
#define DECAY_POWER 1.5f

typedef struct {
    int hits;
    int misses;
} CacheStat;

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e3 + (double)ts.tv_nsec / 1e6;
}

/* Simulate an LRU post-transform vertex cache of k entries over an index
   buffer. Each vertex reference is a hit when already cached, otherwise a
   miss; a hit moves the vertex to the MRU front, and an insertion evicts the
   LRU tail. Hit rate = hits / (3 * ntris). */
static CacheStat vcache_simulate(const unsigned int *idx, int ntris, int k, int nverts) {
    int *pos = (int *)malloc((size_t)nverts * sizeof(int));
    unsigned int *cache = (unsigned int *)malloc((size_t)k * sizeof(unsigned int));
    if (!pos || !cache) { free(pos); free(cache); exit(1); }
    for (int v = 0; v < nverts; v++) pos[v] = -1;

    CacheStat st = {0, 0};
    int used = 0;
    for (int t = 0; t < ntris; t++) {
        for (int j = 0; j < 3; j++) {
            int v = (int)idx[3 * t + j];
            int p = pos[v];
            if (p >= 0) {
                st.hits++;                       /* move to MRU front */
                for (int q = p; q > 0; q--) {
                    cache[q] = cache[q - 1];
                    pos[cache[q]] = q;
                }
                cache[0] = (unsigned int)v;
                pos[v] = 0;
            } else {
                st.misses++;
                if (used == k) { pos[cache[k - 1]] = -1; used--; }
                for (int q = used; q > 0; q--) {
                    cache[q] = cache[q - 1];
                    pos[cache[q]] = q;
                }
                cache[0] = (unsigned int)v;
                pos[v] = 0;
                used++;
            }
        }
    }
    free(cache);
    free(pos);
    return st;
}

/* Reorder ntris triangles from idx into out (both 3*ntris index arrays).
   out contains the same triangles as idx, only permuted; each triangle is
   copied verbatim, so winding is preserved. O(VCACHE_K * ntris) time. */
static void vcache_optimize(unsigned int *out, const unsigned int *idx,
                            int ntris, int nverts) {
    if (ntris <= 0) return;

    /* CSR adjacency: degree per vertex, prefix offsets, packed triangle ids.
       Repeated indices inside one triangle count once. */
    unsigned int *degree = (unsigned int *)calloc((size_t)nverts, sizeof(unsigned int));
    unsigned int *off = (unsigned int *)malloc(((size_t)nverts + 1) * sizeof(unsigned int));
    unsigned int *adj = (unsigned int *)malloc((size_t)(3 * ntris) * sizeof(unsigned int));
    unsigned int *fill = (unsigned int *)malloc((size_t)nverts * sizeof(unsigned int));
    unsigned int *cur = (unsigned int *)malloc((size_t)nverts * sizeof(unsigned int));
    unsigned int *valence = (unsigned int *)malloc((size_t)nverts * sizeof(unsigned int));
    float *vboost = (float *)malloc((size_t)nverts * sizeof(float));
    short *cachepos = (short *)malloc((size_t)nverts * sizeof(short));
    unsigned char *emitted = (unsigned char *)calloc((size_t)ntris, 1);
    unsigned int *tag = (unsigned int *)calloc((size_t)ntris, sizeof(unsigned int));
    if (!degree || !off || !adj || !fill || !cur || !valence || !vboost ||
        !cachepos || !emitted || !tag) exit(1);

    for (int t = 0; t < ntris; t++) {
        for (int j = 0; j < 3; j++) {
            int v = (int)idx[3 * t + j];
            int dup = 0;
            for (int q = 0; q < j; q++)
                if (idx[3 * t + q] == idx[3 * t + j]) dup = 1;
            if (!dup) degree[v]++;
        }
    }
    off[0] = 0;
    for (int v = 0; v < nverts; v++) off[v + 1] = off[v] + degree[v];
    memcpy(fill, off, (size_t)nverts * sizeof(unsigned int));
    for (int t = 0; t < ntris; t++) {
        for (int j = 0; j < 3; j++) {
            int v = (int)idx[3 * t + j];
            int dup = 0;
            for (int q = 0; q < j; q++)
                if (idx[3 * t + q] == idx[3 * t + j]) dup = 1;
            if (!dup) adj[fill[v]++] = (unsigned int)t;
        }
    }
    memcpy(cur, off, (size_t)nverts * sizeof(unsigned int));
    memcpy(valence, degree, (size_t)nverts * sizeof(unsigned int));
    for (int v = 0; v < nverts; v++) {
        cachepos[v] = -1;
        vboost[v] = valence[v]
            ? (float)(VALENCE_SCALE / pow((double)valence[v], VALENCE_POWER)) : 0.0f;
    }

    /* Cache-position score: positions 0..2 hold the last triangle's verts. */
    float pos_score[VCACHE_K];
    for (int i = 0; i < VCACHE_K; i++) {
        pos_score[i] = i < 3
            ? LAST_TRI_SCORE
            : (float)pow((double)(i - 2), -DECAY_POWER);
    }

    unsigned int cache[VCACHE_K];
    int warm = 0;                  /* live entries in the LRU cache */
    int gcur = 0;                  /* dead-end cursor, only advances */
    unsigned int stamp = 0;

    for (int done = 0; done < ntris; done++) {
        int best = -1;
        float best_score = -1.0f;
        stamp++;

        /* Tipsify candidate rule: among cached vertices, look only at each
           vertex's next live triangle. Cursors only move forward, so the
           total adjacency scan over the whole run is O(3 * ntris). */
        for (int ci = 0; ci < warm; ci++) {
            int v = (int)cache[ci];
            unsigned int end = off[v + 1];
            while (cur[v] < end && emitted[adj[cur[v]]]) cur[v]++;
            if (cur[v] == end) continue;
            int t = (int)adj[cur[v]];
            if (tag[t] == stamp) continue;
            tag[t] = stamp;

            float score = 0.0f;
            for (int j = 0; j < 3; j++) {
                int u = (int)idx[3 * t + j];
                score += (cachepos[u] >= 0 ? pos_score[cachepos[u]] : 0.0f)
                       + vboost[u];
            }
            if (score > best_score) { best_score = score; best = t; }
        }

        if (best < 0) {
            /* True dead end: no cached vertex has a live triangle. Grab any
               remaining triangle via the monotonic global cursor. */
            while (emitted[gcur]) gcur++;
            best = gcur;
            for (int ci = 0; ci < warm; ci++) cachepos[cache[ci]] = -1;
            warm = 0;
        }

        /* Emit best: pull its distinct vertices out of the cache, then insert
           them at positions 0..2 (MRU), evicting the LRU tail if full. */
        int seen[3], distinct = 0;
        for (int j = 0; j < 3; j++) {
            int v = (int)idx[3 * best + j];
            int dup = 0;
            for (int q = 0; q < distinct; q++)
                if (seen[q] == v) dup = 1;
            if (!dup) seen[distinct++] = v;
        }
        for (int j = 0; j < distinct; j++) {
            int v = seen[j];
            int p = cachepos[v];
            if (p >= 0) {
                for (int q = p; q < warm - 1; q++) {
                    cache[q] = cache[q + 1];
                    cachepos[cache[q]] = (short)q;
                }
                warm--;
                cachepos[v] = -1;
            }
        }
        int drop = warm + distinct - VCACHE_K;
        if (drop > 0) {
            for (int q = 0; q < drop; q++) cachepos[cache[warm - 1 - q]] = -1;
            warm -= drop;
        }
        for (int q = warm - 1; q >= 0; q--) {
            cache[q + distinct] = cache[q];
            cachepos[cache[q + distinct]] = (short)(q + distinct);
        }
        for (int j = 0; j < distinct; j++) {
            cache[j] = (unsigned int)seen[j];
            cachepos[seen[j]] = (short)j;
        }
        warm += distinct;

        for (int j = 0; j < distinct; j++) {
            int v = seen[j];
            valence[v]--;
            vboost[v] = valence[v]
                ? (float)(VALENCE_SCALE / pow((double)valence[v], VALENCE_POWER))
                : 0.0f;
        }
        emitted[best] = 1;

        /* Verbatim copy: original within-triangle index order is preserved. */
        out[3 * done + 0] = idx[3 * best + 0];
        out[3 * done + 1] = idx[3 * best + 1];
        out[3 * done + 2] = idx[3 * best + 2];
    }

    free(tag);
    free(emitted);
    free(cachepos);
    free(vboost);
    free(valence);
    free(cur);
    free(fill);
    free(adj);
    free(off);
    free(degree);
}

/* ---- test harness ---- */

typedef struct {
    unsigned int a, b, c;
    int id;
} Tri;

static int cmp_tri(const void *p, const void *q) {
    const Tri *x = (const Tri *)p, *y = (const Tri *)q;
    if (x->a != y->a) return x->a < y->a ? -1 : 1;
    if (x->b != y->b) return x->b < y->b ? -1 : 1;
    if (x->c != y->c) return x->c < y->c ? -1 : 1;
    return 0;
}

/* Verify out is a permutation of idx and every triangle kept its winding. */
static int check_perm_winding(const unsigned int *idx, const unsigned int *out,
                              int ntris) {
    Tri *a = (Tri *)malloc((size_t)ntris * sizeof(Tri));
    Tri *b = (Tri *)malloc((size_t)ntris * sizeof(Tri));
    if (!a || !b) exit(1);
    for (int t = 0; t < ntris; t++) {
        a[t].a = idx[3 * t]; a[t].b = idx[3 * t + 1]; a[t].c = idx[3 * t + 2];
        a[t].id = t;
        b[t].a = out[3 * t]; b[t].b = out[3 * t + 1]; b[t].c = out[3 * t + 2];
        b[t].id = t;
    }
    qsort(a, (size_t)ntris, sizeof(Tri), cmp_tri);
    qsort(b, (size_t)ntris, sizeof(Tri), cmp_tri);
    int ok = 1;
    for (int t = 0; t < ntris; t++) {
        if (a[t].a != b[t].a || a[t].b != b[t].b || a[t].c != b[t].c) { ok = 0; break; }
        /* componentwise equality on the sorted triple proves winding survived */
    }
    free(a);
    free(b);
    return ok;
}

static unsigned int *build_grid(int w, int h, int *ntris, int *nverts) {
    *ntris = 2 * w * h;
    *nverts = (w + 1) * (h + 1);
    unsigned int *idx = (unsigned int *)malloc((size_t)(3 * *ntris) * sizeof(unsigned int));
    if (!idx) exit(1);
    int n = 0;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            unsigned int v00 = (unsigned int)(y * (w + 1) + x);
            unsigned int v10 = v00 + 1;
            unsigned int v01 = v00 + (unsigned int)(w + 1);
            unsigned int v11 = v01 + 1;
            idx[n++] = v00; idx[n++] = v10; idx[n++] = v11;   /* CCW */
            idx[n++] = v00; idx[n++] = v11; idx[n++] = v01;
        }
    }
    return idx;
}

static void shuffle_tris(unsigned int *idx, int ntris, unsigned int seed) {
    for (int t = ntris - 1; t > 0; t--) {
        seed = seed * 1103515245u + 12345u;
        int j = (int)((seed >> 8) % (unsigned int)(t + 1));
        for (int q = 0; q < 3; q++) {
            unsigned int tmp = idx[3 * t + q];
            idx[3 * t + q] = idx[3 * j + q];
            idx[3 * j + q] = tmp;
        }
    }
}

static void print_rate(const char *name, const unsigned int *before,
                       const unsigned int *after, int ntris, int nverts, int k) {
    CacheStat s0 = vcache_simulate(before, ntris, k, nverts);
    CacheStat s1 = vcache_simulate(after, ntris, k, nverts);
    int refs = 3 * ntris;
    double h0 = 100.0 * s0.hits / refs, h1 = 100.0 * s1.hits / refs;
    printf("%-10s K=%-2d  before: hit=%6.2f%% ACMR=%5.3f | after: hit=%6.2f%% "
           "ACMR=%5.3f | delta=%+6.2f pts\n",
           name, k, h0, (double)s0.misses / ntris,
           h1, (double)s1.misses / ntris, h1 - h0);
}

int main(void) {
    /* ---- tiny hand-checkable case: two triangles sharing an edge ---- */
    {
        unsigned int idx[6] = {0, 1, 2, 1, 3, 2};
        unsigned int out[6];
        vcache_optimize(out, idx, 2, 4);
        int ok = check_perm_winding(idx, out, 2);
        CacheStat st = vcache_simulate(out, 2, VCACHE_K, 4);
        /* tri 0: 3 misses; tri 1 reuses verts 1 and 2 -> exactly 2 hits */
        printf("tiny: perm_winding_ok=%d hits=%d (expect 2)\n", ok, st.hits);
        if (!ok || st.hits != 2) return 1;
    }

    /* ---- 1M-triangle grid benchmark ---- */
    int w = 1000, h = 500, ntris, nverts;
    unsigned int *grid = build_grid(w, h, &ntris, &nverts);
    unsigned int *shuffled = (unsigned int *)malloc((size_t)(3 * ntris) * sizeof(unsigned int));
    unsigned int *opt_grid = (unsigned int *)malloc((size_t)(3 * ntris) * sizeof(unsigned int));
    unsigned int *opt_shuf = (unsigned int *)malloc((size_t)(3 * ntris) * sizeof(unsigned int));
    if (!shuffled || !opt_grid || !opt_shuf) exit(1);
    memcpy(shuffled, grid, (size_t)(3 * ntris) * sizeof(unsigned int));
    shuffle_tris(shuffled, ntris, 42u);

    printf("grid %dx%d: ntris=%d nverts=%d (refs=%d)\n", w, h, ntris, nverts, 3 * ntris);

    double t0 = now_ms();
    vcache_optimize(opt_grid, grid, ntris, nverts);
    double t1 = now_ms();
    vcache_optimize(opt_shuf, shuffled, ntris, nverts);
    double t2 = now_ms();

    int perm1 = check_perm_winding(grid, opt_grid, ntris);
    int perm2 = check_perm_winding(shuffled, opt_shuf, ntris);
    printf("perm_winding_ok=%d,%d\n", perm1, perm2);

    print_rate("row-major", grid, opt_grid, ntris, nverts, VCACHE_K);
    print_rate("shuffled", shuffled, opt_shuf, ntris, nverts, VCACHE_K);

    CacheStat gb = vcache_simulate(grid, ntris, VCACHE_K, nverts);
    CacheStat ga = vcache_simulate(opt_grid, ntris, VCACHE_K, nverts);
    CacheStat sb = vcache_simulate(shuffled, ntris, VCACHE_K, nverts);
    CacheStat sa = vcache_simulate(opt_shuf, ntris, VCACHE_K, nverts);
    int improved = ga.hits > gb.hits && sa.hits > sb.hits;
    double after_pct = 100.0 * sa.hits / (3.0 * ntris);
    printf("improvement_ok=%d optimized_hitrate=%.2f%%\n", improved, after_pct);
    if (!perm1 || !perm2 || !improved || after_pct < 70.0) return 1;

    printf("optimize_ms=%.1f (row-major), %.1f (shuffled), budget=2000\n",
           t1 - t0, t2 - t1);
    if (t1 - t0 >= 2000.0 || t2 - t1 >= 2000.0) return 1;

    /* ---- same reordered stream measured under other cache sizes ---- */
    printf("robustness (shuffled stream, hit%%):\n");
    int ks[] = {8, 12, 16, 24, 32};
    for (size_t i = 0; i < sizeof(ks) / sizeof(ks[0]); i++) {
        CacheStat b = vcache_simulate(shuffled, ntris, ks[i], nverts);
        CacheStat a = vcache_simulate(opt_shuf, ntris, ks[i], nverts);
        printf("  K=%-2d before=%6.2f%% after=%6.2f%% delta=%+6.2f pts\n",
               ks[i], 100.0 * b.hits / (3.0 * ntris),
               100.0 * a.hits / (3.0 * ntris),
               100.0 * (a.hits - b.hits) / (3.0 * ntris));
    }

    /* ---- adversarial fan: one vertex referenced by every triangle ---- */
    {
        int fn = 200000;
        unsigned int *fidx = (unsigned int *)malloc((size_t)(3 * fn) * sizeof(unsigned int));
        unsigned int *fout = (unsigned int *)malloc((size_t)(3 * fn) * sizeof(unsigned int));
        if (!fidx || !fout) exit(1);
        for (int t = 0; t < fn; t++) {
            fidx[3 * t] = 0;
            fidx[3 * t + 1] = (unsigned int)(t + 1);
            fidx[3 * t + 2] = (unsigned int)(t + 2);
        }
        double f0 = now_ms();
        vcache_optimize(fout, fidx, fn, fn + 2);
        double f1 = now_ms();
        int fok = check_perm_winding(fidx, fout, fn);
        printf("fan ntris=%d perm_winding_ok=%d optimize_ms=%.1f (linearity guard)\n",
               fn, fok, f1 - f0);
        if (!fok || f1 - f0 >= 2000.0) return 1;
        free(fidx);
        free(fout);
    }

    free(opt_shuf);
    free(opt_grid);
    free(shuffled);
    free(grid);
    printf("ALL OK\n");
    return 0;
}
