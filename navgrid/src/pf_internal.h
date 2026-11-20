/* pf_internal.h - shared machinery for the pathfinding suite. */
#ifndef PF_INTERNAL_H
#define PF_INTERNAL_H

#include "pf.h"

#define PF_INF 1e30
#define PF_EPS 1e-9
#define PF_DIR_COUNT 8
#define HPA_CLUSTER 8
#define HPA_CLUSTER_CELLS (HPA_CLUSTER * HPA_CLUSTER)

extern const int PF_DX[PF_DIR_COUNT];
extern const int PF_DY[PF_DIR_COUNT];

/* Binary min-heap keyed by (k1, k2, idx). Entries carry a version so callers
 * can use lazy deletion (LPA* and HPA* both need it). */
typedef struct { double k1, k2; int idx; unsigned ver; } PfHeapNode;
typedef struct { PfHeapNode* a; int n, cap; } PfHeap;

void pf_heap_init(PfHeap* h);
void pf_heap_free(PfHeap* h);
void pf_heap_clear(PfHeap* h);
int  pf_heap_push(PfHeap* h, PfHeapNode v);
int  pf_heap_pop(PfHeap* h, PfHeapNode* out);
int  pf_heap_peek(PfHeap* h, PfHeapNode* out);
void pf_heap_compact(PfHeap* h, const unsigned* ver);

typedef struct {
    double* g;
    double* rhs;
    unsigned* ver;
    PfHeap heap;
    int n, start, goal;
    int valid;
} PfLpa;

typedef struct { int to; float cost; } PfHEdge;
typedef struct { PfHEdge* e; int n, cap; } PfHAdj;

typedef struct {
    int valid;
    int border_count;
    int* cell_node;             /* w*h -> abstract node id or -1 */
    int* nx;                    /* border cell coordinates */
    int* ny;
    int* ncl;                   /* cluster id of each border node */
    PfHAdj* adj;                /* one list per abstract (border) node */
    int adj_count;              /* number of lists in adj */
    int* cl_start;              /* cluster id -> slice in cl_nodes */
    int* cl_len;
    int* cl_nodes;              /* border node ids grouped by cluster */
    int cluster_cols, cluster_rows, cluster_count;
} PfHpa;

struct PfSolver {
    PfGrid grid;
    int changes_expected;
    int changes;
    int* dirty;                 /* cells edited since the last LPA* plan */
    int ndirty, dirty_cap;
    PfProfile profile;
    int profile_valid;
    PfAlgo auto_algo;
    PfLpa lpa;
    PfHpa hpa;
};

void pf__grid_copy(PfGrid* dst, const PfGrid* src);
double pf__octile(PfPoint a, PfPoint b);
int pf__cluster_of(const PfGrid* g, int x, int y);
double pf__step_cost(int dx, int dy);

/* Canonical optimal-path reconstruction from settled g values.
 * Both A* and LPA* use it, so whenever both are optimal they emit exactly the
 * same point sequence instead of whichever parent pointers their queues
 * happened to settle on. `get` reads g for a cell index (A* keeps its values
 * inside node structs, LPA* in a flat array). Returns cell count, or -1. */
typedef double (*PfGvalFn)(const void* ctx, int idx);
int pf__descend_optimal(const PfGrid* g, PfGvalFn get, const void* ctx,
                        int start, int goal, PfPoint* out, int cap);

int pf__astar(const PfGrid* g, PfPoint s, PfPoint t, int theta,
              PfPath* out, long* expansions);
void pf__lpa_init(PfSolver* s);
void pf__lpa_free(PfSolver* s);
int  pf__lpa_solve(PfSolver* s, PfPoint start, PfPoint goal, PfPath* out);
int  pf__hpa_solve(PfSolver* s, PfPoint start, PfPoint goal, PfPath* out);
void pf__hpa_free(PfSolver* s);
void pf__hpa_invalidate(PfSolver* s);

#endif
