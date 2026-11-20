/* pf.h - pathfinding suite public interface.
 *
 * Four planners share one entry point, pf_solve():
 *   PF_ASTAR        8-directional A*, no corner cutting
 *   PF_ANYANGLE     Theta* (A* with line-of-sight parent relaxation)
 *   PF_INCREMENTAL  LPA* (reuses the previous search across map edits)
 *   PF_HIERARCHICAL HPA* (cluster abstraction, 8x8 clusters)
 *
 * pf_select_algo() classifies the map and picks a planner; PF_AUTO uses it.
 */
#ifndef PF_H
#define PF_H

#include <stddef.h>

#define PF_MAX_DIM 256
#define PF_OK 0
#define PF_ENOPATH 1
#define PF_EBADARG 2
#define PF_EPATHBUF 3

typedef enum {
    PF_AUTO = 0,
    PF_ASTAR,
    PF_ANYANGLE,
    PF_INCREMENTAL,
    PF_HIERARCHICAL,
    PF_ALGO_COUNT
} PfAlgo;

typedef struct {
    int w, h;
    unsigned char* blocked;     /* w*h, 1 = wall */
} PfGrid;

typedef struct { int x, y; } PfPoint;

typedef struct {
    PfPoint* pts;               /* caller-owned buffer */
    int n;
    int cap;
    double cost;                /* true polyline length of pts */
    long expansions;            /* cells popped during this query */
    PfAlgo algo;                /* planner that produced this path */
} PfPath;

typedef struct PfSolver PfSolver;

typedef struct {
    double blocked_ratio;
    double corridor_ratio;      /* walkable cells with <= 2 walkable 4-neighbours */
    double junction_ratio;      /* walkable cells with >= 3 walkable 4-neighbours */
    int clusters;               /* 8x8 cluster count */
    int large;                  /* area >= 96x96 */
} PfProfile;

const char* pf_algo_name(PfAlgo a);

int  pf_grid_init(PfGrid* g, int w, int h);
void pf_grid_free(PfGrid* g);
int  pf_grid_load(PfGrid* g, const char* const* rows, int h);
void pf_grid_set(PfGrid* g, int x, int y, int blocked);
int  pf_grid_blocked(const PfGrid* g, int x, int y);
int  pf_walkable(const PfGrid* g, int x, int y);
int  pf_step_ok(const PfGrid* g, PfPoint a, PfPoint b);
int  pf_line_clear(const PfGrid* g, PfPoint a, PfPoint b);
double pf_path_length(const PfPoint* pts, int n);

/* Any-angle paths may have non-adjacent consecutive points, so validation is
 * parameterised: adjacent steps for the grid planners, line-of-sight for
 * the any-angle one. */
int  pf_path_valid(const PfGrid* g, const PfPath* p, int any_angle);
void pf_path_init(PfPath* p, PfPoint* buf, int cap);

void pf_profile(const PfGrid* g, PfProfile* out);
PfAlgo pf_select_algo(const PfGrid* g, PfPoint s, PfPoint t,
                      int changes_expected, PfProfile* out);

PfSolver* pf_solver_new(const PfGrid* g);
void pf_solver_free(PfSolver* s);
void pf_solver_set_blocked(PfSolver* s, int x, int y, int blocked);
void pf_solver_set_changes_expected(PfSolver* s, int yes);
const PfGrid* pf_solver_grid(const PfSolver* s);

/* One interface for all four algorithms. PF_AUTO asks pf_select_algo().
 * Returns PF_OK, PF_ENOPATH, PF_EBADARG or PF_EPATHBUF. */
int pf_solve(PfSolver* s, PfPoint start, PfPoint goal, PfAlgo algo, PfPath* out);

#endif
