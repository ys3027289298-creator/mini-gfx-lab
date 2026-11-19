/* regions.h: chunk-aligned connected-component region map for streaming.

   A region is a maximal set of walkable cells that are connected without
   crossing a render-chunk boundary.  Regions never span chunks, so a region
   is always streamed as one unit and drawn as one batch.

   Connectivity is 4-neighbour orthogonal.  This matches the movement model
   of the A* in navgrid.c: a legal diagonal step there requires both
   orthogonal neighbours to be free, so any diagonally reachable pair is also
   4-connected.  Partitioning by 4-connectivity therefore yields exactly the
   sets reachable under navgrid's corner-cut rule.

   Region ids are canonical: id = (smallest linear cell index in the region)+1,
   so 0 always means "blocked / no region" and the same partition always
   produces the same label array.  This makes incremental relabelling
   directly comparable with a from-scratch partition. */
#ifndef MINI_NAVGRID_REGIONS_H
#define MINI_NAVGRID_REGIONS_H

typedef struct {
    int cells;                   /* cells in the region */
    int minx, miny, maxx, maxy;  /* cell-space bounding box, inclusive */
} RegionMeta;

typedef struct {
    int w, h, n;                 /* grid dimensions, n = w*h */
    int chunk;                   /* render chunk edge, in cells */
    int cw, ch;                  /* chunk counts: ceil(w/chunk), ceil(h/chunk) */
    unsigned char *blocked;      /* [n] 1 = obstacle */
    int *label;                  /* [n] 0 = none, else canonical region id */
    unsigned char *active;       /* [n+1] region id currently exists */
    RegionMeta *meta;            /* [n+1] indexed by region id */
    int *queue;                  /* [n] BFS scratch */
    int *visit;                  /* [n+1] scratch for chunk queries, lazily allocated */
    unsigned int visit_gen;
    int region_count;
    int built;                   /* rg_build has run */
    int last_relabel_cells;      /* cells rewritten by the last build/toggle */
} RegionGrid;

typedef struct {
    int regions;                 /* active regions */
    int chunks;                  /* render chunks, cw*ch */
    int occupied_chunks;         /* chunks holding at least one region */
    int max_regions_per_chunk;   /* worst-case cull granularity */
    int max_region_cells;        /* largest region */
    long long region_cells;      /* sum of region sizes (= walkable cells) */
} RgStats;

/* Allocate a w*h grid partitioned into chunk x chunk render chunks.
   blocked[] starts zeroed (fully walkable); fill it, then call rg_build(). */
int rg_init(RegionGrid *g, int w, int h, int chunk);
void rg_free(RegionGrid *g);

/* Full partition, O(w*h).  Returns the region count or -1. */
int rg_build(RegionGrid *g);

/* Flip one cell and relabel only the region(s) that touch it.  Chunks other
   than the cell's own are untouched, and labels elsewhere in the chunk are
   preserved.  Cost is O(size of the affected region(s)): closing a cell can
   split its chunk's region into up to 4 pieces, and opening one can merge up
   to 4 neighbouring regions.  Returns the new region count, or -1 on bad
   arguments or if rg_build has not run. */
int rg_toggle(RegionGrid *g, int x, int y);

/* Canonical region id at (x,y); 0 when out of bounds, blocked, or unbuilt. */
int rg_region_at(const RegionGrid *g, int x, int y);

/* Region ids present in chunk (cx,cy), written to out (capacity max_out).
   At most 4*(chunk/2)^2 regions can fit in one chunk (each region needs its
   own row-major minimum cell).  Returns the count, or -1 on bad arguments or
   if the chunk holds more than max_out regions. */
int rg_chunk_regions(RegionGrid *g, int cx, int cy, int *out, int max_out);

void rg_stats(RegionGrid *g, RgStats *out);

/* Debug/test consistency check: canonical ids, metadata, maximality and
   chunk containment.  Returns 1 when consistent. */
int rg_validate(const RegionGrid *g);

#endif
