/* navpath: versioned, self-verifying path export.
 *
 * A path document carries the start, goal, every waypoint, the per-segment
 * cost and the A* expansion count. Export supports versions 1..3; version 3
 * can RLE-compress the payload. Reading walks a migration chain
 * (v1 -> v2 -> v3) and rejects unknown versions, missing fields and invalid
 * coordinates. Verification recomputes every segment with nav_cost() and
 * reports the first segment whose value disagrees with the file. */
#ifndef NAVPATH_H
#define NAVPATH_H

#include <stddef.h>

#define NAVPATH_VERSION 3
#define NAVPATH_MAX_PTS 4096

typedef struct { int x, y; } NavPt;

/* The one cost function shared by A* expansion and export/verify.
   A unit orthogonal step costs 1, a diagonal step sqrt(2), and a smoothed
   segment costs its Euclidean length. */
double nav_cost(int dx, int dy);

enum {
    NAVPATH_HAS_COSTS     = 1u << 0,  /* seg_cost[] is populated        */
    NAVPATH_HAS_EXPANDED  = 1u << 1,  /* expanded count came from disk  */
    NAVPATH_COSTS_DERIVED = 1u << 2,  /* costs filled in by v1 migration */
    NAVPATH_COMPRESSED    = 1u << 3   /* payload was stored RLE-coded   */
};

typedef struct {
    int version;          /* version this document conforms to (current)  */
    int source_version;   /* version actually read from / written to disk */
    int w, h;
    NavPt start, goal;
    int n;
    NavPt pts[NAVPATH_MAX_PTS];
    double seg_cost[NAVPATH_MAX_PTS]; /* cost of pts[i-1]->pts[i]; [0] = 0 */
    double total_cost;
    int expanded;         /* -1 when unknown (migrated from v1)           */
    unsigned flags;
} NavPathDoc;

enum {
    NAVPATH_OK = 0,
    NAVPATH_ERR_ARG,
    NAVPATH_ERR_FORMAT,
    NAVPATH_ERR_VERSION,
    NAVPATH_ERR_MISSING,
    NAVPATH_ERR_FIELD,
    NAVPATH_ERR_COORD,
    NAVPATH_ERR_ENDPOINT,
    NAVPATH_ERR_COUNT,
    NAVPATH_ERR_DATA,
    NAVPATH_ERR_CAPACITY
};

typedef struct {
    int code;
    int line;
    char msg[192];
} NavPathErr;

/* Build a document from an already-computed path. Returns 0 on success. */
int navpath_build(NavPathDoc* d, int w, int h, NavPt start, NavPt goal,
                  const NavPt* pts, int n, int expanded);

/* Serialize to a NUL-terminated text buffer.
   version: 1..3; compress: RLE the v3 payload when it is smaller.
   out_len excludes the terminator. */
int navpath_export(const NavPathDoc* d, int version, int compress,
                   char* out, size_t cap, size_t* out_len, NavPathErr* err);

/* Parse and migrate to the current version. w/h may be 0 to trust the file
   header; a non-zero pair must match it. */
int navpath_parse(const char* text, size_t len, int w, int h,
                  NavPathDoc* d, NavPathErr* err);

enum {
    NAVPATH_BAD_NONE = 0,
    NAVPATH_BAD_ENDPOINT,
    NAVPATH_BAD_COORD,
    NAVPATH_BAD_SEG_COST,
    NAVPATH_BAD_TOTAL,
    NAVPATH_BAD_EXPANDED
};

typedef struct {
    int ok;
    int first_bad_segment; /* -1 when every segment agrees */
    int bad_kind;          /* NAVPATH_BAD_* */
    char detail[200];
} NavPathCheck;

/* Recompute costs and compare against the exported values.
   expected_expanded < 0 skips the expansion-count comparison. */
int navpath_verify(const NavPathDoc* d, int w, int h, int expected_expanded,
                   NavPathCheck* out);

const char* navpath_strerror(int code);
const char* navpath_bad_name(int kind);

#endif
