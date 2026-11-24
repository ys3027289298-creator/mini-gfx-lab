/*
 * gfault: unified failure encoding for the five render segments.
 *
 * Before this module every subsystem failed in its own dialect (bare -1,
 * a silent return, a printed "unknown"). gfault gives every failure one
 * global code, partitioned by segment so the owning module is recoverable
 * from the code alone:
 *
 *   segment                id   code block
 *   particle  (粒子)        0   0x10..0x1F
 *   tilemap   (图块)        1   0x20..0x2F
 *   navgrid   (网格)        2   0x30..0x3F
 *   fontatlas (字体)        3   0x40..0x4F
 *   palette   (调色板)      4   0x50..0x5F
 *
 * 0 is OK. Codes are negative when returned from functions so existing
 * `rc < 0` callers keep working unchanged.
 *
 * The diagnostic path is allocation-free by construction: the ring buffer
 * and its records are static storage, no variadic formatting is performed,
 * and gfault_strerror returns string literals. It is safe to leave the
 * diagnostic sink permanently enabled on a hot render path.
 */
#ifndef GFAULT_H
#define GFAULT_H

#include <stddef.h>

#define GFAULT_SEGMENTS 5

#define GFAULT_OK 0

/* Segment base codes (positive block ids; function returns negate them). */
#define GFAULT_PARTICLE_BASE 0x10
#define GFAULT_TILEMAP_BASE  0x20
#define GFAULT_NAVGRID_BASE  0x30
#define GFAULT_FONT_BASE     0x40
#define GFAULT_PALETTE_BASE  0x50

/* Per-segment failure codes. The numeric offset inside a block is the
   module-local reason; the block itself names the module. */
enum {
    /* particle 0x10 */
    GFAULT_P_EMITTER_FULL   = GFAULT_PARTICLE_BASE + 0, /* emitter table full */
    GFAULT_P_POOL_EXHAUSTED = GFAULT_PARTICLE_BASE + 1, /* no free particle */
    GFAULT_P_BAD_EMITTER    = GFAULT_PARTICLE_BASE + 2, /* unknown emitter id */
    GFAULT_P_BAD_ARG        = GFAULT_PARTICLE_BASE + 3,

    /* tilemap 0x20 */
    GFAULT_T_OUT_OF_RANGE   = GFAULT_TILEMAP_BASE + 0,  /* x/y outside map */
    GFAULT_T_BAD_LAYER      = GFAULT_TILEMAP_BASE + 1,  /* layer id invalid */
    GFAULT_T_BAD_TILE       = GFAULT_TILEMAP_BASE + 2,

    /* navgrid 0x30 */
    GFAULT_N_UNWALKABLE     = GFAULT_NAVGRID_BASE + 0,  /* endpoint blocked */
    GFAULT_N_NO_PATH        = GFAULT_NAVGRID_BASE + 1,  /* search exhausted */
    GFAULT_N_BAD_GRID       = GFAULT_NAVGRID_BASE + 2,  /* load/shape error */
    GFAULT_N_ALLOC          = GFAULT_NAVGRID_BASE + 3,  /* scratch alloc fail */
    GFAULT_N_SMOOTH_CAP     = GFAULT_NAVGRID_BASE + 4,

    /* fontatlas 0x40 */
    GFAULT_F_GLYPH_REJECTED = GFAULT_FONT_BASE + 0,     /* zero/oversize glyph */
    GFAULT_F_ATLAS_FULL     = GFAULT_FONT_BASE + 1,     /* shelf space exhausted */
    GFAULT_F_GLYPH_TABLE    = GFAULT_FONT_BASE + 2,     /* MAX_GLYPHS reached */
    GFAULT_F_NO_GLYPH       = GFAULT_FONT_BASE + 3,     /* unpacked codepoint */

    /* palette 0x50 */
    GFAULT_PAL_INDEX_RANGE  = GFAULT_PALETTE_BASE + 0,  /* index outside LUT */
    GFAULT_PAL_LUT_FULL     = GFAULT_PALETTE_BASE + 1,  /* no free LUT slot */
    GFAULT_PAL_BAD_COLOR    = GFAULT_PALETTE_BASE + 2,
    GFAULT_PAL_BAD_ARG      = GFAULT_PALETTE_BASE + 3,
};

/* Segment ids, also used as framebuffer region indices. */
typedef enum {
    GFAULT_SEG_PARTICLE = 0,
    GFAULT_SEG_TILEMAP  = 1,
    GFAULT_SEG_NAVGRID  = 2,
    GFAULT_SEG_FONT     = 3,
    GFAULT_SEG_PALETTE  = 4,
} GfaultSeg;

typedef struct {
    unsigned int seq;       /* monotonic record id */
    unsigned int frame;     /* frame on which it was reported */
    unsigned char segment;  /* GfaultSeg */
    unsigned char code;     /* positive gfault code */
    unsigned char line;     /* low 8 bits of source line */
} GfaultRecord;

/* Convert a raw gfault code to/from the negative form functions return. */
int gfault_wrap(int code);           /* positive code -> negative, OK stays 0 */
int gfault_unwrap(int rc);           /* negative rc -> positive code */
int gfault_segment_of(int code);     /* segment id that owns a code */
const char* gfault_strerror(int code);
const char* gfault_segment_name(int segment);

/* Report a failure. code is a positive GFAULT_* value; passing GFAULT_OK
   is a no-op so callers can forward any status blindly. Returns the
   negative form so `return gfault_report(...)` works at call sites. */
int gfault_report(GfaultSeg segment, int code, int src_line);

/* Per-frame lifecycle. gfault_frame_begin re-arms every segment; a segment
   that reports during the frame stays failed until the next begin. */
void gfault_frame_begin(unsigned int frame);
int gfault_segment_failed(GfaultSeg segment);
int gfault_segment_code(GfaultSeg segment);

/* Drain the ring (oldest first). Returns records written (<= cap) and the
   number dropped due to overflow. Reads do not mutate the ring. */
int gfault_pull(GfaultRecord* out, int cap, int* dropped_out);

/* Full reset (also zeroes the allocation-free statistics). */
void gfault_reset(void);

/* Enable/disable the diagnostic sink. When disabled, gfault_report() is an
   immediate argument-check-and-return: no ring writes, no formatting. The
   render path does not allocate in either mode; this switch exists so the
   allocation-counter test can prove that directly. */
void gfault_set_enabled(int on);
int gfault_enabled(void);

/* Number of reports observed since reset and how many the ring dropped. */
int gfault_total_reports(void);
int gfault_total_dropped(void);

#endif /* GFAULT_H */
