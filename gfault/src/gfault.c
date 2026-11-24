
/* gfault core: static ring buffer, zero heap traffic. See include/gfault.h. */
#include "gfault.h"

#define RING_CAP 64

typedef struct {
    GfaultRecord ring[RING_CAP];
    int head;               /* next write slot */
    int count;              /* records currently held */
    unsigned int seq;
    unsigned int frame;
    int seg_code[GFAULT_SEGMENTS];   /* last code per segment, 0 = healthy */
    int total_reports;
    int total_dropped;
} Diag;

static Diag g_diag;
static int g_enabled = 1;

int gfault_wrap(int code) {
    return code == GFAULT_OK ? GFAULT_OK : -code;
}

int gfault_unwrap(int rc) {
    return rc < 0 ? -rc : rc;
}

int gfault_segment_of(int code) {
    if (code >= GFAULT_PARTICLE_BASE && code < GFAULT_PARTICLE_BASE + 16)
        return GFAULT_SEG_PARTICLE;
    if (code >= GFAULT_TILEMAP_BASE && code < GFAULT_TILEMAP_BASE + 16)
        return GFAULT_SEG_TILEMAP;
    if (code >= GFAULT_NAVGRID_BASE && code < GFAULT_NAVGRID_BASE + 16)
        return GFAULT_SEG_NAVGRID;
    if (code >= GFAULT_FONT_BASE && code < GFAULT_FONT_BASE + 16)
        return GFAULT_SEG_FONT;
    if (code >= GFAULT_PALETTE_BASE && code < GFAULT_PALETTE_BASE + 16)
        return GFAULT_SEG_PALETTE;
    return -1;
}

const char* gfault_segment_name(int segment) {
    switch (segment) {
    case GFAULT_SEG_PARTICLE: return "particle";
    case GFAULT_SEG_TILEMAP:  return "tilemap";
    case GFAULT_SEG_NAVGRID:  return "navgrid";
    case GFAULT_SEG_FONT:     return "fontatlas";
    case GFAULT_SEG_PALETTE:  return "palette";
    default: return "unknown";
    }
}

const char* gfault_strerror(int code) {
    switch (code) {
    case GFAULT_OK:                return "ok";
    case GFAULT_P_EMITTER_FULL:    return "particle: emitter table full";
    case GFAULT_P_POOL_EXHAUSTED:  return "particle: particle pool exhausted";
    case GFAULT_P_BAD_EMITTER:     return "particle: unknown emitter id";
    case GFAULT_P_BAD_ARG:         return "particle: invalid argument";
    case GFAULT_T_OUT_OF_RANGE:    return "tilemap: coordinate out of range";
    case GFAULT_T_BAD_LAYER:       return "tilemap: invalid layer";
    case GFAULT_T_BAD_TILE:        return "tilemap: invalid tile id";
    case GFAULT_N_UNWALKABLE:      return "navgrid: endpoint not walkable";
    case GFAULT_N_NO_PATH:         return "navgrid: no path to target";
    case GFAULT_N_BAD_GRID:        return "navgrid: malformed grid";
    case GFAULT_N_ALLOC:           return "navgrid: scratch allocation failed";
    case GFAULT_N_SMOOTH_CAP:      return "navgrid: smoothed path exceeded capacity";
    case GFAULT_F_GLYPH_REJECTED:  return "fontatlas: glyph size rejected";
    case GFAULT_F_ATLAS_FULL:      return "fontatlas: atlas out of shelf space";
    case GFAULT_F_GLYPH_TABLE:     return "fontatlas: glyph table full";
    case GFAULT_F_NO_GLYPH:        return "fontatlas: codepoint not packed";
    case GFAULT_PAL_INDEX_RANGE:   return "palette: index outside LUT";
    case GFAULT_PAL_LUT_FULL:      return "palette: LUT slot table full";
    case GFAULT_PAL_BAD_COLOR:     return "palette: malformed color";
    case GFAULT_PAL_BAD_ARG:       return "palette: invalid argument";
    default:                       return "unknown";
    }
}

int gfault_report(GfaultSeg segment, int code, int src_line) {
    if (code == GFAULT_OK) return GFAULT_OK;
    if (segment < 0 || segment >= GFAULT_SEGMENTS) return -code;
    if (!g_enabled) return -code;

    GfaultRecord* r = &g_diag.ring[g_diag.head];
    if (g_diag.count == RING_CAP) {
        /* overwrite oldest; head advances and count stays pinned */
        g_diag.total_dropped++;
        g_diag.head = (g_diag.head + 1) % RING_CAP;
    } else {
        g_diag.count++;
        g_diag.head = (g_diag.head + 1) % RING_CAP;
    }
    r->seq = ++g_diag.seq;
    r->frame = g_diag.frame;
    r->segment = (unsigned char)segment;
    r->code = (unsigned char)code;
    r->line = (unsigned char)src_line;

    if (g_diag.seg_code[segment] == GFAULT_OK)
        g_diag.seg_code[segment] = code;   /* latch first failure of frame */
    g_diag.total_reports++;
    return -code;
}

void gfault_frame_begin(unsigned int frame) {
    g_diag.frame = frame;
    for (int i = 0; i < GFAULT_SEGMENTS; i++) g_diag.seg_code[i] = GFAULT_OK;
}

int gfault_segment_failed(GfaultSeg segment) {
    if (segment < 0 || segment >= GFAULT_SEGMENTS) return 0;
    return g_diag.seg_code[segment] != GFAULT_OK;
}

int gfault_segment_code(GfaultSeg segment) {
    if (segment < 0 || segment >= GFAULT_SEGMENTS) return GFAULT_OK;
    return g_diag.seg_code[segment];
}

int gfault_pull(GfaultRecord* out, int cap, int* dropped_out) {
    if (cap <= 0 || out == 0) {
        if (dropped_out) *dropped_out = g_diag.total_dropped;
        return 0;
    }
    int start = (g_diag.head - g_diag.count + RING_CAP) % RING_CAP;
    int n = g_diag.count < cap ? g_diag.count : cap;
    for (int i = 0; i < n; i++)
        out[i] = g_diag.ring[(start + i) % RING_CAP];
    if (dropped_out) *dropped_out = g_diag.total_dropped;
    return n;
}

void gfault_reset(void) {
    for (int i = 0; i < RING_CAP; i++) {
        g_diag.ring[i].seq = 0; g_diag.ring[i].frame = 0;
        g_diag.ring[i].segment = 0; g_diag.ring[i].code = 0;
        g_diag.ring[i].line = 0;
    }
    g_diag.head = 0; g_diag.count = 0; g_diag.seq = 0; g_diag.frame = 0;
    for (int i = 0; i < GFAULT_SEGMENTS; i++) g_diag.seg_code[i] = 0;
    g_diag.total_reports = 0; g_diag.total_dropped = 0;
}

int gfault_total_reports(void) { return g_diag.total_reports; }
int gfault_total_dropped(void) { return g_diag.total_dropped; }

void gfault_set_enabled(int on) { g_enabled = on ? 1 : 0; }
int gfault_enabled(void) { return g_enabled; }
