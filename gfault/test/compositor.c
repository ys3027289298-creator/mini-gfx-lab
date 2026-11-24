/*
 * compositor harness: renders the five segments into one framebuffer with
 * per-segment fault isolation, and exports both the composite and one
 * cropped screenshot per segment.
 *
 * Modes:
 *   harness healthy <outdir>      all five segments, no injected fault
 *   harness fault <seg> <outdir>  inject one fault into <seg> for frame 0
 *
 * A segment that reports a failure is cropped back to the panel background
 * ("that segment's draw is pulled") while every other segment renders
 * normally. The harness never aborts on a segment failure.
 */
#include "gfault.h"
#include "gfault_inject.h"
#include "framebuf.h"
#include <stdio.h>
#include <string.h>

int bridge_particle_render(FrameBuf* fb);
int bridge_tilemap_render(FrameBuf* fb);
int bridge_navgrid_render(FrameBuf* fb);
int bridge_font_render(FrameBuf* fb);
int bridge_palette_render(FrameBuf* fb);

typedef int (*BridgeFn)(FrameBuf* fb);

typedef struct {
    GfaultSeg seg;
    const char* name;
    int fault_code;
    BridgeFn fn;
} SegEntry;

static const SegEntry SEGS[GFAULT_SEGMENTS] = {
    {GFAULT_SEG_PARTICLE, "particle", GFAULT_P_POOL_EXHAUSTED, bridge_particle_render},
    {GFAULT_SEG_TILEMAP,  "tilemap",  GFAULT_T_BAD_LAYER,      bridge_tilemap_render},
    {GFAULT_SEG_NAVGRID,  "navgrid",  GFAULT_N_NO_PATH,        bridge_navgrid_render},
    {GFAULT_SEG_FONT,     "fontatlas",GFAULT_F_ATLAS_FULL,     bridge_font_render},
    {GFAULT_SEG_PALETTE,  "palette",  GFAULT_PAL_INDEX_RANGE,  bridge_palette_render},
};

static const Rgba BG = {18, 18, 24, 255};

static void clear_region(FrameBuf* fb, GfaultSeg seg) {
    const FbRegion* r = fb_region(seg);
    fb_rect(fb, r->x, r->y, r->x + r->w - 1, r->y + r->h - 1, BG);
}

/* crop one panel into its own framebuffer-sized image (panel placed at the
   same coordinates, everything else background) so per-segment screenshots
   are directly pixel-comparable across builds. */
static void crop_segment(const FrameBuf* full, FrameBuf* out, GfaultSeg seg) {
    fb_clear(out, BG);
    const FbRegion* r = fb_region(seg);
    for (int y = 0; y < r->h; y++)
        for (int x = 0; x < r->w; x++)
            out->px[r->y + y][r->x + x] = full->px[r->y + y][r->x + x];
}

static int write_segment(const FrameBuf* full, GfaultSeg seg,
                         const char* name, const char* outdir) {
    FrameBuf crop;
    crop_segment(full, &crop, seg);
    char path[256];
    snprintf(path, sizeof(path), "%s/seg_%s.ppm", outdir, name);
    if (fb_write_ppm(&crop, path) != 0) {
        printf("write_fail %s\n", path);
        return 1;
    }
    unsigned char hash[32];
    fb_hash_rgba(&crop, hash);
    printf("seg=%s file=%s sha256=", name, path);
    for (int i = 0; i < 32; i++) printf("%02x", hash[i]);
    printf("\n");
    return 0;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        printf("usage: %s healthy|fault [seg] <outdir>\n", argv[0]);
        return 2;
    }
    int fault_mode = strcmp(argv[1], "fault") == 0;
    int fault_seg = -1;
    const char* outdir;
    if (fault_mode) {
        if (argc < 4) { printf("fault mode needs a segment and outdir\n"); return 2; }
        for (int i = 0; i < GFAULT_SEGMENTS; i++)
            if (strcmp(argv[2], SEGS[i].name) == 0) fault_seg = i;
        if (fault_seg < 0) { printf("unknown segment %s\n", argv[2]); return 2; }
        outdir = argv[3];
    } else {
        outdir = argv[2];
    }

    gfault_reset();
    if (fault_mode)
        gfault_inject_arm((GfaultSeg)fault_seg, SEGS[fault_seg].fault_code, 1);

    static FrameBuf fb;
    fb_clear(&fb, BG);
    gfault_frame_begin(0);

    int failed[GFAULT_SEGMENTS] = {0};
    for (int i = 0; i < GFAULT_SEGMENTS; i++) {
        if (gfault_inject_trip(SEGS[i].seg) != GFAULT_OK) {
            failed[i] = 1;                       /* injected fault trips first */
            clear_region(&fb, SEGS[i].seg);
            continue;
        }
        int rc = SEGS[i].fn(&fb);
        if (rc < 0 || gfault_segment_failed(SEGS[i].seg)) {
            failed[i] = 1;
            clear_region(&fb, SEGS[i].seg);     /* pull this segment only */
        }
    }

    for (int i = 0; i < GFAULT_SEGMENTS; i++)
        printf("segment=%s status=%s code=0x%02x\n",
               SEGS[i].name, failed[i] ? "FAILED" : "ok",
               gfault_segment_code(SEGS[i].seg) & 0xff);

    GfaultRecord recs[64];
    int dropped = 0;
    int nrec = gfault_pull(recs, 64, &dropped);
    printf("records=%d dropped=%d total=%d\n",
           nrec, dropped, gfault_total_reports());
    for (int i = 0; i < nrec; i++) {
        printf("  record seg=%s code=0x%02x (%s)\n",
               gfault_segment_name(recs[i].segment),
               recs[i].code, gfault_strerror(recs[i].code));
    }

    for (int i = 0; i < GFAULT_SEGMENTS; i++)
        if (write_segment(&fb, SEGS[i].seg, SEGS[i].name, outdir)) return 1;

    char comp[256];
    snprintf(comp, sizeof(comp), "%s/composite.ppm", outdir);
    if (fb_write_ppm(&fb, comp) != 0) { printf("composite write fail\n"); return 1; }
    unsigned char h[32];
    fb_hash_rgba(&fb, h);
    printf("composite=%s sha256=", comp);
    for (int i = 0; i < 32; i++) printf("%02x", h[i]);
    printf("\n");

    int any_failed = 0;
    for (int i = 0; i < GFAULT_SEGMENTS; i++) any_failed |= failed[i];
    /* fault mode expects exactly one failure; healthy expects none */
    if (fault_mode) {
        int nfail = 0;
        for (int i = 0; i < GFAULT_SEGMENTS; i++) nfail += failed[i];
        return nfail == 1 ? 0 : 1;
    }
    return any_failed ? 1 : 0;
}
