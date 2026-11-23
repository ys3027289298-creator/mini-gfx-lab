#ifndef COLORPIPE_H
#define COLORPIPE_H

/* Shared interface for the four-stage colour pipeline.  All stage work
   happens in one working space (linear light); see colorpipe.c for why. */

#define CP_W 144
#define CP_H 96
#define CP_STAGES 4
#define CP_IMG_BYTES (CP_W * CP_H * 3)

typedef struct {
    int unify;          /* 1 = single working space, 0 = per-stage curves */
} Config;

typedef struct {
    long long decodes;       /* transfer-function decodes applied */
    long long encodes;       /* transfer-function encodes applied */
    long long quantizations; /* 8-bit stores */
} Counters;

/* Render the chart.  `stage_out` may be NULL; when non-NULL it receives the
   four stage buffers (zeroed on the unified path, which has none).  `image`
   receives the final RGB8 pixels. */
int cp_render(const Config* cfg, unsigned char stage_out[CP_STAGES][CP_IMG_BYTES],
              unsigned char* image, Counters* ctr);

#endif
