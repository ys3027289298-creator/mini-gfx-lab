/*
 * gfault_inject: test-only fault injection. One armed fault per segment;
 * the render bridges call gfault_inject_trip() at the top of each segment.
 *
 * A fault either (a) fires once and disarms itself, or (b) stays armed for
 * `remain` frames. All state is plain static integers: injection adds no
 * heap traffic either.
 */
#ifndef GFAULT_INJECT_H
#define GFAULT_INJECT_H

#include "gfault.h"

typedef struct {
    int armed;          /* a fault is configured */
    int code;           /* positive gfault code to report when it trips */
    int remain;         /* frames remaining (0 = disarmed) */
    int tripped;        /* cumulative trip count since arming */
} GfaultInject;

void gfault_inject_arm(GfaultSeg segment, int code, int frames);
void gfault_inject_disarm(GfaultSeg segment);
void gfault_inject_disarm_all(void);

/* Called once per frame per segment by the bridges. Returns the negative
   gfault code when the fault trips (and records it in the diagnostic sink),
   GFAULT_OK otherwise. */
int gfault_inject_trip(GfaultSeg segment);

const GfaultInject* gfault_inject_state(GfaultSeg segment);

#endif /* GFAULT_INJECT_H */
