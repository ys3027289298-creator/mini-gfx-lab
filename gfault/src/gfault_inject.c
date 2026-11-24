
/* fault injection state machine; static storage, no allocation. */
#include "gfault_inject.h"

static GfaultInject g_inj[GFAULT_SEGMENTS];

void gfault_inject_arm(GfaultSeg segment, int code, int frames) {
    if (segment < 0 || segment >= GFAULT_SEGMENTS || code <= 0 || frames <= 0)
        return;
    g_inj[segment].armed = 1;
    g_inj[segment].code = code;
    g_inj[segment].remain = frames;
    g_inj[segment].tripped = 0;
}

void gfault_inject_disarm(GfaultSeg segment) {
    if (segment < 0 || segment >= GFAULT_SEGMENTS) return;
    g_inj[segment].armed = 0;
    g_inj[segment].remain = 0;
}

void gfault_inject_disarm_all(void) {
    for (int i = 0; i < GFAULT_SEGMENTS; i++)
        gfault_inject_disarm((GfaultSeg)i);
}

int gfault_inject_trip(GfaultSeg segment) {
    if (segment < 0 || segment >= GFAULT_SEGMENTS) return GFAULT_OK;
    GfaultInject* j = &g_inj[segment];
    if (!j->armed || j->remain <= 0) return GFAULT_OK;
    j->remain--;
    j->tripped++;
    if (j->remain == 0) j->armed = 0;
    return gfault_report(segment, j->code, 0);
}

const GfaultInject* gfault_inject_state(GfaultSeg segment) {
    if (segment < 0 || segment >= GFAULT_SEGMENTS) return 0;
    return &g_inj[segment];
}
