/* Acceptance suite for the spatial render stage.  See
   particle/tests/harness.c for the shared structure. */
#include <stdio.h>

#include "acctest.h"
#include "canvas.h"
#include "diag.h"
#include "spatial.h"
#include "testharness.h"

static Canvas g_canvas;

static int run_stage(void* arg) {
    (void)arg;
    return spatial_stage(&g_canvas, 0, CANVAS_H);
}

int main(void) {
    th_init();
    TestCtx t = {0, 0};
    canvas_init(&g_canvas);

    th_check(&t, "success path renders clean", run_stage(NULL) == DIAG_OK);

    AccSuite suite = { "spatial", run_stage, NULL, "tests/faults" };
    acctest_run(&t, &suite);

    return th_finish(&t, "spatial");
}
