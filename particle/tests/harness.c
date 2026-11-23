/* Acceptance suite for the particle render stage.

   Success path
     * the stage renders its band without touching the heap whether
       diagnostics are enabled or disabled (proved with --wrap counters);
   Failure paths
     * every fixture in tests/faults/ (module, point, code) is injected with
       DIAG_FAULT and must surface as exactly that failure;
     * failures must escape as DIAG_IS_ERR() so the chain can degrade.

   `make acceptance` runs this suite, then re-checks coverage and the golden
   output. */
#include <stdio.h>

#include "acctest.h"
#include "canvas.h"
#include "diag.h"
#include "particle.h"
#include "testharness.h"

static Canvas g_canvas;

static int run_stage(void* arg) {
    (void)arg;
    return particle_stage(&g_canvas, 0, CANVAS_H);
}

int main(void) {
    th_init();
    TestCtx t = {0, 0};
    canvas_init(&g_canvas);

    th_check(&t, "success path renders clean",
             run_stage(NULL) == DIAG_OK);

    AccSuite suite = { "particle", run_stage, NULL, "tests/faults" };
    acctest_run(&t, &suite);

    return th_finish(&t, "particle");
}
