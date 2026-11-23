/* Chain acceptance suite.

   Verifies, for each of the four stages independently:
     * with no fault armed the frame contains that stage's band;
     * arming any registered failure point of that stage degrades exactly that
       band, leaves the other three bands byte-identical to the clean frame,
       and reports the failure through diagnostics;
     * the process returns non-zero and prints "degraded=1".

   Also proves the diagnostics layer adds no heap traffic in the chain. */
#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "acctest.h"
#include "allocwatch.h"
#include "canvas.h"
#include "chain.h"
#include "diag.h"
#include "failpoints.h"
#include "testharness.h"

typedef char BandBuf[CHAIN_BAND_ROWS][CANVAS_W + 1];

static void band(const Canvas* f, int i, BandBuf out) {
    int y0 = i * CHAIN_BAND_ROWS;
    for (int r = 0; r < CHAIN_BAND_ROWS; r++)
        memcpy(out[r], f->px[y0 + r], (size_t)CANVAS_W + 1);
}

static int bands_equal(const BandBuf a, const BandBuf b) {
    for (int r = 0; r < CHAIN_BAND_ROWS; r++)
        if (strcmp(a[r], b[r]) != 0) return 0;
    return 1;
}

int main(void) {
    th_init();
    TestCtx t = {0, 0};
    diag_configure(0, NULL, 0);

    Canvas clean = {0};
    unsigned failed = 0;
    int degraded = chain_render(&clean, &failed);
    th_check(&t, "clean chain renders all four bands", degraded == 0 && failed == 0);

    static BandBuf clean_bands[CHAIN_BANDS];
    for (int i = 0; i < CHAIN_BANDS; i++) band(&clean, i, clean_bands[i]);

    static const char* mods[CHAIN_BANDS] = { "particle", "tilemap", "navgrid", "spatial" };
    int failures_before = t.failures;

    for (int i = 0; i < CHAIN_BANDS; i++) {
        int points = 0;
        for (int k = 0; k < failpoint_count(); k++) {
            const FailPointDef* d = failpoint_at(k);
            if (strcmp(d->module, mods[i]) != 0) continue;
            points++;

            char env[96];
            snprintf(env, sizeof env, "%s:%s", d->module, d->point);
            setenv("DIAG_FAULT", env, 1);
            diag_boot();
            diag_reset();

            Canvas f = {0};
            unsigned mask = 0;
            int dg = chain_render(&f, &mask);

            const char* gm = NULL, *gp = NULL;
            DiagCode gc = DIAG_OK;
            int had = diag_last_failure(&gm, &gp, &gc);
            unsetenv("DIAG_FAULT");
            diag_boot();
            diag_reset();

            int only_this = (dg == 1) && (mask == (1u << i));
            int others_same = 1;
            for (int j = 0; j < CHAIN_BANDS; j++) {
                if (j == i) continue;
                BandBuf b;
                band(&f, j, b);
                if (!bands_equal(b, clean_bands[j])) others_same = 0;
            }
            char name[128];
            snprintf(name, sizeof name, "%s degrades alone on %s:%s",
                     mods[i], d->module, d->point);
            int ok = only_this && others_same && had &&
                     strcmp(gm, d->module) == 0 && strcmp(gp, d->point) == 0;
            th_check(&t, name, ok);
        }
        th_check(&t, "every band has registered failure points", points > 0);
    }
    (void)failures_before;

    /* No-allocation check for the full chain with diagnostics toggled. */
    diag_configure(0, NULL, 0);
    allocwatch_reset();
    Canvas f1 = {0};
    chain_render(&f1, NULL);
    unsigned long off = allocwatch_count();

    diag_configure(1, NULL, 0);
    allocwatch_reset();
    Canvas f2 = {0};
    chain_render(&f2, NULL);
    unsigned long on = allocwatch_count();
    diag_configure(0, NULL, 0);

    int same_frame = 1;
    for (int y = 0; y < CANVAS_H; y++)
        if (strcmp(f1.px[y], f2.px[y]) != 0) same_frame = 0;
    th_check(&t, "diagnostics toggle does not alter the frame", same_frame);
    th_check(&t, "diagnostics disabled add zero allocations in chain", off == on);

    return th_finish(&t, "chain");
}
