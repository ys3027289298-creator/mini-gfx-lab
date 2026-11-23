/* acctest: shared driver for the per-module acceptance suites.
   Fault cases are data files (<module>/tests/faults/<point>.fault) with
   key=value pairs: module, point, code.  The driver injects each point with
   DIAG_FAULT, runs the module stage, and verifies the recorded failure. */
#ifndef MINI_GFX_ACCTEST_H
#define MINI_GFX_ACCTEST_H

#include "canvas.h"
#include "testharness.h"

typedef int (*StageCall)(void* arg);

typedef struct {
    const char* module;
    StageCall   run_stage;   /* renders the module's success scene */
    void*       stage_arg;
    const char* faults_dir;
} AccSuite;

/* Runs every fault fixture plus the disabled-diagnostics no-allocation check.
   Returns the number of new failures (also accumulated into `t`). */
int acctest_run(TestCtx* t, const AccSuite* suite);

#endif
