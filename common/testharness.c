#include "testharness.h"

#include <stdio.h>

#include "diag.h"

void th_init(void) { diag_boot(); }

void th_check(TestCtx* t, const char* name, int ok) {
    t->checks++;
    printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) t->failures++;
}

int th_finish(TestCtx* t, const char* suite) {
    printf("%s: %d/%d checks passed\n", suite, t->checks - t->failures, t->checks);
    return t->failures == 0 ? 0 : 1;
}
