/* small shared test helpers for the lod tools */
#ifndef LODTEST_H
#define LODTEST_H

#include <stdio.h>

static int g_failures = 0;

static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) g_failures++;
}

static int test_result(void) {
    printf("all_ok=%d\n", g_failures == 0);
    return g_failures == 0 ? 0 : 1;
}

#endif
