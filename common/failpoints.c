/* failpoints.c: static registry generated from FAILPOINT_LIST. */
#include "failpoints.h"

#include <stdio.h>
#include <string.h>

#define X(mod, pt, code, msg) { #mod, #pt, code, msg },
static const FailPointDef k_points[] = { FAILPOINT_LIST(X) };
#undef X

int failpoint_count(void) {
    return (int)(sizeof k_points / sizeof k_points[0]);
}

const FailPointDef* failpoint_at(int index) {
    if (index < 0 || index >= failpoint_count()) return NULL;
    return &k_points[index];
}

const FailPointDef* failpoint_lookup(const char* module, const char* point) {
    if (!module || !point) return NULL;
    for (int i = 0; i < failpoint_count(); i++) {
        if (strcmp(k_points[i].module, module) == 0 &&
            strcmp(k_points[i].point, point) == 0)
            return &k_points[i];
    }
    return NULL;
}

int failpoint_print(const char* module) {
    int n = 0;
    for (int i = 0; i < failpoint_count(); i++) {
        if (module && strcmp(k_points[i].module, module) != 0) continue;
        printf("%s:%s:%s\n", k_points[i].module, k_points[i].point,
               diag_code_name(k_points[i].code));
        n++;
    }
    return n;
}
