#include "acctest.h"

#define _POSIX_C_SOURCE 200809L

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "allocwatch.h"
#include "diag.h"
#include "failpoints.h"

typedef struct {
    char module[32];
    char point[64];
    DiagCode code;
} FaultCase;

static int parse_fault(const char* path, FaultCase* fc) {
    FILE* f = fopen(path, "r");
    if (!f) return -1;
    memset(fc, 0, sizeof *fc);
    fc->code = DIAG_E_INTERNAL;
    char line[256];
    while (fgets(line, sizeof line, f)) {
        char* nl = strchr(line, '\n');
        if (nl) *nl = '\0';
        char* eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';
        const char* k = line;
        const char* v = eq + 1;
        if (strcmp(k, "module") == 0) snprintf(fc->module, sizeof fc->module, "%s", v);
        else if (strcmp(k, "point") == 0) snprintf(fc->point, sizeof fc->point, "%s", v);
        else if (strcmp(k, "code") == 0) {
            for (int i = 1; i <= DIAG_E_INVARIANT; i++)
                if (strcmp(v, diag_code_name((DiagCode)i)) == 0) fc->code = (DiagCode)i;
        }
    }
    fclose(f);
    return (fc->module[0] && fc->point[0]) ? 0 : -1;
}

static int cmp_name(const void* a, const void* b) {
    return strcmp(*(const char* const*)a, *(const char* const*)b);
}

static int run_faults(TestCtx* t, const AccSuite* suite) {
    DIR* d = opendir(suite->faults_dir);
    if (!d) {
        th_check(t, "faults directory present", 0);
        return 1;
    }
    char* names[64];
    int n = 0;
    struct dirent* e;
    while ((e = readdir(d)) && n < 64) {
        size_t len = strlen(e->d_name);
        if (len > 6 && strcmp(e->d_name + len - 6, ".fault") == 0) {
            size_t sl = strlen(e->d_name) + 1;
            names[n] = malloc(sl);
            if (names[n]) memcpy(names[n], e->d_name, sl);
            n++;
        }
    }
    closedir(d);
    qsort(names, (size_t)n, sizeof names[0], cmp_name);

    int before = t->failures;
    for (int i = 0; i < n; i++) {
        char path[512], tag[128], env[160];
        snprintf(path, sizeof path, "%s/%s", suite->faults_dir, names[i]);
        FaultCase fc;
        if (parse_fault(path, &fc) != 0) {
            snprintf(tag, sizeof tag, "fault fixture parses: %s", names[i]);
            th_check(t, tag, 0);
            free(names[i]);
            continue;
        }
        snprintf(env, sizeof env, "%s:%s", fc.module, fc.point);
        setenv("DIAG_FAULT", env, 1);
        diag_boot();
        diag_reset();

        int rc = suite->run_stage(suite->stage_arg);

        const char* gm = NULL, *gp = NULL;
        DiagCode gc = DIAG_OK;
        int had = diag_last_failure(&gm, &gp, &gc);
        int ok = rc < 0 && had && strcmp(gm, fc.module) == 0 &&
                 strcmp(gp, fc.point) == 0 && gc == fc.code;
        snprintf(tag, sizeof tag, "fault %s:%s -> %s", fc.module, fc.point,
                 diag_code_name(fc.code));
        th_check(t, tag, ok);
        free(names[i]);
    }
    unsetenv("DIAG_FAULT");
    diag_boot();
    return t->failures - before;
}

static int run_noalloc(TestCtx* t, const AccSuite* suite) {
    /* Disabled diagnostics must add no allocator calls on the success path. */
    unsetenv("DIAG_FAULT");
    diag_boot();
    diag_configure(0, NULL, 0);
    diag_boot();
    allocwatch_reset();
    suite->run_stage(suite->stage_arg);
    unsigned long off = allocwatch_count();

    /* Enabled diagnostics must not change the allocation profile either. */
    diag_configure(1, NULL, 0);
    diag_boot();
    allocwatch_reset();
    suite->run_stage(suite->stage_arg);
    unsigned long on = allocwatch_count();
    diag_configure(0, NULL, 0);

    th_check(t, "diag disabled adds zero allocations vs enabled", off == on);

    /* The diagnostics API itself never allocates, disabled or enabled. */
    allocwatch_reset();
    diag_configure(0, NULL, 0);
    diag_boot();
    for (int i = 0; i < 1000; i++) {
        diag_fault_armed(suite->module, "no_such_point");
        diag_is_enabled();
    }
    th_check(t, "diag API with disabled recording allocates nothing",
             allocwatch_count() == 0);

    allocwatch_reset();
    diag_configure(1, NULL, 0);
    diag_boot();
    for (int i = 0; i < 1000; i++) {
        diag_emitf(suite->module, "output_overflow", DIAG_E_CAPACITY,
                   "test", "record %d", i);
    }
    unsigned long emitted = allocwatch_count();
    th_check(t, "diag recording into static ring allocates nothing", emitted == 0);
    diag_flush(NULL);
    diag_configure(0, NULL, 0);
    diag_reset();
    return 0;
}

int acctest_run(TestCtx* t, const AccSuite* suite) {
    diag_configure(0, NULL, 0);
    run_faults(t, suite);
    run_noalloc(t, suite);
    return t->failures;
}
