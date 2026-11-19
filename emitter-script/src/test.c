/* test: ten example scripts vs hand-written reference, plus diagnostics.

   Usage: ems-test <examples-dir>
   Each examples/NN_*.emitters is compiled and executed, then compared field
   by field against ref_NN from ref.h. The examples are read from disk, not
   embedded, so editing a script and rebuilding exercises the real parser. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ems.h"
#include "ref.h"

static int failures = 0;
static int checks = 0;

static void check(const char* name, int ok, const char* detail) {
    checks++;
    printf("%s=%d%s%s\n", name, ok ? 1 : 0, ok || !detail ? "" : "  # ", ok || !detail ? "" : detail);
    if (!ok) failures++;
}

static char* read_file(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return NULL;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
    long n = ftell(f);
    if (n < 0) { fclose(f); return NULL; }
    rewind(f);
    char* buf = malloc((size_t)n + 1);
    if (!buf) { fclose(f); return NULL; }
    size_t got = fread(buf, 1, (size_t)n, f);
    buf[got] = '\0';
    fclose(f);
    return buf;
}

typedef struct {
    const char* file;
    int pool, budget, ticks;
    float dt;
    void (*ref)(EmsResult*);
} Example;

static const Example EXAMPLES[] = {
    { "01_single.emitters",        128,  10,  5, 0.1f,  ref_01 },
    { "02_priority.emitters",     4096,  50, 20, 0.1f,  ref_02 },
    { "03_curve_quota.emitters",   256, 100,  4, 0.25f, ref_03 },
    { "04_curve_velocity.emitters",256, 100,  8, 0.125f, ref_04 },
    { "05_curve_life.emitters",    256, 100,  8, 0.125f, ref_05 },
    { "06_step.emitters",          256, 100,  6, 0.5f,  ref_06 },
    { "07_cubic.emitters",         256, 100,  8, 0.125f, ref_07 },
    { "08_pool_saturation.emitters",10, 100,  5, 0.1f,  ref_08 },
    { "09_fractional_quota.emitters",64,  10,  8, 1.0f,  ref_09 },
    { "10_budget_split.emitters",  256,   7, 10, 0.2f,  ref_10 },
};

static int near(double a, double b) {
    double d = fabs(a - b);
    double m = fabs(a) > fabs(b) ? fabs(a) : fabs(b);
    return d <= 1e-5 || d <= 1e-5 * m;
}

static int compare_results(const char* name, const EmsResult* got, const EmsResult* want) {
    char detail[160];
    if (got->frames != want->frames) {
        snprintf(detail, sizeof(detail), "frames got=%d want=%d", got->frames, want->frames);
        check(name, 0, detail);
        return 0;
    }
    for (int t = 0; t < got->frames; t++) {
        if (got->frame_alive[t] != want->frame_alive[t]) {
            snprintf(detail, sizeof(detail), "frame_alive[%d] got=%d want=%d",
                     t, got->frame_alive[t], want->frame_alive[t]);
            check(name, 0, detail);
            return 0;
        }
    }
    if (got->high_water != want->high_water) {
        snprintf(detail, sizeof(detail), "high_water got=%d want=%d", got->high_water, want->high_water);
        check(name, 0, detail);
        return 0;
    }
    for (int e = 0; e < EMS_MAX_EMITTERS; e++) {
        if (got->spawned[e] != want->spawned[e] || got->expired[e] != want->expired[e] ||
            got->alive[e] != want->alive[e]) {
            snprintf(detail, sizeof(detail),
                     "emitter[%d] spawned got=%d want=%d, expired got=%d want=%d, alive got=%d want=%d",
                     e, got->spawned[e], want->spawned[e], got->expired[e], want->expired[e],
                     got->alive[e], want->alive[e]);
            check(name, 0, detail);
            return 0;
        }
    }
    if (!near(got->sum_x, want->sum_x) || !near(got->sum_y, want->sum_y) ||
        !near(got->sum_age, want->sum_age)) {
        snprintf(detail, sizeof(detail), "sum got=(%.6f,%.6f,%.6f) want=(%.6f,%.6f,%.6f)",
                 got->sum_x, got->sum_y, got->sum_age,
                 want->sum_x, want->sum_y, want->sum_age);
        check(name, 0, detail);
        return 0;
    }
    return 1;
}

static void run_examples(const char* dir) {
    for (size_t i = 0; i < sizeof(EXAMPLES) / sizeof(EXAMPLES[0]); i++) {
        const Example* ex = &EXAMPLES[i];
        char path[512];
        snprintf(path, sizeof(path), "%s/%s", dir, ex->file);
        char* src = read_file(path);
        if (!src) {
            printf("read_%s=0  # cannot open %s\n", ex->file, path);
            failures++;
            checks++;
            continue;
        }
        char label[160];
        snprintf(label, sizeof(label), "example_%02zu_compile", i + 1);
        EmsProgram prog;
        EmsError err;
        if (ems_compile(src, &prog, &err) != 0) {
            char msg[512];
            ems_format_error(src, ex->file, &err, msg, sizeof(msg));
            check(label, 0, msg);
            free(src);
            continue;
        }
        check(label, 1, NULL);
        snprintf(label, sizeof(label), "example_%02zu_params", i + 1);
        if (prog.pool != ex->pool || prog.budget != ex->budget ||
            prog.ticks != ex->ticks || prog.dt != ex->dt) {
            char detail[160];
            snprintf(detail, sizeof(detail),
                     "script got pool=%d budget=%d ticks=%d dt=%.3f, reference assumes %d %d %d %.3f",
                     prog.pool, prog.budget, prog.ticks, prog.dt,
                     ex->pool, ex->budget, ex->ticks, ex->dt);
            check(label, 0, detail);
            free(src);
            continue;
        }
        check(label, 1, NULL);

        EmsResult got, want;
        snprintf(label, sizeof(label), "example_%02zu_matches_handwritten", i + 1);
        if (ems_run(&prog, &got) != 0) {
            check(label, 0, "ems_run failed");
            free(src);
            continue;
        }
        ex->ref(&want);
        if (compare_results(label, &got, &want)) check(label, 1, NULL);

        EmsResult again;
        snprintf(label, sizeof(label), "example_%02zu_deterministic", i + 1);
        ems_run(&prog, &again);
        check(label, memcmp(&got, &again, sizeof(got)) == 0, "second run differs");
        free(src);
    }
}

/* ---- diagnostics: expected line/column for hand-picked broken scripts ---- */

typedef struct {
    const char* src;
    int line, col;
    const char* needle;   /* substring that must appear in the message */
} BadCase;

static const BadCase BAD[] = {
    { "pool 8\nemitter a priority 1 {\n  quota 1\n  pos 0, 0\n  vel 0, 0\n  life 1\n",
      7, 1, "expected '}'" },
    { "emitter a priority x {\n}\n", 1, 20, "expected an integer" },
    { "emitter a priority 1 {\n  quota @nope\n  pos 0, 0\n  vel 0, 0\n  life 1\n}\n",
      2, 9, "unknown curve '@nope'" },
    { "emitter a priority 1 {\n  pos 0, 0\n  pos 1, 1\n  vel 0, 0\n  life 1\n}\n",
      3, 3, "duplicate field 'pos'" },
    { "emitter a priority 1 {\n  quota 1\n  pos 0, 0\n  vel 0, 0\n}\n",
      5, 1, "needs quota, pos, vel and life" },
    { "curve c(0, 1, linear)\ncurve c(0, 1, linear)\n", 2, 7, "already defined on line 1" },
    { "pool 0\n", 1, 6, "expected an integer in [1, 65536]" },
    { "pool 4\nfrobnicate 3\n", 2, 1, "unknown statement 'frobnicate'" },
    { "pool 4 $ 3\n", 1, 8, "invalid token" },
    { "pool 8\nemitter a priority 1 {\n  quota 1\n  pos 0, 0\n  vel 0, @x\n  life 1\n}\n",
      5, 10, "unknown curve '@x'" },
    { "pool 8\nemitter a priority 1 {\n  quota 1\n  pos 0, 0, 3\n  vel 0, 0\n  life 1\n}\n",
      4, 11, "unknown field ','" },
    { "dt -0.5\n", 1, 4, "dt must not be negative" },
    { "emitter a priority 1 {\n  quota 1\n  pos 0 0\n  vel 0, 0\n  life 1\n}\n",
      3, 9, "expected ','" },
    { "curve c(0, 1, wobbly)\n", 1, 15, "expected a curve shape" },
    { "emitter a priority 1 {\n  quota 1\n  pos 0, 0\n  vel 0, 0\n  life 1\n}\nemitter a priority 2 {\n  quota 1\n  pos 0, 0\n  vel 0, 0\n  life 1\n}\n",
      7, 9, "already defined on line 1" },
};

static void run_diagnostics(void) {
    for (size_t i = 0; i < sizeof(BAD) / sizeof(BAD[0]); i++) {
        const BadCase* b = &BAD[i];
        EmsProgram prog;
        EmsError err;
        char name[64];
        snprintf(name, sizeof(name), "error_%02zu_line_col", i + 1);
        if (ems_compile(b->src, &prog, &err) == 0) {
            check(name, 0, "expected compile failure");
            continue;
        }
        int ok = err.line == b->line && err.col == b->col &&
                 strstr(err.message, b->needle) != NULL;
        char detail[256];
        snprintf(detail, sizeof(detail), "got %d:%d \"%s\"", err.line, err.col, err.message);
        check(name, ok, detail);

        snprintf(name, sizeof(name), "error_%02zu_rendered", i + 1);
        char msg[512];
        ems_format_error(b->src, "bad.emitters", &err, msg, sizeof(msg));
        char want_prefix[96];
        snprintf(want_prefix, sizeof(want_prefix), "bad.emitters:%d:%d: error: ",
                 b->line, b->col);
        int rendered = strncmp(msg, want_prefix, strlen(want_prefix)) == 0;
        if (rendered && b->col > 0) {
            const char* caret = strchr(msg, '\n');
            if (caret) {
                caret = strchr(caret + 1, '\n');
                if (caret) {
                    caret++;
                    /* the caret line must be spaces/tabs up to col-1, then '^' */
                    int k = 0;
                    while (caret[k] == ' ' || caret[k] == '\t') k++;
                    rendered = (k == b->col - 1 && caret[k] == '^');
                } else rendered = 0;
            } else rendered = 0;
        }
        check(name, rendered, msg);
    }
}

static void run_edge_cases(void) {
    EmsProgram prog;
    EmsError err;

    /* empty program compiles and runs to an empty result */
    check("empty_compiles", ems_compile("", &prog, &err) == 0, err.message);
    EmsResult r;
    check("empty_runs", ems_run(&prog, &r) == 0 && r.frames == 0 && r.high_water == 0, NULL);

    /* comments and blank lines anywhere */
    const char* commented = "  # leading comment\npool 4 # trailing\n\n# only\n"
                            "curve c(0,1,linear)\n"
                            "emitter a priority 3 { # open\n quota 1\n pos 0,0\n vel 0,0\n life 1\n} # close\n";
    check("comments_compile", ems_compile(commented, &prog, &err) == 0, err.message);

    /* same priority: script order decides */
    const char* tie = "pool 8\nbudget 3\nticks 1\n"
                      "emitter first priority 1 { quota 2 pos 0,0 vel 0,0 life 9 }\n"
                      "emitter second priority 1 { quota 2 pos 0,0 vel 0,0 life 9 }\n";
    check("tie_compiles", ems_compile(tie, &prog, &err) == 0, err.message);
    check("tie_runs", ems_run(&prog, &r) == 0, NULL);
    check("tie_script_order_wins", r.spawned[0] == 2 && r.spawned[1] == 1, NULL);

    /* pool full: budget not consumed by a blocked emitter */
    const char* full = "pool 2\nbudget 10\nticks 2\ndt 1\n"
                       "emitter fill priority 9 { quota 5 pos 0,0 vel 0,0 life 100 }\n"
                       "emitter starve priority 1 { quota 5 pos 0,0 vel 0,0 life 100 }\n";
    check("full_compiles", ems_compile(full, &prog, &err) == 0, err.message);
    check("full_runs", ems_run(&prog, &r) == 0, NULL);
    check("full_starves_second", r.spawned[0] == 2 && r.spawned[1] == 0 && r.high_water == 2, NULL);

    /* negative life clamps to zero: particle expires on its spawn tick */
    const char* neg = "pool 4\nticks 1\ndt 1\n"
                      "emitter a priority 0 { quota 2 pos 0,0 vel 0,0 life -5 }\n";
    check("neg_life_compiles", ems_compile(neg, &prog, &err) == 0, err.message);
    check("neg_life_expires", ems_run(&prog, &r) == 0 && r.spawned[0] == 2 &&
                              r.expired[0] == 2 && r.frame_alive[0] == 0, NULL);

    /* unknown fields, missing fields and stray tokens are rejected */
    const char* unknown = "emitter a priority 0 { quota 1 pos 0,0 vel 0,0 life 1 mass 2 }\n";
    check("unknown_field_rejected", ems_compile(unknown, &prog, &err) == -1, NULL);
    const char* stray = "pool 4\n}\n";
    check("stray_token_rejected", ems_compile(stray, &prog, &err) == -1, NULL);

    /* out-of-range limits are reported, not silently truncated */
    const char* big = "pool 999999\n";
    check("pool_limit_rejected", ems_compile(big, &prog, &err) == -1 &&
                                 err.line == 1 && err.col == 6, NULL);

    /* cloned programs run independently */
    const char* one = "pool 8\nbudget 2\nticks 3\ndt 1\n"
                      "emitter a priority 0 { quota 1 pos @c, 0 vel 0,0 life 1 }\n"
                      "curve c(0, 5, linear)\n";
    check("curve_after_use_compiles", ems_compile(one, &prog, &err) == 0,
          "curves may be declared after the emitter that references them");
    EmsResult a, b;
    ems_run(&prog, &a);
    EmsProgram copy = prog;
    ems_run(&copy, &b);
    check("copy_runs", memcmp(&a, &b, sizeof(a)) == 0, NULL);
}

int main(int argc, char** argv) {
    const char* dir = argc > 1 ? argv[1] : "examples";
    run_examples(dir);
    run_diagnostics();
    run_edge_cases();
    printf("checks=%d\n", checks);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
