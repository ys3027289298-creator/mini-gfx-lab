/* diag.h — unified diagnostics + graceful degradation for mini-gfx-lab.
 *
 * Shared policy (the "unified" part, same for every module):
 *   1. the module runs its acceptance checks via diag_check()
 *   2. all pass  -> the drawing is emitted byte-identical to the non-diag build
 *   3. any fail  -> the drawing is dropped (nothing on stdout), the module's
 *                   own evidence goes to stderr, and the process exits with
 *                   DIAG_EXIT_DROP so a compositor skips just this module
 *   4. the evidence FORMAT is per-module and lives in each module's source:
 *      particle -> numeric table, tilemap -> run log,
 *      navgrid  -> artifact hash, fontatlas -> copyable repro command
 *
 * Fault injection for exercising the degrade path (diag builds only):
 *   MINI_GFX_FAULT=<module> ./diag-bin     (or MINI_GFX_FAULT=all)
 *
 * Build with -DDIAG=1 -I../diag. With DIAG=0 (the default) everything here
 * compiles to nothing: no allocation, no global state, no output, no extra
 * work — the binary behaves exactly as before.
 */
#ifndef MINI_GFX_DIAG_H
#define MINI_GFX_DIAG_H

#include <stdio.h>

#define DIAG_EXIT_DROP 2

#if DIAG

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    const char* module;
    int checks;
    int failed;
    int forced;   /* MINI_GFX_FAULT matched this module */
} Diag;

static void diag_begin(Diag* d, const char* module) {
    const char* f;
    d->module = module;
    d->checks = 0;
    d->failed = 0;
    f = getenv("MINI_GFX_FAULT");
    d->forced = f && (strcmp(f, module) == 0 || strcmp(f, "all") == 0);
}

static int diag_check(Diag* d, int ok) {
    d->checks++;
    if (!ok) d->failed++;
    return ok;
}

static int diag_forced(const Diag* d) { return d->forced; }
static int diag_ok(const Diag* d) { return d->failed == 0; }

/* unified drop notice: one identical first line for every module,
   then the module-specific evidence follows on stderr. */
static void diag_drop_notice(const Diag* d) {
    fprintf(stderr, "[diag] %s: %d/%d acceptance checks failed%s, drawing dropped\n",
            d->module, d->failed, d->checks, d->forced ? " (fault injected)" : "");
}

/* draw buffer: drawing output is staged here and flushed only when every
   acceptance check passed, so a failed module emits nothing on stdout.
   fixed-size static storage — no heap in either build mode. */
#ifndef DIAG_DRAW_CAP
#define DIAG_DRAW_CAP 8192
#endif
static char diag_draw_buf[DIAG_DRAW_CAP];
static size_t diag_draw_len;

static void diag_draw_append(const char* fmt, ...) {
    va_list ap;
    int n;
    if (diag_draw_len >= DIAG_DRAW_CAP) return;
    va_start(ap, fmt);
    n = vsnprintf(diag_draw_buf + diag_draw_len, DIAG_DRAW_CAP - diag_draw_len, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if ((size_t)n >= DIAG_DRAW_CAP - diag_draw_len) diag_draw_len = DIAG_DRAW_CAP; /* truncated */
    else diag_draw_len += (size_t)n;
}

static void diag_draw_flush(void) {
    fwrite(diag_draw_buf, 1, diag_draw_len, stdout);
}

#define DIAG_DRAW(...) diag_draw_append(__VA_ARGS__)

#else /* DIAG == 0: zero allocation, zero overhead, behavior unchanged */

typedef struct { int unused_; } Diag;
#define diag_begin(d, module) ((void)(d))
#define diag_check(d, ok) ((void)0)
#define diag_forced(d) (0)
#define diag_ok(d) (1)
#define diag_drop_notice(d) ((void)0)
#define diag_draw_flush() ((void)0)
#define DIAG_DRAW(...) printf(__VA_ARGS__)

#endif
#endif
