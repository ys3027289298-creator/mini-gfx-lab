/* diag.c: allocation-free diagnostics.  This translation unit deliberately
   references no allocator symbol; the buffer is a static arena and every
   record is formatted with snprintf into that arena. */
#include "diag.h"
#include "failpoints.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef DIAG_RING_CAP
#define DIAG_RING_CAP 8192
#endif

static char g_static_buf[DIAG_RING_CAP];
static char* g_buf = g_static_buf;
static size_t g_cap = DIAG_RING_CAP;
static size_t g_head = 0;
static int g_enabled = 0;
static int g_dropped = 0;
static const char* g_faults = NULL;

static int g_has_last = 0;
static const char* g_last_module = NULL;
static const char* g_last_point = NULL;
static DiagCode g_last_code = DIAG_OK;

/* Set once the program calls diag_configure(); from then on diag_boot() must
   not overwrite that explicit choice from the environment. */
static int g_explicit = 0;

static int truthy(const char* s) {
    if (!s) return 0;
    return s[0] == '1' || strcmp(s, "true") == 0 || strcmp(s, "on") == 0 ||
           strcmp(s, "yes") == 0;
}

void diag_boot(void) {
    g_faults = getenv("DIAG_FAULT");
    if (g_explicit) return;
    g_enabled = truthy(getenv("DIAG_ENABLE"));
    const char* cap = getenv("DIAG_BUF_CAP");
    if (cap) {
        unsigned long v = strtoul(cap, NULL, 10);
        if (v > 0 && v <= DIAG_RING_CAP) g_cap = (size_t)v;
    }
}

void diag_configure(int enabled, char* buf, size_t cap) {
    g_explicit = 1;
    if (buf && cap > 0) {
        g_buf = buf;
        g_cap = cap;
    } else {
        g_buf = g_static_buf;
        g_cap = DIAG_RING_CAP;
    }
    g_head = 0;
    g_dropped = 0;
    g_enabled = enabled ? 1 : 0;
}

int diag_is_enabled(void) { return g_enabled; }

int diag_suspend(void) {
    int prev = g_enabled;
    g_enabled = 0;
    return prev;
}

void diag_resume(int previous) {
    g_enabled = previous ? 1 : 0;
}

void diag_reset(void) {
    g_head = 0;
    g_dropped = 0;
    g_has_last = 0;
    g_last_module = NULL;
    g_last_point = NULL;
    g_last_code = DIAG_OK;
}

const char* diag_code_name(DiagCode code) {
    switch (code) {
    case DIAG_OK:              return "DIAG_OK";
    case DIAG_E_INVALID_ARG:   return "DIAG_E_INVALID_ARG";
    case DIAG_E_OUT_OF_RANGE:  return "DIAG_E_OUT_OF_RANGE";
    case DIAG_E_CAPACITY:      return "DIAG_E_CAPACITY";
    case DIAG_E_NO_PATH:       return "DIAG_E_NO_PATH";
    case DIAG_E_NOMEM:         return "DIAG_E_NOMEM";
    case DIAG_E_INTERNAL:      return "DIAG_E_INTERNAL";
    case DIAG_E_DEGRADED:      return "DIAG_E_DEGRADED";
    case DIAG_E_INVARIANT:     return "DIAG_E_INVARIANT";
    default:                   return "DIAG_E_UNKNOWN";
    }
}

/* token matches "point" or "module:point"; tokens are comma/semicolon/space
   separated.  Length-based so the environment string is never modified. */
static int token_matches(const char* tok, size_t len, const char* module,
                         const char* point) {
    size_t ml = strlen(module), pl = strlen(point);
    if (len == pl && memcmp(tok, point, pl) == 0) return 1;
    if (len == ml + 1 + pl && memcmp(tok, module, ml) == 0 && tok[ml] == ':' &&
        memcmp(tok + ml + 1, point, pl) == 0) return 1;
    return 0;
}

int diag_fault_armed(const char* module, const char* point) {
    const char* s = g_faults;
    if (!s || !*s) return 0;
    while (*s) {
        while (*s == ',' || *s == ';' || *s == ' ' || *s == '\t') s++;
        const char* start = s;
        while (*s && *s != ',' && *s != ';' && *s != ' ' && *s != '\t') s++;
        if (s > start && token_matches(start, (size_t)(s - start), module, point))
            return 1;
    }
    return 0;
}

static void record_last(const char* module, const char* point, DiagCode code) {
    g_has_last = 1;
    g_last_module = module;
    g_last_point = point;
    g_last_code = code;
}

int diag_last_failure(const char** module, const char** point, DiagCode* code) {
    if (!g_has_last) return 0;
    if (module) *module = g_last_module;
    if (point) *point = g_last_point;
    if (code) *code = g_last_code;
    return 1;
}

static void append_record(const char* module, const char* point, DiagCode code,
                          const char* message) {
    if (!g_enabled || g_cap == 0) return;
    int n = snprintf(g_buf + g_head, g_cap - g_head, "diag %s %s %s %s\n",
                     module, point, diag_code_name(code), message);
    if (n < 0 || (size_t)n >= g_cap - g_head) {
        g_dropped++;
        g_buf[g_head] = '\0';
        return;
    }
    g_head += (size_t)n;
}

int diag_fail_point(const char* module, const char* point, DiagCode code) {
    if (code == DIAG_OK) code = DIAG_E_INTERNAL;
    record_last(module, point, code);
    const FailPointDef* d = failpoint_lookup(module, point);
    if (d) {
        append_record(module, point, code, d->message);
    } else {
        append_record(module, point, code, "unregistered failure point");
    }
    return DIAG_ERR(code);
}

void diag_emitf(const char* module, const char* point, DiagCode code,
                const char* severity, const char* fmt, ...) {
    if (!g_enabled) return;
    char msg[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    (void)severity;
    append_record(module, point, code, msg);
}

void diag_flush(void* file) {
    if (!file || !g_enabled || g_head == 0) return;
    fwrite(g_buf, 1, g_head, (FILE*)file);
    g_head = 0;
}

size_t diag_buffer_used(void) { return g_head; }
int diag_dropped(void) { return g_dropped; }
unsigned long diag_alloc_count(void) { return 0UL; }
