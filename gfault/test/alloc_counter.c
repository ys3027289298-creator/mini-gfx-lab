/*
 * alloc_counter: interposes the libc allocator with counting wrappers.
 *
 * The compositor's render path (framebuffer + gfault + five bridges) must
 * not allocate while diagnostics stay on. To prove that without a profiler
 * we define strong malloc/calloc/realloc/free here: at link time these
 * override the libc symbols, and every allocation increments a counter.
 *
 * The harness prints counter deltas around measured regions. Only stdio's
 * first-use internals are allowed to allocate; render regions must show a
 * delta of 0.
 */
#define _GNU_SOURCE
#include <stdlib.h>
#include <stdio.h>
#include <dlfcn.h>

static void* (*real_malloc)(size_t) = NULL;
static void* (*real_calloc)(size_t, size_t) = NULL;
static void* (*real_realloc)(void*, size_t) = NULL;
static void  (*real_free)(void*) = NULL;

static unsigned long g_allocs = 0;
static unsigned long g_frees = 0;
static unsigned long g_bytes = 0;

static void boot(void) {
    if (real_malloc) return;
    real_malloc  = dlsym(RTLD_NEXT, "malloc");
    real_calloc  = dlsym(RTLD_NEXT, "calloc");
    real_realloc = dlsym(RTLD_NEXT, "realloc");
    real_free    = dlsym(RTLD_NEXT, "free");
}

void* malloc(size_t n) {
    boot();
    g_allocs++; g_bytes += n;
    return real_malloc(n);
}

void* calloc(size_t a, size_t b) {
    boot();
    g_allocs++; g_bytes += a * b;
    return real_calloc(a, b);
}

void* realloc(void* p, size_t n) {
    boot();
    if (p == NULL) { g_allocs++; g_bytes += n; }
    return real_realloc(p, n);
}

void free(void* p) {
    boot();
    if (p) g_frees++;
    real_free(p);
}

unsigned long alloc_counter_allocs(void) { return g_allocs; }
unsigned long alloc_counter_frees(void)  { return g_frees; }
unsigned long alloc_counter_bytes(void)  { return g_bytes; }
