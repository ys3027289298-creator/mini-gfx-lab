/* Build with:
     -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=realloc -Wl,--wrap=free
   Every wrapped allocation/release bumps a counter so the harness can assert
   that a code path performed no heap traffic at all. */
#include "allocwatch.h"

#include <stddef.h>
#include <stdlib.h>

void* __real_malloc(size_t);
void* __real_calloc(size_t, size_t);
void* __real_realloc(void*, size_t);
void  __real_free(void*);

static unsigned long g_allocs;

void allocwatch_reset(void) { g_allocs = 0; }
unsigned long allocwatch_count(void) { return g_allocs; }

void* __wrap_malloc(size_t n) { g_allocs++; return __real_malloc(n); }
void* __wrap_calloc(size_t a, size_t b) { g_allocs++; return __real_calloc(a, b); }
void* __wrap_realloc(void* p, size_t n) { g_allocs++; return __real_realloc(p, n); }
void  __wrap_free(void* p) { __real_free(p); }
