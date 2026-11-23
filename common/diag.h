/* diag: allocation-free, structured failure diagnostics shared by every
   renderable sub-project.  Disabled diagnostics must not touch the heap and
   must not change control flow: DIAG_FAIL_RETURN always returns non-zero. */
#ifndef MINI_GFX_DIAG_H
#define MINI_GFX_DIAG_H

#include <stddef.h>

typedef enum {
    DIAG_OK = 0,
    DIAG_E_INVALID_ARG = 1,
    DIAG_E_OUT_OF_RANGE = 2,
    DIAG_E_CAPACITY = 3,
    DIAG_E_NO_PATH = 4,
    DIAG_E_NOMEM = 5,
    DIAG_E_INTERNAL = 6,
    DIAG_E_DEGRADED = 7,
    DIAG_E_INVARIANT = 8
} DiagCode;

/* Read DIAG_FAULT / DIAG_ENABLE from the environment and configure the ring
   buffer.  Never allocates.  Safe to call more than once. */
void diag_boot(void);

/* Turn recording on/off.  `buf == NULL` selects the internal static arena. */
void diag_configure(int enabled, char* buf, size_t cap);
int  diag_is_enabled(void);
void diag_reset(void);

/* Temporarily suppress recording without discarding the ring.  Used when a
   self-check deliberately feeds an invalid value and expects rejection. */
int  diag_suspend(void);          /* returns the previous enabled state */
void diag_resume(int previous);

/* Write pending records to a FILE* (passed as void* to keep stdio out of the
   header) and clear the ring.  No-op when disabled. */
void diag_flush(void* file);

/* True when `module:point` (or `point`) is armed through DIAG_FAULT. */
int diag_fault_armed(const char* module, const char* point);

/* Record the failure and return a negative error value (-code), matching the
   -1-on-failure convention of the original modules.  The message and canonical
   code name come from the shared failpoint table. */
int diag_fail_point(const char* module, const char* point, DiagCode code);

/* Record a runtime-formatted diagnostic.  No-op when disabled; never
   allocates. */
void diag_emitf(const char* module, const char* point, DiagCode code,
                const char* severity, const char* fmt, ...);

/* Last failure recorded regardless of whether recording was enabled. */
int diag_last_failure(const char** module, const char** point, DiagCode* code);

const char* diag_code_name(DiagCode code);
size_t      diag_buffer_used(void);
int         diag_dropped(void);
unsigned long diag_alloc_count(void);   /* diagnostics never allocate: always 0 */

/* Failure-site macros.  Every failure return in module render paths must go
   through DIAG_FAIL_RETURN so the coverage checker can enumerate it. */
/* Failure returns live at -100-code so they can never be confused with the
   -1 / 0..n sentinels used by module helper functions. */
#define DIAG_RC_BASE 100
#define DIAG_FAULT(mod, pt)              diag_fault_armed((mod), (pt))
#define DIAG_FAIL_RETURN(mod, pt, code)  return diag_fail_point((mod), (pt), (code))
#define DIAG_ERR(code)                   (-(DIAG_RC_BASE + (int)(code)))
#define DIAG_IS_ERR(rc)                  ((rc) <= -(DIAG_RC_BASE + 1))
#define DIAG_CODE_OF(rc)                 ((DiagCode)(-(rc) - DIAG_RC_BASE))

#endif /* MINI_GFX_DIAG_H */
