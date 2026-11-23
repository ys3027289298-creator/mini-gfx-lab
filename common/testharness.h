/* testharness: minimal in-process check counter shared by every module's
   acceptance suite.  Results are printed one line per check. */
#ifndef MINI_GFX_TESTHARNESS_H
#define MINI_GFX_TESTHARNESS_H

typedef struct {
    int failures;
    int checks;
} TestCtx;

void th_init(void);
void th_check(TestCtx* t, const char* name, int ok);
int  th_finish(TestCtx* t, const char* suite);

#endif /* MINI_GFX_TESTHARNESS_H */
