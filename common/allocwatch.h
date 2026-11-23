/* allocwatch: counts allocator calls via --wrap for the acceptance builds.
   Used to prove that disabled diagnostics add zero allocations. */
#ifndef MINI_GFX_ALLOCWATCH_H
#define MINI_GFX_ALLOCWATCH_H

void allocwatch_reset(void);
unsigned long allocwatch_count(void);

#endif
