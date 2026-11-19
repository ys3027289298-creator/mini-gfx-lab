CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
LDFLAGS ?=

# Capacity is a compile-time preallocation size; override with CAP= for the
# linearity sweep, e.g. make bench CAP=20000
CAP ?= 10000
CAPFLAGS = -DMAX_PARTICLES=$(CAP)

# Intercept the whole heap-allocator family at link time.
WRAP = -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=realloc \
       -Wl,--wrap=aligned_alloc -Wl,--wrap=posix_memalign \
       -Wl,--wrap=valloc -Wl,--wrap=memalign -Wl,--wrap=pvalloc -Wl,--wrap=free

all: particle-pool

particle-pool: src/particle.c
	$(CC) $(CFLAGS) $< -o $@ -lm

bench: src/bench.c src/particle.c
	$(CC) $(CFLAGS) $(CAPFLAGS) -Isrc src/bench.c -o $@ -lm $(WRAP)

oldbench: src/oldbench.c
	$(CC) $(CFLAGS) $(CAPFLAGS) $< -o $@ -lm

# Compile-only object for symbol auditing (no stdio main pulls anything in here)
audit-obj:
	$(CC) $(CFLAGS) -Wno-unused-function $(CAPFLAGS) -DPARTICLE_NO_MAIN -c src/particle.c -o particle.o
	@echo "--- undefined symbols referenced by the pool implementation ---"
	@nm -u particle.o
	@echo "(no malloc/calloc/realloc/free/aligned_alloc above = clean)"

test: particle-pool
	./particle-pool

clean:
	rm -f particle-pool bench oldbench particle.o
