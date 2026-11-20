CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -D_POSIX_C_SOURCE=199309L

# The runtime links only the format reader and the selector: it cannot
# simplify, which keeps the bake/runtime split enforced at link time.
BAKE_SRC = src/bake_main.c src/lodfmt.c src/lodsimp.c
RUN_SRC  = src/run_main.c  src/lodfmt.c src/lodrt.c

all: lod-bake lod-run

lod-bake: $(BAKE_SRC) src/lodfmt.h src/lodsimp.h src/lodtest.h
	$(CC) $(CFLAGS) $(BAKE_SRC) -o $@ -lm

lod-run: $(RUN_SRC) src/lodfmt.h src/lodrt.h src/lodtest.h
	$(CC) $(CFLAGS) $(RUN_SRC) -o $@ -lm

test: lod-bake lod-run
	./lod-bake
	./lod-run

clean:
	rm -f lod-bake lod-run lod.pack

.PHONY: all test clean
