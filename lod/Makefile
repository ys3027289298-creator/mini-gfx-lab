CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: lod-selector lod-adaptive

lod-selector: src/lod.c
	$(CC) $(CFLAGS) $< -o $@ -lm

lod-adaptive: src/lod_adaptive.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: lod-selector lod-adaptive
	./lod-selector
	./lod-adaptive

clean:
	rm -f lod-selector lod-adaptive
