CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: lod-selector

lod-selector: src/lod.c
	$(CC) $(CFLAGS) $< -o $@ -lm

lod-verify: src/lod_verify.c src/lod.c
	$(CC) $(CFLAGS) src/lod_verify.c -o $@ -lm

test: lod-selector lod-verify
	./lod-selector
	./lod-verify

verify: lod-verify
	./lod-verify

clean:
	rm -f lod-selector lod-verify
