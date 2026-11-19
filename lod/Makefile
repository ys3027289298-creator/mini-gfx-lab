CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: lod-selector bbox-check

lod-selector: src/lod.c
	$(CC) $(CFLAGS) $< -o $@ -lm

bbox-check: src/bbox.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: lod-selector bbox-check
	./lod-selector
	./bbox-check

clean:
	rm -f lod-selector bbox-check
