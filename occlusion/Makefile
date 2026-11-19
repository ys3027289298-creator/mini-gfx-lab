CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: occlusion-cull

occlusion-cull: src/occlusion.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: occlusion-cull
	./occlusion-cull

clean:
	rm -f occlusion-cull
