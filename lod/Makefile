CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: lod-selector

lod-selector: src/lod.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: lod-selector
	./lod-selector

imgpipe: src/imgpipe.c ../imgbuf/src/imgbuf.c ../imgbuf/src/imgbuf.h
	$(CC) $(CFLAGS) -I../imgbuf/src src/imgpipe.c ../imgbuf/src/imgbuf.c -o $@ -lm

bench: imgpipe
	./imgpipe

clean:
	rm -f lod-selector imgpipe
