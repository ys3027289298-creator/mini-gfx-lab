CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: fontatlas

fontatlas: src/fontatlas.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: fontatlas
	./fontatlas

imgpipe: src/imgpipe.c ../imgbuf/src/imgbuf.c ../imgbuf/src/imgbuf.h
	$(CC) $(CFLAGS) -I../imgbuf/src src/imgpipe.c ../imgbuf/src/imgbuf.c -o $@ -lm

bench: imgpipe
	./imgpipe

clean:
	rm -f fontatlas imgpipe
