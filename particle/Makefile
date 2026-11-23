CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: particle-pool

particle-pool: src/particle.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: particle-pool
	./particle-pool

imgpipe: src/imgpipe.c ../imgbuf/src/imgbuf.c ../imgbuf/src/imgbuf.h
	$(CC) $(CFLAGS) -I../imgbuf/src src/imgpipe.c ../imgbuf/src/imgbuf.c -o $@ -lm

bench: imgpipe
	./imgpipe

clean:
	rm -f particle-pool imgpipe
