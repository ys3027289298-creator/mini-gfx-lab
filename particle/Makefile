CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: particle-pool

particle-pool: src/particle.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: particle-pool
	./particle-pool

decpipe: src/decpipe.c ../decodecache/src/dc_codec.c ../decodecache/src/dc_codec.h
	$(CC) $(CFLAGS) -I../decodecache/src src/decpipe.c ../decodecache/src/dc_codec.c -o $@ -lm

bench: decpipe
	./decpipe

clean:
	rm -f particle-pool decpipe
