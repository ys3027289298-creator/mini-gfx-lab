CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: tilemap

tilemap: src/tilemap.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: tilemap
	./tilemap

decpipe: src/decpipe.c ../decodecache/src/dc_codec.c ../decodecache/src/dc_codec.h
	$(CC) $(CFLAGS) -I../decodecache/src src/decpipe.c ../decodecache/src/dc_codec.c -o $@ -lm

bench: decpipe
	./decpipe

clean:
	rm -f tilemap decpipe
