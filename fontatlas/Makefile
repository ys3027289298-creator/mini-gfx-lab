CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: fontatlas

fontatlas: src/fontatlas.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: fontatlas
	./fontatlas

decpipe: src/decpipe.c ../decodecache/src/dc_codec.c ../decodecache/src/dc_codec.h
	$(CC) $(CFLAGS) -I../decodecache/src src/decpipe.c ../decodecache/src/dc_codec.c -o $@ -lm

bench: decpipe
	./decpipe

clean:
	rm -f fontatlas decpipe
