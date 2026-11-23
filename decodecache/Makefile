CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: dc_test

dc_test: src/dc_test.c src/dc_codec.c src/dc_codec.h
	$(CC) $(CFLAGS) -Isrc src/dc_test.c src/dc_codec.c -o $@

test: dc_test
	./dc_test

clean:
	rm -f dc_test
