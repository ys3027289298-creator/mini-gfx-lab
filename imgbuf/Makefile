CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: imgbuf_test

imgbuf_test: src/imgbuf_test.c src/imgbuf.c src/imgbuf.h
	$(CC) $(CFLAGS) -Isrc src/imgbuf_test.c src/imgbuf.c -o $@

test: imgbuf_test
	./imgbuf_test

clean:
	rm -f imgbuf_test
