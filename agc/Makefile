CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Wshadow -Wconversion

all: agc

agc: src/agc.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: agc
	./agc

clean:
	rm -f agc
