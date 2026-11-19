CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: conv

conv: src/conv.c src/conv.h
	$(CC) $(CFLAGS) $< -o $@ -lm

test: conv
	./conv

clean:
	rm -f conv
