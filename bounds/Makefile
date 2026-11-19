CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: bounds-inc

bounds-inc: src/bounds.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: bounds-inc
	./bounds-inc

clean:
	rm -f bounds-inc
