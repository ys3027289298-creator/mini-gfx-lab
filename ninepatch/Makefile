CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: ninepatch

ninepatch: src/ninepatch.c
	$(CC) $(CFLAGS) $< -o $@

test: ninepatch
	./ninepatch

clean:
	rm -f ninepatch
