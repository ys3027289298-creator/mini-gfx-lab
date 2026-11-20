CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: matbatch

matbatch: src/matbatch.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: matbatch
	./matbatch

clean:
	rm -f matbatch
