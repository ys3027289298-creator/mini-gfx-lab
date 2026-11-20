CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: paramcache

paramcache: src/paramcache.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: paramcache
	./paramcache

clean:
	rm -f paramcache
