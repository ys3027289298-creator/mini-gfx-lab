CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: decal-pool

decal-pool: src/decal.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: decal-pool
	./decal-pool

clean:
	rm -f decal-pool
