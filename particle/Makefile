CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: particle-pool

particle-pool: src/particle.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: particle-pool
	./particle-pool
	./particle-pool replay

clean:
	rm -f particle-pool
