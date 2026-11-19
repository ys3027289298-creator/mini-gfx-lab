CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: boids

boids: src/boids.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: boids
	./boids

clean:
	rm -f boids
