CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

BINS = particle-pool trail-history

all: $(BINS)

particle-pool: src/particle.c
	$(CC) $(CFLAGS) $< -o $@ -lm

trail-history: src/trail.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: $(BINS)
	./particle-pool
	./trail-history

clean:
	rm -f $(BINS)
