CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
CPPFLAGS += -I../renderdiag/src

all: particle-pool

particle-pool: src/particle.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -o $@ -lm

test: particle-pool
	./particle-pool

clean:
	rm -f particle-pool
