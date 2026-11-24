CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -I../gfault/include
GFAULT_SRC := ../gfault/src/gfault.c

all: particle-pool

particle-pool: src/particle.c $(GFAULT_SRC)
	$(CC) $(CFLAGS) src/particle.c $(GFAULT_SRC) -o $@ -lm

test: particle-pool
	./particle-pool

clean:
	rm -f particle-pool
