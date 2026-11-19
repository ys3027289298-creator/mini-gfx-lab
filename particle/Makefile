CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
SIMENV := ../simenv

all: particle-pool

particle-pool: src/particle.c $(SIMENV)/src/simenv.c $(SIMENV)/include/simenv/simenv.h
	$(CC) $(CFLAGS) -I$(SIMENV)/include src/particle.c $(SIMENV)/src/simenv.c -o $@ -lm

test: particle-pool
	./particle-pool

clean:
	rm -f particle-pool
