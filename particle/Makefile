CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: particle-pool

particle-pool: src/particle.c ../diag/diag.h
	$(CC) $(CFLAGS) -I../diag $< -o $@ -lm

diag-bin: src/particle.c ../diag/diag.h
	$(CC) $(CFLAGS) -DDIAG=1 -I../diag $< -o $@ -lm

diag: diag-bin

diag-test: diag-bin
	./diag-bin

test: particle-pool
	./particle-pool

clean:
	rm -f particle-pool diag-bin
