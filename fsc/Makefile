CC ?= gcc
# -ffp-contract=off keeps the reference and streaming float paths in the same
# operation order; without it FMA contraction can differ across call sites.
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -ffp-contract=off

all: fsc

fsc: src/fsc.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: fsc
	./fsc

clean:
	rm -f fsc
