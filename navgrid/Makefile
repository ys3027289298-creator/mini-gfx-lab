CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mini-navgrid

mini-navgrid: src/navgrid.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: mini-navgrid
	./mini-navgrid

bench: mini-navgrid
	./mini-navgrid --bench

clean:
	rm -f mini-navgrid

.PHONY: all test bench clean
