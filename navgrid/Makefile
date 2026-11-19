CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mini-navgrid regions-test

mini-navgrid: src/navgrid.c
	$(CC) $(CFLAGS) $< -o $@ -lm

regions-test: tests/regions_test.c src/regions.c src/regions.h
	$(CC) $(CFLAGS) -Isrc tests/regions_test.c src/regions.c -o $@

test: mini-navgrid regions-test
	./mini-navgrid
	./regions-test

clean:
	rm -f mini-navgrid regions-test
