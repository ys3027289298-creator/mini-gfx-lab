CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mini-navgrid

mini-navgrid: src/navgrid.c src/gridhash.c src/gridhash.h
	$(CC) $(CFLAGS) src/navgrid.c src/gridhash.c -o $@ -lm

gridhash_test: src/gridhash_test.c src/gridhash.c src/gridhash.h
	$(CC) $(CFLAGS) src/gridhash_test.c src/gridhash.c -o $@ -lm

test: mini-navgrid gridhash_test
	./mini-navgrid
	./gridhash_test

clean:
	rm -f mini-navgrid gridhash_test
