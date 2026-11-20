CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mini-navgrid

mini-navgrid: src/navgrid.c src/navpath.c src/navpath.h
	$(CC) $(CFLAGS) src/navgrid.c src/navpath.c -o $@ -lm

test: mini-navgrid
	./mini-navgrid

clean:
	rm -f mini-navgrid
