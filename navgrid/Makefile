CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -I../gfault/include
GFAULT_SRC := ../gfault/src/gfault.c

all: mini-navgrid

mini-navgrid: src/navgrid.c $(GFAULT_SRC)
	$(CC) $(CFLAGS) src/navgrid.c $(GFAULT_SRC) -o $@ -lm

test: mini-navgrid
	./mini-navgrid

clean:
	rm -f mini-navgrid
