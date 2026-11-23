CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mini-navgrid

mini-navgrid: src/navgrid.c ../diag/diag.h
	$(CC) $(CFLAGS) -I../diag $< -o $@ -lm

diag-bin: src/navgrid.c ../diag/diag.h
	$(CC) $(CFLAGS) -DDIAG=1 -I../diag $< -o $@ -lm

diag: diag-bin

diag-test: diag-bin
	./diag-bin

test: mini-navgrid
	./mini-navgrid

clean:
	rm -f mini-navgrid diag-bin
