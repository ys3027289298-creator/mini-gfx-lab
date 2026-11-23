CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: tilemap

tilemap: src/tilemap.c ../diag/diag.h
	$(CC) $(CFLAGS) -I../diag $< -o $@ -lm

diag-bin: src/tilemap.c ../diag/diag.h
	$(CC) $(CFLAGS) -DDIAG=1 -I../diag $< -o $@ -lm

diag: diag-bin

diag-test: diag-bin
	./diag-bin

test: tilemap
	./tilemap

clean:
	rm -f tilemap diag-bin
