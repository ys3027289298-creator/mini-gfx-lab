CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: fontatlas

fontatlas: src/fontatlas.c ../diag/diag.h
	$(CC) $(CFLAGS) -I../diag $< -o $@ -lm

diag-bin: src/fontatlas.c ../diag/diag.h
	$(CC) $(CFLAGS) -DDIAG=1 -I../diag $< -o $@ -lm

diag: diag-bin

diag-test: diag-bin
	./diag-bin

test: fontatlas
	./fontatlas

clean:
	rm -f fontatlas diag-bin
