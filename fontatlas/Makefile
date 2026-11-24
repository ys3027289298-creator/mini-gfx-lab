CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -I../gfault/include
GFAULT_SRC := ../gfault/src/gfault.c

all: fontatlas

fontatlas: src/fontatlas.c $(GFAULT_SRC)
	$(CC) $(CFLAGS) src/fontatlas.c $(GFAULT_SRC) -o $@ -lm

test: fontatlas
	./fontatlas

clean:
	rm -f fontatlas
