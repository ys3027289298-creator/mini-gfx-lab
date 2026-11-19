CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: fontatlas

fontatlas: src/fontatlas.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: fontatlas
	./fontatlas

clean:
	rm -f fontatlas
