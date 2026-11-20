CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: ccd

ccd: src/ccd.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: ccd
	./ccd

clean:
	rm -f ccd
