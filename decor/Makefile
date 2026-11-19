CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: decor

decor: src/decor.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: decor
	./decor

clean:
	rm -f decor
