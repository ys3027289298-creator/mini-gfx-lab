CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: screen-cull

screen-cull: src/cull.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: screen-cull
	./screen-cull

clean:
	rm -f screen-cull
