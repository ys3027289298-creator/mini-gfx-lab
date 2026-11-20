CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: busgraph

busgraph: src/busgraph.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: busgraph
	./busgraph

clean:
	rm -f busgraph
