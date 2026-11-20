CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: propagation

propagation: src/propagation.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: propagation
	./propagation

clean:
	rm -f propagation
