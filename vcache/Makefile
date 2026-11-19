CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: vcache-opt

vcache-opt: src/vcache.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: vcache-opt
	./vcache-opt

clean:
	rm -f vcache-opt
