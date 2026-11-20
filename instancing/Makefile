CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
LDLIBS ?= -lm

all: instancing

instancing: src/instancing.c
	$(CC) $(CFLAGS) $< -o $@ $(LDLIBS)

test: instancing
	./instancing

clean:
	rm -f instancing

.PHONY: all test clean
