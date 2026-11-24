CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: density-budget

density-budget: src/density.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: density-budget
	./density-budget

clean:
	rm -f density-budget
