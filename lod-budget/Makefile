CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: lod-budget

lod-budget: src/lod_budget.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: lod-budget
	./lod-budget

clean:
	rm -f lod-budget
