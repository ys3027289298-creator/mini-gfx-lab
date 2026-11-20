CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: tri-budget

tri-budget: src/budget.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: tri-budget
	./tri-budget

clean:
	rm -f tri-budget
