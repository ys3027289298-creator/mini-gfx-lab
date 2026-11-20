CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mini-orca

mini-orca: src/orca.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: mini-orca
	./mini-orca

clean:
	rm -f mini-orca
