CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: modifier

modifier: src/modifier.c
	$(CC) $(CFLAGS) $< -o $@

test: modifier
	./modifier

clean:
	rm -f modifier
