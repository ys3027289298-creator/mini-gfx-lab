CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: moveval

moveval: src/moveval.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: moveval
	./moveval

clean:
	rm -f moveval
