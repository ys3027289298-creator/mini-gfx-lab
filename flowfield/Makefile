CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mini-flowfield

mini-flowfield: src/flowfield.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: mini-flowfield
	./mini-flowfield

clean:
	rm -f mini-flowfield
