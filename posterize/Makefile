CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: posterize

posterize: src/posterize.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: posterize
	./posterize

clean:
	rm -f posterize
