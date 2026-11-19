CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: stuck

stuck: src/stuck.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: stuck
	./stuck

clean:
	rm -f stuck
