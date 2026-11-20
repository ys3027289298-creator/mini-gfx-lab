CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: levelstream

levelstream: src/levelstream.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: levelstream
	./levelstream

clean:
	rm -f levelstream
