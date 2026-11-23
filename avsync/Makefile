CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: avsync

avsync: src/avsync.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: avsync
	./avsync --selftest

clean:
	rm -f avsync
