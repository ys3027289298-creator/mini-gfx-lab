CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: netsync

netsync: src/netsync.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: netsync
	./netsync

clean:
	rm -f netsync
