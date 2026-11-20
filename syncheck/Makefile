CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: syncheck

syncheck: src/syncheck.c
	$(CC) $(CFLAGS) $< -o $@

test: syncheck
	./syncheck

clean:
	rm -f syncheck
