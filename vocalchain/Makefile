CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: vocalchain

vocalchain: src/vocalchain.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: vocalchain
	./vocalchain

clean:
	rm -f vocalchain
