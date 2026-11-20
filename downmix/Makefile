CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: downmix

downmix: src/downmix.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: downmix
	./downmix

clean:
	rm -f downmix
