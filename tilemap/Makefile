CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: tilemap

tilemap: src/tilemap.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: tilemap
	./tilemap

clean:
	rm -f tilemap
