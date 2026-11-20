CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: tilemap

tilemap: src/tilemap.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: tilemap
	./tilemap

check: tilemap
	python3 -I check_tilemap.py

clean:
	rm -f tilemap

.PHONY: all test check clean
