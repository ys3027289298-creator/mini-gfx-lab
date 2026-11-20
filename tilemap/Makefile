CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
CPPFLAGS += -I../renderdiag/src

all: tilemap

tilemap: src/tilemap.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -o $@ -lm

test: tilemap
	./tilemap

clean:
	rm -f tilemap
