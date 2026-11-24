CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -I../gfault/include
GFAULT_SRC := ../gfault/src/gfault.c

all: tilemap

tilemap: src/tilemap.c $(GFAULT_SRC)
	$(CC) $(CFLAGS) src/tilemap.c $(GFAULT_SRC) -o $@ -lm

test: tilemap
	./tilemap

clean:
	rm -f tilemap
