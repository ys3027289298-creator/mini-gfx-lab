CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
CPPFLAGS += -I../renderdiag/src

all: mini-spatial

mini-spatial: src/spatial.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -o $@ -lm

test: mini-spatial
	./mini-spatial

clean:
	rm -f mini-spatial
