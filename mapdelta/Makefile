CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mapdelta

mapdelta: src/mapdelta.c
	$(CC) $(CFLAGS) $< -o $@

test: mapdelta
	./mapdelta

clean:
	rm -f mapdelta
