CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: inputbuf

inputbuf: src/inputbuf.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: inputbuf
	./inputbuf

clean:
	rm -f inputbuf
