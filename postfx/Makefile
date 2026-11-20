CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: postfx

postfx: src/postfx.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: postfx
	./postfx

clean:
	rm -f postfx
