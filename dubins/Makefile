CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mini-dubins

mini-dubins: src/dubins.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: mini-dubins
	./mini-dubins

clean:
	rm -f mini-dubins
