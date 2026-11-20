CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: volfog

volfog: src/volfog.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: volfog
	./volfog

clean:
	rm -f volfog
