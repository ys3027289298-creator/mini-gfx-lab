CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: parallax-pom

parallax-pom: src/parallax.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: parallax-pom
	./parallax-pom

clean:
	rm -f parallax-pom
