CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mini-tangent

mini-tangent: src/tangent.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: mini-tangent
	./mini-tangent

clean:
	rm -f mini-tangent
