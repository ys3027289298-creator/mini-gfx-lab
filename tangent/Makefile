CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: tangent-basis

tangent-basis: src/tangent.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: tangent-basis
	./tangent-basis

clean:
	rm -f tangent-basis
