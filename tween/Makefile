CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: tween

tween: src/tween.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: tween
	./tween

clean:
	rm -f tween
