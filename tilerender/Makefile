CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: tilerender

tilerender: src/tilerender.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: tilerender
	./tilerender

clean:
	rm -f tilerender
