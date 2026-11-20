CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: ssaa-render

ssaa-render: src/ssaa.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: ssaa-render
	./ssaa-render

clean:
	rm -f ssaa-render
