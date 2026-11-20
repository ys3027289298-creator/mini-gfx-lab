CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: pyramid-demo

pyramid-demo: src/pyramid.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: pyramid-demo
	./pyramid-demo

clean:
	rm -f pyramid-demo
