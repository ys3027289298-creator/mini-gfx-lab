CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: sky-dome

sky-dome: src/sky.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: sky-dome
	./sky-dome

clean:
	rm -f sky-dome
