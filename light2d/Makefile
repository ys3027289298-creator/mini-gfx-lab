CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: light2d

light2d: src/light2d.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: light2d
	./light2d

clean:
	rm -f light2d
