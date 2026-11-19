CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: airabs

airabs: src/airabs.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: airabs
	./airabs

clean:
	rm -f airabs
