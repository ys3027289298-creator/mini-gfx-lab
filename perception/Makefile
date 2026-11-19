CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mini-perception

mini-perception: src/perception.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: mini-perception
	./mini-perception

clean:
	rm -f mini-perception
