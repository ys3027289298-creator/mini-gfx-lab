CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mini-lightprobe

mini-lightprobe: src/lightprobe.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: mini-lightprobe
	./mini-lightprobe

clean:
	rm -f mini-lightprobe
