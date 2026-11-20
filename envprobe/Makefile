CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: envprobe

envprobe: src/envprobe.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: envprobe
	./envprobe

clean:
	rm -f envprobe
