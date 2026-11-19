CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: animlayer

animlayer: src/animlayer.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: animlayer
	./animlayer

clean:
	rm -f animlayer
