CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: atlasstream

atlasstream: src/atlasstream.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: atlasstream
	./atlasstream

clean:
	rm -f atlasstream
