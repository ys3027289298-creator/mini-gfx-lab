CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: color-pipe

color-pipe: src/colorpipe.c src/colorpipe.h
	$(CC) $(CFLAGS) -Isrc $< -o $@ -lm

test: color-pipe
	./color-pipe --check-golden tests/golden.legacy

artifacts: color-pipe
	mkdir -p artifacts
	./color-pipe --artifacts artifacts --check-golden tests/golden.legacy

clean:
	rm -f color-pipe
	rm -rf artifacts

.PHONY: all test artifacts clean
