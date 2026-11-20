CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: palette

palette: src/palette.c
	$(CC) $(CFLAGS) $< -o $@

test: palette
	./palette

clean:
	rm -f palette
