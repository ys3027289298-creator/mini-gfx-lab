CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: fontfallback

fontfallback: src/fontfallback.c
	$(CC) $(CFLAGS) $< -o $@

test: fontfallback
	./fontfallback

clean:
	rm -f fontfallback
