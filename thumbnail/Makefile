CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: thumbnail-cache

thumbnail-cache: src/thumbnail.c
	$(CC) $(CFLAGS) $< -o $@

test: thumbnail-cache
	./thumbnail-cache

clean:
	rm -f thumbnail-cache
