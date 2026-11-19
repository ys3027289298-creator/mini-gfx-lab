CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror

all: streammap

streammap: src/streammap.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: streammap
	./streammap

clean:
	rm -f streammap
