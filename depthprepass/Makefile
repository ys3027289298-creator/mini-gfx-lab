CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: depthprepass

depthprepass: src/depthprepass.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: depthprepass
	./depthprepass

clean:
	rm -f depthprepass
