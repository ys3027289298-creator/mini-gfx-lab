CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: linkage

linkage: src/linkage.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: linkage
	./linkage

clean:
	rm -f linkage
