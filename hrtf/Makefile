CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: hrtf

hrtf: src/hrtf.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: hrtf
	./hrtf

clean:
	rm -f hrtf

.PHONY: all test clean
