CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: animcomp

animcomp: src/animcomp.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: animcomp
	./animcomp

clean:
	rm -f animcomp
