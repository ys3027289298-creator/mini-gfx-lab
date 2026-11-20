CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: texcomp

texcomp: src/texcomp.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: texcomp
	./texcomp

clean:
	rm -f texcomp
