CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: normal-correct

normal-correct: src/normalmap.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: normal-correct
	./normal-correct

clean:
	rm -f normal-correct
