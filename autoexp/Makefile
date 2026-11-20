CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: autoexp

autoexp: src/autoexp.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: autoexp
	./autoexp

clean:
	rm -f autoexp
