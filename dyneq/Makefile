CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: dyneq

dyneq: src/dyneq.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: dyneq
	./dyneq

clean:
	rm -f dyneq
