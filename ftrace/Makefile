CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: ftrace

ftrace: src/ftrace.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: ftrace
	./ftrace

clean:
	rm -f ftrace
