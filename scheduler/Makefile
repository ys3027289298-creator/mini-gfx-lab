CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: scheduler

scheduler: src/scheduler.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: scheduler
	./scheduler

clean:
	rm -f scheduler
