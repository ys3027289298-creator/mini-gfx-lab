CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: clockdrift

clockdrift: src/clockdrift.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: clockdrift
	./clockdrift

clean:
	rm -f clockdrift
