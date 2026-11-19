CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

BINS = tween fsm

all: $(BINS)

tween: src/tween.c
	$(CC) $(CFLAGS) $< -o $@ -lm

fsm: src/fsm.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: $(BINS)
	./tween
	./fsm

clean:
	rm -f $(BINS)
