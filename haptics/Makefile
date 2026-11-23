CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: haptics

haptics: src/haptics.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: haptics
	./haptics

clean:
	rm -f haptics
