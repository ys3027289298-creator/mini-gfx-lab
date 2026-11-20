CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: motion-blur

motion-blur: src/motionblur.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: motion-blur
	./motion-blur

clean:
	rm -f motion-blur
