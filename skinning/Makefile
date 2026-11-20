CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: skinning

skinning: src/skinning.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: skinning
	./skinning

clean:
	rm -f skinning
