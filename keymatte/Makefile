CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: keymatte

keymatte: src/keymatte.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: keymatte
	./keymatte

clean:
	rm -f keymatte
