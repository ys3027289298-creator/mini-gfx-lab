CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: savemig

savemig: src/savemig.c
	$(CC) $(CFLAGS) $< -o $@

test: savemig
	./savemig

clean:
	rm -f savemig
