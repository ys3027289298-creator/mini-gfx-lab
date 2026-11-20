CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: iconexport

iconexport: src/iconexport.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: iconexport
	./iconexport

clean:
	rm -f iconexport
