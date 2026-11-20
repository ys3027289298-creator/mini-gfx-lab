CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: doppler

doppler: src/doppler.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: doppler
	./doppler

clean:
	rm -f doppler
