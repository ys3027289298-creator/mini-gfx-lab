CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: aniso-filter

aniso-filter: src/aniso.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: aniso-filter
	mkdir -p ../artifacts/aniso
	./aniso-filter

clean:
	rm -f aniso-filter
