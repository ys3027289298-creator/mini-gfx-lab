CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: aniso-spec

aniso-spec: src/aniso.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: aniso-spec
	./aniso-spec

clean:
	rm -f aniso-spec
