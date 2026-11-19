CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mini-navmesh

mini-navmesh: src/navmesh.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: mini-navmesh
	./mini-navmesh

clean:
	rm -f mini-navmesh
