CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: scene-graph

scene-graph: src/scenegraph.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: scene-graph
	./scene-graph

clean:
	rm -f scene-graph
