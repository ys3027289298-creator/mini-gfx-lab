CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mini-spatial render_chain

mini-spatial: src/spatial.c
	$(CC) $(CFLAGS) $< -o $@ -lm

render_chain: src/render_chain.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: mini-spatial
	./mini-spatial

chain-test: render_chain
	./render_chain

clean:
	rm -f mini-spatial render_chain
