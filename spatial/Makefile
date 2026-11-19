CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
HDRS = src/binaural.h src/spatial_quant.h

all: mini-spatial

mini-spatial: src/spatial.c $(HDRS)
	$(CC) $(CFLAGS) $< -o $@ -lm

test: mini-spatial
	./mini-spatial

clean:
	rm -f mini-spatial
