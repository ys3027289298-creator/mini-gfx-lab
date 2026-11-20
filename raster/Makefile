CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: raster

raster: src/raster.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: raster
	./raster

clean:
	rm -f raster
