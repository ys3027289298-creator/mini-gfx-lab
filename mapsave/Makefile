CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mapsave

mapsave: src/mapsave.c
	$(CC) $(CFLAGS) $< -o $@

test: mapsave
	./mapsave

clean:
	rm -f mapsave mapsave_tmp.mgms
