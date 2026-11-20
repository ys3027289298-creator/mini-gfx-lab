CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mini-spatial

mini-spatial: src/spatial.c src/reference_data.h
	$(CC) $(CFLAGS) src/spatial.c -o $@ -lm

test: mini-spatial
	./mini-spatial

regen-reference:
	python3 -I tools/gen_reference.py > src/reference_data.h

clean:
	rm -f mini-spatial

.PHONY: all test regen-reference clean
