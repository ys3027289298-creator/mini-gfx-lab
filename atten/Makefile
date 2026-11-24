CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: atten

atten: src/atten.c
	$(CC) $(CFLAGS) $< -o $@ -lm

out:
	mkdir -p out

test: atten out
	./atten

images: atten out
	./atten >/dev/null
	python3 tools/compose.py

clean:
	rm -f atten
	rm -rf out

.PHONY: all test images clean
