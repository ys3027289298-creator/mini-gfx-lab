CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: renderdiag

renderdiag: src/renderdiag.c src/renderdiag.h
	$(CC) $(CFLAGS) $< -o $@

test: renderdiag
	./renderdiag
	sh check_coverage.sh

clean:
	rm -f renderdiag
