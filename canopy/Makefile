CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: canopy

canopy: src/canopy.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: canopy
	./canopy

report: canopy
	./canopy canopy-report.md

clean:
	rm -f canopy canopy-report.md
