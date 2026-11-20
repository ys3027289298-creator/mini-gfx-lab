CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: shadow-pipeline

shadow-pipeline: src/shadowmap.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: shadow-pipeline
	./shadow-pipeline

clean:
	rm -f shadow-pipeline
