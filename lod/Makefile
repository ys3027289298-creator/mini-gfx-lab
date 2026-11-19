CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: lod-selector lod-pipeline

lod-selector: src/lod.c
	$(CC) $(CFLAGS) $< -o $@ -lm

lod-pipeline: src/lod_pipeline.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: lod-selector lod-pipeline
	./lod-selector
	./lod-pipeline

clean:
	rm -f lod-selector lod-pipeline
