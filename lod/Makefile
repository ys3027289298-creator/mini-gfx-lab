CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: lod-selector

# shipping build: on-disk variant load with validation and fallback
lod-selector: src/lod.c
	$(CC) $(CFLAGS) -DLOD_ENABLE_LOAD $< -o $@ -lm

# load path compiled out: must behave exactly like the original selector
lod-selector-no-load: src/lod.c
	$(CC) $(CFLAGS) -ULOD_ENABLE_LOAD $< -o $@ -lm

test: lod-selector lod-selector-no-load
	./lod-selector
	@ref=/tmp/lod_ref.$$$$; \
	git show HEAD:lod/src/lod.c > $$ref.c && \
	$(CC) $(CFLAGS) $$ref.c -o $$ref -lm && \
	$$ref > $$ref.out && \
	./lod-selector-no-load > $$ref.noload && \
	cmp $$ref.out $$ref.noload && \
	echo no_load_byte_identical=1; \
	rc=$$?; rm -f $$ref $$ref.c $$ref.out $$ref.noload; exit $$rc

clean:
	rm -f lod-selector lod-selector-no-load
