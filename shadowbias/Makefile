CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: shadow-bias

shadow-bias: src/shadowbias.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: shadow-bias
	./shadow-bias

dump: shadow-bias
	./shadow-bias dump

verify:
	python3 tools/verify_ppm.py

clean:
	rm -f shadow-bias
	rm -rf out

.PHONY: all test dump verify clean
