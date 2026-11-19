CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
LDLIBS ?= -lm

all: ems-repl ems-test ems-bench

ems-repl: src/ems.c src/repl.c src/ems.h
	$(CC) $(CFLAGS) -Isrc src/ems.c src/repl.c -o $@ $(LDLIBS)

ems-test: src/ems.c src/test.c src/ref.h src/ems.h
	$(CC) $(CFLAGS) -Isrc src/ems.c src/test.c -o $@ $(LDLIBS)

ems-bench: src/ems.c src/bench.c src/ems.h
	$(CC) $(CFLAGS) -Isrc src/ems.c src/bench.c -o $@ $(LDLIBS)

test: ems-test
	./ems-test examples

bench: ems-bench
	./ems-bench examples

clean:
	rm -f ems-repl ems-test ems-bench

.PHONY: all test bench clean
