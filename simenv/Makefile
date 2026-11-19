CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
IDIR := include
MODULES := ../particle ../tween ../mixer ../spatial ../lod ../navgrid ../tilemap ../fontatlas

all: simenv-selftest replay-demo overhead-bench dispatch-micro tape-cross

simenv-selftest: tests/selftest.c src/simenv.c $(IDIR)/simenv/simenv.h
	$(CC) $(CFLAGS) -I$(IDIR) tests/selftest.c src/simenv.c -o $@ -lm

replay-demo: tests/replay-demo.c src/simenv.c $(IDIR)/simenv/simenv.h
	$(CC) $(CFLAGS) -I$(IDIR) tests/replay-demo.c src/simenv.c -o $@ -lm

overhead-bench: tests/overhead-bench.c tests/bench_iface.c tests/bench_direct.c src/simenv.c $(IDIR)/simenv/simenv.h
	$(CC) $(CFLAGS) -Wno-unused-parameter -I$(IDIR) tests/overhead-bench.c tests/bench_iface.c tests/bench_direct.c src/simenv.c -o $@ -lm

dispatch-micro: tests/dispatch-micro.c tests/bench_iface.c src/simenv.c $(IDIR)/simenv/simenv.h
	$(CC) $(CFLAGS) -I$(IDIR) tests/dispatch-micro.c tests/bench_iface.c src/simenv.c -o $@ -lm

tape-cross: tests/tape-cross.c src/simenv.c $(IDIR)/simenv/simenv.h
	$(CC) $(CFLAGS) -Wno-unused-parameter -I$(IDIR) tests/tape-cross.c src/simenv.c -o $@ -lm

# guard: module sources must not reach for time/randomness directly.
# Only simenv's own implementation may call the platform APIs.
guard:
	@bad=0; for d in $(MODULES); do \
	  if grep -nE '\b(rand|srand|random|srandom|drand48|clock_gettime|gettimeofday|time)\s*\(' $$d/src/*.c | grep -vE 'sim_(env_)?(now|dt|rand)'; then \
	    echo "GUARD FAIL: direct time/random API in $$d"; bad=1; fi; \
	done; exit $$bad

test: all guard
	./simenv-selftest
	./replay-demo
	./overhead-bench
	./dispatch-micro
	./tape-cross write /tmp/tc.tape > /tmp/tc_write.txt
	./tape-cross read  /tmp/tc.tape > /tmp/tc_read.txt
	diff /tmp/tc_write.txt /tmp/tc_read.txt && echo "cross_process_tape=1"

clean:
	rm -f simenv-selftest replay-demo overhead-bench dispatch-micro tape-cross
