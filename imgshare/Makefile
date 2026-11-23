CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror
CPPFLAGS ?= -Isrc

COMMON := src/imgshare.c
HEADERS := src/imgshare.h

all: imgshare-demo imgshare-tests

imgshare-demo: src/demo.c $(COMMON) $(HEADERS)
	$(CC) $(CPPFLAGS) $(CFLAGS) src/demo.c $(COMMON) -o $@

imgshare-tests: src/tests.c $(COMMON) $(HEADERS)
	$(CC) $(CPPFLAGS) $(CFLAGS) src/tests.c $(COMMON) -o $@

test: imgshare-tests
	./imgshare-tests

# Prints the peak-memory table, the copy-count log, the pixel hashes and the
# reproduce command for both modes.  Enforced by scripts/check_evidence.sh.
evidence: imgshare-demo
	bash scripts/collect_evidence.sh
	bash scripts/check_evidence.sh

clean:
	rm -f imgshare-demo imgshare-tests
	rm -rf evidence/*.txt

.PHONY: all test evidence clean
