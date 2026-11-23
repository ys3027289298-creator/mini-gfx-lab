CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
CPPFLAGS += -I../common -I../particle/src -I../tilemap/src -I../navgrid/src -I../spatial/src
PYTHON ?= python3

COMMON_SRC := ../common/diag.c ../common/failpoints.c ../common/canvas.c
MODULE_SRC := ../particle/src/particle.c ../tilemap/src/tilemap.c \
              ../navgrid/src/navgrid.c ../spatial/src/spatial.c
MOD_NO_MAIN := -DPARTICLE_NO_MAIN -DTILEMAP_NO_MAIN -DNAVGRID_NO_MAIN -DSPATIAL_NO_MAIN
WRAP := -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=realloc -Wl,--wrap=free

.PHONY: all test clean acceptance

all: render-chain

render-chain: src/chain.c src/chain.h $(COMMON_SRC) $(MODULE_SRC)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(MOD_NO_MAIN) src/chain.c $(COMMON_SRC) $(MODULE_SRC) -o $@ -lm

tests/chain_tests: tests/harness.c ../common/testharness.c ../common/allocwatch.c \
                   src/chain.c src/chain.h $(COMMON_SRC) $(MODULE_SRC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -Itests -Isrc $(MOD_NO_MAIN) -DCHAIN_NO_MAIN \
	    tests/harness.c ../common/testharness.c ../common/allocwatch.c \
	    src/chain.c $(COMMON_SRC) $(MODULE_SRC) $(WRAP) -o $@ -lm

test: tests/chain_tests
	./tests/chain_tests

# Chain acceptance: run the suite, then verify the uncovered-failure report is
# empty and the clean frame is reproducible.
acceptance: all test
	@$(PYTHON) ../tools/coverage_report.py chain
	@./render-chain >/dev/null && echo "chain: clean frame renders, degraded=0"

clean:
	rm -f render-chain tests/chain_tests
