# Shared build rules for the mini-gfx-lab sub-projects.
# Each module Makefile sets: NAME, MODULE, SRC, HDRS.

CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
CPPFLAGS += -I../common -Isrc
COMMON_SRC := ../common/diag.c ../common/failpoints.c ../common/canvas.c
COMMON_HDR := ../common/diag.h ../common/failpoints.h ../common/canvas.h \
              ../common/testharness.h ../common/acctest.h ../common/allocwatch.h

TESTBIN := tests/$(NAME)_tests
ACCBIN  := tests/$(NAME)_accept

# The module sources carry their CLI demos behind *_NO_MAIN; the test binary
# links its own main.  The demos are the golden-output programs.
NO_MAIN := -DPARTICLE_NO_MAIN -DTILEMAP_NO_MAIN -DNAVGRID_NO_MAIN -DSPATIAL_NO_MAIN

# Allocator wrapping lets the suite prove that disabled diagnostics perform
# zero heap traffic.
WRAP_LDFLAGS := -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=realloc -Wl,--wrap=free

.PHONY: all test clean check-failpoints acceptance

all: $(NAME)

$(NAME): $(SRC) $(HDRS) $(COMMON_HDR)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(SRC) $(COMMON_SRC) -o $@ -lm

# --- acceptance harness ------------------------------------------------------
$(TESTBIN): tests/harness.c $(SRC) $(HDRS) $(COMMON_HDR) \
            ../common/testharness.c ../common/acctest.c ../common/allocwatch.c
	$(CC) $(CFLAGS) $(CPPFLAGS) -Itests \
	    tests/harness.c ../common/testharness.c ../common/acctest.c \
	    ../common/allocwatch.c $(SRC) $(COMMON_SRC) $(NO_MAIN) \
	    $(WRAP_LDFLAGS) -o $@ -lm

test: $(TESTBIN)
	./$(TESTBIN)

# Enumerate every DIAG_FAIL_RETURN site and require a matching harness case.
check-failpoints:
	$(PYTHON) ../tools/check_failpoints.py $(MODULE)

# One-command acceptance: suite, failpoint coverage, golden byte-equality.
acceptance: all test check-failpoints
	@$(PYTHON) ../tools/golden_check.py $(MODULE) ./$(NAME) tests/golden.stdout tests/golden.rc

clean:
	rm -f $(NAME) $(TESTBIN)
