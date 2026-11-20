CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
SRC := src/pf_grid.c src/pf_heap.c src/pf_astar.c src/pf_lpa.c src/pf_hpa.c \
       src/pf_solver.c src/test_pf.c
BIN := test-pf

all: $(BIN)

$(BIN): $(SRC) src/pf.h src/pf_internal.h
	$(CC) $(CFLAGS) $(SRC) -o $@ -lm

bench: $(BIN)
	./$(BIN) bench

test: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)

.PHONY: all test bench clean
