CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: replay

replay: src/replay.c
	$(CC) $(CFLAGS) $< -o $@

test: replay
	./replay

clean:
	rm -f replay replay_test.log replay_trunc.log replay_linear.log replay_linear_inputs.log
