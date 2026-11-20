CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: blockpack

blockpack: src/blockpack.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: blockpack
	./blockpack

asan: src/blockpack.c
	$(CC) -O1 -g -std=c11 -Wall -Wextra -fsanitize=address,undefined $< -o blockpack_asan -lm
	./blockpack_asan

clean:
	rm -f blockpack blockpack_asan
