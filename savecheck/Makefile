CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: savecheck

savecheck: src/savecheck.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: savecheck
	./savecheck

clean:
	rm -f savecheck
