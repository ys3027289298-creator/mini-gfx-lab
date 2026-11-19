CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: vquant

vquant: src/vquant.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: vquant
	./vquant

clean:
	rm -f vquant
