CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: transsort

transsort: src/transsort.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: transsort
	./transsort

clean:
	rm -f transsort
