CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: rollback

rollback: src/rollback.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: rollback
	./rollback

metrics: rollback
	./rollback --csv > metrics.csv
	python3 tools/chart.py metrics.csv metrics.svg

clean:
	rm -f rollback metrics.csv metrics.svg
