CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: damage

damage: src/damage.c
	$(CC) $(CFLAGS) $< -o $@

test: damage
	./damage

clean:
	rm -f damage
