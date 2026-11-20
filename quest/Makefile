CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: quest

quest: src/quest.c
	$(CC) $(CFLAGS) $< -o $@

test: quest
	./quest

clean:
	rm -f quest
