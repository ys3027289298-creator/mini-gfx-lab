CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: skillsched

skillsched: src/skillsched.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: skillsched
	./skillsched

clean:
	rm -f skillsched
