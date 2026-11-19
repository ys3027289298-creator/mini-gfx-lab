CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
SIMENV := ../simenv

all: tween

tween: src/tween.c $(SIMENV)/src/simenv.c $(SIMENV)/include/simenv/simenv.h
	$(CC) $(CFLAGS) -I$(SIMENV)/include src/tween.c $(SIMENV)/src/simenv.c -o $@ -lm

test: tween
	./tween

clean:
	rm -f tween
