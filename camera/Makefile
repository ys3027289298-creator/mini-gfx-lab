CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: camera

camera: src/camera.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: camera
	./camera

clean:
	rm -f camera
