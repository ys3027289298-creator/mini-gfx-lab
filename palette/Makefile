CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
GFAULT_SRC := ../gfault/src/gfault.c
RENDER_SRC := ../gfault/render/framebuf.c

all: palette-selftest

palette-selftest: src/selftest.c src/palette.c $(GFAULT_SRC) $(RENDER_SRC)
	$(CC) $(CFLAGS) -Iinclude -I../gfault/include -I../gfault/render src/selftest.c src/palette.c \
		$(GFAULT_SRC) $(RENDER_SRC) -o $@ -lm

test: palette-selftest
	./palette-selftest

clean:
	rm -f palette-selftest
