CC ?= gcc
# Determinism flags:
#   -ffp-contract=off    forbid fusing mul+add into FMA (changes rounding;
#                        clang enables it by default even at -std=c11)
#   -fno-fast-math       explicit: fast-math/associative reordering is never
#                        allowed for this target
#   -fexcess-precision=standard  no x87-style extended temporaries
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -ffp-contract=off -fno-fast-math -fexcess-precision=standard

all: mixer

mixer: src/mixer.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: mixer
	./mixer

clean:
	rm -f mixer
