CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: footstep

footstep: src/footstep.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: footstep
	./footstep

report: footstep
	mkdir -p out
	./footstep --report
	sha256sum out/mix_align_off.f32 out/mix_align_on.f32 | tee out/sha256.txt

clean:
	rm -f footstep
	rm -rf out

.PHONY: all test report clean
