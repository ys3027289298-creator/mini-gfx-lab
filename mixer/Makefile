CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: mixer pipeline pipeline_unify

mixer: src/mixer.c
	$(CC) $(CFLAGS) $< -o $@ -lm

# legacy export chain: per-stage rates 8000/44100/22050/48000, phase reset
# at each 2 s block boundary (reproduces the periodic clicks)
pipeline: src/pipeline.c
	$(CC) $(CFLAGS) $< -o $@ -lm

# unified export chain: all four stages at 48000 Hz; the legacy code path is
# still compiled in, only the rate-table selection changes
pipeline_unify: src/pipeline.c
	$(CC) $(CFLAGS) -DMIXER_UNIFY_RATE=1 $< -o $@ -lm

test: mixer
	./mixer

pipeline-test: pipeline pipeline_unify
	@mkdir -p ../artifacts/mixer/legacy ../artifacts/mixer/unified
	./pipeline ../artifacts/mixer/legacy
	./pipeline_unify ../artifacts/mixer/unified

# four verification materials: click counts, waveform summary, per-stage
# spectrograms, sample-rate/byte-identity verification
analysis: pipeline-test
	python3 -I ../artifacts/scripts/click_count.py
	python3 -I ../artifacts/scripts/waveform_summary.py
	python3 -I ../artifacts/scripts/spectrograms.py
	python3 -I ../artifacts/scripts/verify_stages.py

clean:
	rm -f mixer pipeline pipeline_unify
