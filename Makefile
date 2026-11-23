# 共享图像缓冲接入的四处：缺任一处数据都算失败
IMG_MODULES := tilemap fontatlas particle lod

bench:
	@status=0; \
	for m in $(IMG_MODULES); do \
	  out=$$( $(MAKE) -s -C $$m bench 2>&1 ); \
	  rc=$$?; \
	  echo "$$out"; \
	  if [ $$rc -ne 0 ]; then echo "$$m: bench run failed"; status=1; continue; fi; \
	  for want in "module=$$m" "mode=shared peak_bytes=" "mode=private peak_bytes=" \
	              "elapsed_us=" "identical=1" "peak_ok=1" "no_second_copy=1" \
	              "error_reported=1" "all_ok=1"; do \
	    echo "$$out" | grep -q "$$want" || { echo "$$m: MISSING '$$want'"; status=1; }; \
	  done; \
	done; \
	if [ $$status -eq 0 ]; then echo "bench_complete=1"; else echo "bench_complete=0"; fi; \
	exit $$status

test:
	$(MAKE) -C imgbuf test

.PHONY: bench test
