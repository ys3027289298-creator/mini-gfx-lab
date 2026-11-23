# 解码缓存接入的四处：缺任一处数据都算失败
DC_MODULES := tilemap fontatlas particle lod

bench:
	@status=0; \
	for m in $(DC_MODULES); do \
	  out=$$( $(MAKE) -s -C $$m bench 2>&1 ); \
	  rc=$$?; \
	  echo "$$out"; \
	  if [ $$rc -ne 0 ]; then echo "$$m: bench run failed"; status=1; continue; fi; \
	  for want in "module=$$m" "on_zero_redecode=1" "off_decodes_every_frame=1" \
	              "identical=1" "pixel_hash:" "hitlog:" "decode_time:" \
	              "repro=make -C $$m bench" "all_ok=1"; do \
	    echo "$$out" | grep -q "$$want" || { echo "$$m: MISSING '$$want'"; status=1; }; \
	  done; \
	done; \
	if [ $$status -eq 0 ]; then echo "bench_complete=1"; else echo "bench_complete=0"; fi; \
	exit $$status

test:
	$(MAKE) -C decodecache test

evidence:
	./decodecache/scripts/collect_evidence.sh
	./decodecache/scripts/check_evidence.sh

.PHONY: bench test evidence
