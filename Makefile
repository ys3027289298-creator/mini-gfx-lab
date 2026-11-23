MODULES := navgrid spatial particle lod tilemap mixer fontatlas tween
DIAG_MODULES := particle tilemap navgrid fontatlas

# 普通自检：逐模块跑，失败不中断，最后汇总
test:
	@status=0; for m in $(MODULES); do \
		if $(MAKE) -s -C $$m test >/dev/null 2>&1; then echo "$$m: ok"; \
		else echo "$$m: FAIL"; status=1; fi; \
	done; exit $$status

# 诊断构建：全部通过则绘制输出与默认构建逐字节一致；
# 某模块验收失败 -> 退出码 2，只丢该模块绘制（stdout 为空，证据在 stderr），其余照常
diag-test:
	@status=0; for m in $(DIAG_MODULES); do \
		$(MAKE) -s -C $$m diag >/dev/null || { echo "[diag] $$m: build failed"; status=1; continue; }; \
		./$$m/diag-bin; rc=$$?; \
		if [ $$rc -eq 0 ]; then echo "[diag] $$m: ok (drew)"; \
		elif [ $$rc -eq 2 ]; then echo "[diag] $$m: DROPPED (drawing skipped, evidence above)"; \
		else echo "[diag] $$m: hard failure rc=$$rc"; status=1; fi; \
	done; exit $$status

# 演示降级：给某个模块注入故障，其余模块仍正常绘制
# 用法: make diag-demo MODULE=tilemap   (MODULE 缺省 tilemap，all 表示全部)
diag-demo: MODULE ?= tilemap
diag-demo:
	@$(MAKE) -s diag-test MINI_GFX_FAULT=$(MODULE)

clean:
	@for m in $(MODULES); do $(MAKE) -s -C $$m clean; done

.PHONY: test diag-test diag-demo clean
