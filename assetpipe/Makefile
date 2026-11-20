PYTHON ?= python3

.PHONY: all test clean

all:
	@echo "assetpipe 是纯 Python 模块，直接 make test 运行自检"

test:
	$(PYTHON) -I tests/selftest.py

clean:
	rm -rf __pycache__ src/__pycache__ tests/__pycache__
