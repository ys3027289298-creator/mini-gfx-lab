.PHONY: all test acceptance clean

DIRS := particle tilemap navgrid spatial chain

all:
	@for d in $(DIRS); do $(MAKE) -s -C $$d || exit 1; done

test:
	@for d in $(DIRS); do $(MAKE) -s -C $$d test || exit 1; done

acceptance:
	@python3 tools/acceptance.py

clean:
	@for d in $(DIRS); do $(MAKE) -s -C $$d clean; done
