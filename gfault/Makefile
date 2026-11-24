CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
ROOT := $(abspath ..)
INC := -I$(ROOT)/gfault/include -I$(ROOT)/gfault/render -I$(ROOT)/palette/include

# Sources of the five segments in the current (hardened) tree.
PARTICLE_SRC := $(ROOT)/particle/src/particle.c
TILEMAP_SRC  := $(ROOT)/tilemap/src/tilemap.c
NAVGRID_SRC  := $(ROOT)/navgrid/src/navgrid.c
FONT_SRC     := $(ROOT)/fontatlas/src/fontatlas.c
PALETTE_SRC  := $(ROOT)/palette/src/palette.c

GFAULT_CORE  := $(ROOT)/gfault/src/gfault.c $(ROOT)/gfault/src/gfault_inject.c
FRAMEBUF_SRC := $(ROOT)/gfault/render/framebuf.c

# Override on the command line for the golden build:
#   make GOLDEN_TREE=/path/to/base/worktree compositor_golden
GOLDEN_TREE ?= $(ROOT)
G_PARTICLE := $(GOLDEN_TREE)/particle/src/particle.c
G_TILEMAP  := $(GOLDEN_TREE)/tilemap/src/tilemap.c
G_NAVGRID  := $(GOLDEN_TREE)/navgrid/src/navgrid.c
G_FONT     := $(GOLDEN_TREE)/fontatlas/src/fontatlas.c

BRIDGES := test/bridge_particle.c test/bridge_tilemap.c test/bridge_navgrid.c \
           test/bridge_font.c test/bridge_palette.c
HARNESS := test/compositor.c

.PHONY: all test clean

all: compositor compositor_golden alloc_test natural_fault_test

# --- current build: hardened module sources ---
compositor: $(HARNESS) $(BRIDGES) $(GFAULT_CORE) $(FRAMEBUF_SRC) $(PALETTE_SRC)
	$(CC) $(CFLAGS) $(INC) \
	  -DMODULE_SRC='"$(PARTICLE_SRC)"' -c test/bridge_particle.c -o bridge_particle.o
	$(CC) $(CFLAGS) $(INC) \
	  -DMODULE_SRC='"$(TILEMAP_SRC)"'  -c test/bridge_tilemap.c  -o bridge_tilemap.o
	$(CC) $(CFLAGS) $(INC) \
	  -DMODULE_SRC='"$(NAVGRID_SRC)"'  -c test/bridge_navgrid.c  -o bridge_navgrid.o
	$(CC) $(CFLAGS) $(INC) \
	  -DMODULE_SRC='"$(FONT_SRC)"'     -c test/bridge_font.c     -o bridge_font.o
	$(CC) $(CFLAGS) $(INC) -c test/bridge_palette.c -o bridge_palette.o
	$(CC) $(CFLAGS) $(INC) -c test/compositor.c -o compositor.o
	$(CC) $(CFLAGS) $(INC) compositor.o bridge_particle.o bridge_tilemap.o \
	  bridge_navgrid.o bridge_font.o bridge_palette.o \
	  $(GFAULT_CORE) $(FRAMEBUF_SRC) $(PALETTE_SRC) -o $@ -lm

# --- golden build: pre-change module sources from GOLDEN_TREE ---
compositor_golden: $(HARNESS) $(BRIDGES) $(GFAULT_CORE) $(FRAMEBUF_SRC) $(PALETTE_SRC)
	$(CC) $(CFLAGS) $(INC) \
	  -DMODULE_SRC='"$(G_PARTICLE)"' -c test/bridge_particle.c -o g_bridge_particle.o
	$(CC) $(CFLAGS) $(INC) \
	  -DMODULE_SRC='"$(G_TILEMAP)"'  -c test/bridge_tilemap.c  -o g_bridge_tilemap.o
	$(CC) $(CFLAGS) $(INC) \
	  -DMODULE_SRC='"$(G_NAVGRID)"'  -c test/bridge_navgrid.c  -o g_bridge_navgrid.o
	$(CC) $(CFLAGS) $(INC) \
	  -DMODULE_SRC='"$(G_FONT)"'     -c test/bridge_font.c     -o g_bridge_font.o
	$(CC) $(CFLAGS) $(INC) -c test/bridge_palette.c -o g_bridge_palette.o
	$(CC) $(CFLAGS) $(INC) -c test/compositor.c -o g_compositor.o
	$(CC) $(CFLAGS) $(INC) g_compositor.o g_bridge_particle.o g_bridge_tilemap.o \
	  g_bridge_navgrid.o g_bridge_font.o g_bridge_palette.o \
	  $(GFAULT_CORE) $(FRAMEBUF_SRC) $(PALETTE_SRC) -o $@ -lm

# --- allocation counter test (current sources only) ---
alloc_test: test/alloc_test.c test/alloc_counter.c $(BRIDGES) $(GFAULT_CORE) \
            $(FRAMEBUF_SRC) $(PALETTE_SRC)
	$(CC) $(CFLAGS) $(INC) \
	  -DMODULE_SRC='"$(PARTICLE_SRC)"' -c test/bridge_particle.c -o a_bridge_particle.o
	$(CC) $(CFLAGS) $(INC) \
	  -DMODULE_SRC='"$(TILEMAP_SRC)"'  -c test/bridge_tilemap.c  -o a_bridge_tilemap.o
	$(CC) $(CFLAGS) $(INC) \
	  -DMODULE_SRC='"$(NAVGRID_SRC)"'  -c test/bridge_navgrid.c  -o a_bridge_navgrid.o
	$(CC) $(CFLAGS) $(INC) \
	  -DMODULE_SRC='"$(FONT_SRC)"'     -c test/bridge_font.c     -o a_bridge_font.o
	$(CC) $(CFLAGS) $(INC) -c test/bridge_palette.c -o a_bridge_palette.o
	$(CC) $(CFLAGS) $(INC) -c test/alloc_test.c -o alloc_test.o
	$(CC) $(CFLAGS) $(INC) -c test/alloc_counter.c -o alloc_counter.o
	$(CC) $(CFLAGS) $(INC) alloc_test.o alloc_counter.o a_bridge_particle.o \
	  a_bridge_tilemap.o a_bridge_navgrid.o a_bridge_font.o a_bridge_palette.o \
	  $(GFAULT_CORE) $(FRAMEBUF_SRC) $(PALETTE_SRC) -o $@ -lm -ldl

# --- natural failure-path encoding test (current hardened sources) ---
natural_fault_test: test/natural_fault_test.c $(GFAULT_CORE) $(FRAMEBUF_SRC) $(PALETTE_SRC)
	$(CC) $(CFLAGS) $(INC) \
	  -DMODULE_PARTICLE='"$(PARTICLE_SRC)"' \
	  -DMODULE_TILEMAP='"$(TILEMAP_SRC)"' \
	  -DMODULE_NAVGRID='"$(NAVGRID_SRC)"' \
	  -DMODULE_FONT='"$(FONT_SRC)"' \
	  test/natural_fault_test.c $(GFAULT_CORE) $(FRAMEBUF_SRC) $(PALETTE_SRC) \
	  -o $@ -lm

test: all
	./compositor
	./alloc_test
	./natural_fault_test

clean:
	rm -f *.o compositor compositor_golden alloc_test natural_fault_test
