IDIR=include
CC=gcc
CCXX=g++
CFLAGS=-I$(IDIR) -g

SRCDIR=src
OUTDIR=bin
OBJDIR=$(OUTDIR)/obj
VENDORDIR=$(SRCDIR)/vendor
LDIR=lib

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
	LIBS=-framework OpenGL -lm -lglfw -lglew
else
	LIBS=-lm -lglfw -lGLEW -lGL -lpng -lz
endif

_DEPS = camera.h collision.h common.h core.h gjk.h gm.h graphics.h menu.h obj.h pbd.h pmc.h physics.h quaternion.h util.h
DEPS = $(patsubst %,$(SRCDIR)/%,$(_DEPS))

_OBJ = camera.o collision.o core.o gjk.o graphics.o main.o menu.o obj.o pbd.o pmc.o physics.o quaternion.o util.o
OBJ = $(patsubst %,$(OBJDIR)/%,$(_OBJ))

_VENDOR = imgui.o imgui_demo.o imgui_draw.o imgui_impl_glfw.o imgui_impl_opengl3.o imgui_widgets.o
VENDOR = $(patsubst %,$(OBJDIR)/%,$(_VENDOR))

all: basic-engine

$(OBJDIR)/%.o: $(VENDORDIR)/%.cpp $(DEPS)
	$(shell mkdir -p $(@D))
	$(CCXX) -c -o $@ $< $(CFLAGS)

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp $(DEPS)
	$(shell mkdir -p $(@D))
	$(CCXX) -c -o $@ $< $(CFLAGS)

$(OBJDIR)/%.o: $(SRCDIR)/%.c $(DEPS)
	$(shell mkdir -p $(@D))
	$(CC) -c -o $@ $< $(CFLAGS)

basic-engine: $(OBJ) $(VENDOR)
	$(shell mkdir -p $(@D))
	$(CCXX) -o $(OUTDIR)/$@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -r $(OUTDIR)
