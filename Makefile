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
	LIBS=-lm -lGLEW -lGL -lpng -lz -lglfw -ldl
endif

_DEPS = camera/camera.h camera/common.h camera/free.h camera/lookat.h common.h core.h gm.h graphics.h menu.h obj.h quaternion.h util.h
DEPS = $(patsubst %,$(SRCDIR)/%,$(_DEPS))

_OBJ = camera/camera.o camera/common.o camera/free.o camera/lookat.o core.o graphics.o main.o menu.o obj.o quaternion.o util.o
OBJ = $(patsubst %,$(OBJDIR)/%,$(_OBJ))

_VENDOR = imgui.o imgui_demo.o imgui_draw.o imgui_impl_glfw.o imgui_impl_opengl3.o imgui_tables.o imgui_widgets.o
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
