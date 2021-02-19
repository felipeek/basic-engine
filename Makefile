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

_DEPS = camera.h common.h core.h gm.h graphics.h hierarchical_model.h inverse_kinematics.h matrix.h menu.h obj.h quaternion.h util.h
DEPS = $(patsubst %,$(SRCDIR)/%,$(_DEPS))

_OBJ = camera.o core.o graphics.o hierarchical_model.o inverse_kinematics.o main.o matrix.o menu.o obj.o quaternion.o util.o
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
