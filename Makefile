IDIR=include
CC=gcc
CCXX=g++
CFLAGS=-I$(IDIR) -g

SRCDIR=src
OUTDIR=bin
OBJDIR=$(OUTDIR)/obj
LDIR=lib

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
	LIBS=-framework OpenGL -lm -lglfw -lglew
else
	LIBS=-lm -lglfw -lGLEW -lGL -lpng -lz
endif

_DEPS = camera.h common.h core.h graphics_math.h graphics.h obj.h util.h
DEPS = $(patsubst %,$(SRCDIR)/%,$(_DEPS))

_OBJ = camera.o core.o graphics_math.o graphics.o main.o obj.o util.o
OBJ = $(patsubst %,$(OBJDIR)/%,$(_OBJ))

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

basic-engine: $(OBJ)
	$(shell mkdir -p $(@D))
	$(CCXX) -o $(OUTDIR)/$@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -r $(OUTDIR)
