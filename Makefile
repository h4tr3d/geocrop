CC ?= gcc
CXX ?= g++

GDAL_INCLUDE ?= `gdal-config --cflags`
GDAL_LIB     ?= `gdal-config --libs`

LIBS += $(LDFLAGS) $(GDAL_LIB) -lproj
CXXFLAGS += -ggdb3 -O0 $(GDAL_INCLUDE)

OBJ = main.o
EXE = geocrop

all: $(OBJ)
		$(CXX) -o $(EXE) $(OBJ) $(LIBS)

