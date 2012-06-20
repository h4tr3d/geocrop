CC = gcc
CXX = g++
LIBS = -lgdal -lproj
CXXFLAGS += -ggdb3 -O0

OBJ = main.o
EXE = geocrop

all: $(OBJ)
		$(CXX) -o $(EXE) $(OBJ) $(LIBS)

