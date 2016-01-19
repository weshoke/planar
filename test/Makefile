
VSR_FOLDER = ../../vsr2.0
EGGS_FOLDER = ../../variant
INCLUDE_FLAGS = -I../third-party/lest/include -I../src -I$(VSR_FOLDER)/include -I$(VSR_FOLDER)/include/vsr -I$(EGGS_FOLDER)/include
CC = g++
CFLAGS += -std=c++11 -Wall $(INCLUDE_FLAGS)
SRCDIR = ../src
CPP_FILES = $(wildcard $(SRCDIR)/*.cpp)
OBJ_FILES = $(notdir $(CPP_FILES:.cpp=.o))

all: tests

tests: run_tests.o $(OBJ_FILES)
	$(CC) run_tests.o -o run_tests

%.o: run_tests.cpp
	$(CC) $(CFLAGS) run_tests.cpp

clean:
	rm *.o run_tests
