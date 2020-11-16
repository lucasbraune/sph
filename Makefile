COMPILER := g++
FLAGS := -std=c++17 -g -Wall -Wextra -pedantic -framework OpenGL -framework GLUT 

BIN	:= bin
SRC	:= src
UTIL := src/util
EXEC := main

INCL := -I $(SRC) -I $(UTIL)

all: $(BIN)/$(EXEC)

$(BIN)/$(EXEC): $(SRC)/*.cpp
	@$(COMPILER) $(FLAGS) $(INCL) $^ -o $@

run: clean all
	@./$(BIN)/$(EXEC)

clean:
	@rm -rf $(BIN)/*
