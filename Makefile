COMPILER := g++
FLAGS := -std=c++17 -g -Wall -Wextra -pedantic -framework OpenGL -framework GLUT 

BIN	:= bin
SRC	:= src
EXEC := main

INCL := -I $(SRC)

all: $(BIN)/$(EXEC)

$(BIN)/$(EXEC): $(SRC)/*.cpp
	@$(COMPILER) $(FLAGS) $(INCL) $^ -o $@

run: clean all
	@./$(BIN)/$(EXEC)

clean:
	@rm -rf $(BIN)/*
