# Makefile basics: https://www.cs.colby.edu/maxwell/courses/tutorials/maketutor/

SRC	:= src
BIN	:= bin
EXEC := $(BIN)/main

SRC_FILES := $(wildcard $(SRC)/*.cpp)
OBJ_FILES := $(patsubst $(SRC)/%.cpp, $(BIN)/%.o, $(SRC_FILES))

TEST_SRC := test
TEST_BIN := bin
TEST_EXEC := $(TEST_BIN)/test

CXX := g++ -std=c++17 -g -Wall -Wextra -pedantic -framework OpenGL -framework GLUT -I $(SRC)

all: $(EXEC)

# Compile an source file into an object file
$(BIN)/%.o: $(SRC)/%.cpp
	@$(CXX) -c -o $@ $^

# Create app executable by linking object files
$(EXEC): $(OBJ_FILES)
	@$(CXX) -o $@ $^

# Create test executable from test sources and app object files, except for $(EXEC).o
$(TEST_EXEC): $(TEST_SRC)/*.cpp $(filter-out $(EXEC).o, $(OBJ_FILES))
	@$(CXX) -I $(TEST_SRC) -o $@ $^

run: clean $(EXEC)
	@./$(EXEC)

test: clean $(TEST_EXEC)
	@./$(TEST_EXEC)

.PHONY: clean

clean:
	@rm -rf $(BIN)/*
	@rm -rf $(TEST_BIN)/*