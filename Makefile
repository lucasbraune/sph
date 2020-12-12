# Makefile basics: https://www.cs.colby.edu/maxwell/courses/tutorials/maketutor/

APP_SRC:=src/app
APP_BIN:=bin/app
APP_EXEC:=$(APP_BIN)/main

LIB_SRC:=src/lib
LIB_BIN:=bin/lib
LIB_OBJS:=$(patsubst $(LIB_SRC)/%.cpp, $(LIB_BIN)/%.o, $(wildcard $(LIB_SRC)/*.cpp))

TEST_SRC:=src/test
TEST_BIN:=bin/test
TEST_EXEC:=$(TEST_BIN)/main

CXX:=g++ -std=c++17 -g -Wall -Wextra -pedantic -I $(LIB_SRC)
GLUT_FLAGS:=-framework OpenGL -framework GLUT

# Compile app executable
$(APP_EXEC): $(APP_SRC)/*.cpp $(LIB_OBJS)
	@$(CXX) $(GLUT_FLAGS) -I $(APP_SRC) -o $@ $^

# Create test executable
$(TEST_EXEC): $(TEST_SRC)/*.cpp $(LIB_OBJS)
	@$(CXX) -I $(TEST_SRC) -o $@ $^

# Seperately compile a library source file
$(LIB_BIN)/%.o: $(LIB_SRC)/%.cpp
	@$(CXX) -c -o $@ $^

run: clean $(APP_EXEC)
	@./$(APP_EXEC)

test: clean $(TEST_EXEC)
	@./$(TEST_EXEC)

.PHONY: clean 

clean:
	@rm -rf $(APP_BIN)/*
	@rm -rf $(LIB_BIN)/*
	@rm -rf $(TEST_BIN)/*