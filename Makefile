# Directories

include Makefile.rules

TEST_DIR = test
OBJ_DIR = OBJ

# Compiler
CXX = g++

# Compiler flags
INCLUDE_DIRS = \
    -I $(TEST_DIR) \
    $(MAPPING_AND_PATHFINDING_INCLUDE_FLAGS) \
    -Wall -O0 -g
#-std=c++17

# Libraries
LDFLAGS =  -Wl,--no-as-needed \
        $(MAPPING_AND_PATHFINDING_COMPILE_FLAGS) \
        -lpthread \
        -lncurses \
        -lm

# Source files
SRC_FILES = $(MAPPING_AND_PATHFINDING_SRC_FILES) \
            $(wildcard $(TEST_DIR)/*.cpp)

# Object files
OBJ_FILES = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(SRC_FILES))

# Output executable
TARGET = mapping.exe

# Default rule
all: $(TARGET)

# Rule to link the final executable
$(TARGET): $(OBJ_FILES)
	$(CXX) -o $@ $^ $(LDFLAGS)

# Rule to compile .cpp files to .o files inside OBJ folder
$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(OBJ_DIR)/$(MAPPING_AND_PATHFINDING_SRC_DIR)
	@mkdir -p $(OBJ_DIR)/$(MAPPING_AND_PATHFINDING_SRC_DIR)/$(MAPPING_AND_PATHFINDING_DATA_DIR)
	@mkdir -p $(OBJ_DIR)/$(TEST_DIR)
	$(CXX) $(INCLUDE_DIRS) -c $< -o $@

# Clean up
clean:
	rm -rf $(OBJ_DIR) $(TARGET)

# Rebuild rule
re: clean all

# Phony targets
.PHONY: all clean re
