# Directories
# this allow to set the origin and make every include make abel to also add the include flags
ROBOT_CARTOGRAPHER_SOURCE_PATH = $(PWD)

# so that it cna be set elsewhere
include Makefile.rules

TEST_DIR = test
OBJ_DIR = OBJ

# Compiler
CXX = g++

# Compiler flags
INCLUDE_DIRS = \
    -I $(TEST_DIR) \
    $(ROBOT_CORE_INCLUDE_FLAGS) \
    $(ROBOT_CARTOGRAPHER_INCLUDE_FLAGS) \
    -Wall -O0 -g
#-std=c++17

# Libraries
LDFLAGS = \
    $(ROBOT_CORE_COMPILE_FLAGS) \
    $(ROBOT_CARTOGRAPHER_COMPILE_FLAGS) \
    -Wl,--no-as-needed -lpthread -lncurses -lm

# Source files
SRC_FILES = \
    $(ROBOT_CORE_SRC_FILES) \
    $(ROBOT_CARTOGRAPHER_SRC_FILES) \
    $(TEST_DIR)/CreationTools.cpp \
    $(TEST_DIR)/main.cpp


$(info SRC_FILES=$(SRC_FILES))
#RELATIVE_SRC_FILES := $(subst $(CURDIR)/,,$(SRC_FILES))
#$(info RELATIVE_SRC_FILES=$(RELATIVE_SRC_FILES))

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
	@mkdir -p $(dir $@)
	$(CXX) $(INCLUDE_DIRS) -c $< -o $@

# Clean up
clean:
	rm -rf $(OBJ_DIR) $(TARGET)

# Rebuild rule
re: clean all

# Phony targets
.PHONY: all clean re
