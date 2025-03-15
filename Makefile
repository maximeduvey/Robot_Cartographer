# Directories
# this allow to set the origin and make every include make abel to also add the include flags
ROBOT_CARTOGRAPHER_SOURCE_PATH = $(PWD)

# so that it cna be set elsewhere
include Makefile.rules

OBJ_DIR = OBJ
OBJ_DIR_WITH_TESTS = OBJ

TARGET_TESTS = carto_tests.exe

# Compiler
CXX = g++

# Compiler flags
INCLUDE_DIRS = \
    -I $(ROBOT_CARTOGRAPHER_DEBUG_DIR) \
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
    $(ROBOT_CARTOGRAPHER_DEBUG_DIR)/CreationTools.cpp \
    $(ROBOT_CARTOGRAPHER_DEBUG_DIR)/main.cpp

SRC_FILES_WITH_TESTS = \
    $(ROBOT_CORE_SRC_FILES) \
    $(ROBOT_CARTOGRAPHER_SRC_FILES) \
    $(ROBOT_CARTOGRAPHER_DEBUG_DIR)/CreationTools.cpp \
    $(ROBOT_CARTOGRAPHER_GTEST_SRC)

$(info SRC_FILES=$(SRC_FILES))
#RELATIVE_SRC_FILES := $(subst $(CURDIR)/,,$(SRC_FILES))
#$(info RELATIVE_SRC_FILES=$(RELATIVE_SRC_FILES))

# Rule to compile .cpp files to .o files inside OBJ folder
$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(INCLUDE_DIRS) -c $< -o $@

$(OBJ_DIR_WITH_TESTS)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(ROBOT_CARTOGRAPHER_GTEST_INCLUDE) $(INCLUDE_DIRS) -c $< -o $@

# Object files
OBJ_FILES = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(SRC_FILES))

OBJ_FILES_WITH_TEST = $(patsubst %.cpp, $(OBJ_DIR_WITH_TESTS)/%.o, $(SRC_FILES_WITH_TESTS))

# Output executable
TARGET = mapping.exe

# Default rule
all: $(TARGET)

# Rule to link the final executable
$(TARGET): $(OBJ_FILES)
	$(CXX) -o $@ $^ $(LDFLAGS)


tests: $(OBJ_FILES_WITH_TEST)
	$(CXX) -o $(TARGET_TESTS) $^ $(LDFLAGS) -lgtest -lgtest_main 

# Clean up
clean:
	rm -rf $(OBJ_DIR) $(TARGET)

clean_tests:
	rm -rf $(OBJ_DIR_WITH_TESTS) $(TARGET_TESTS)

# Rebuild rule
re: clean all

re_tests: clean_tests tests

# Phony targets
.PHONY: all clean re tests
