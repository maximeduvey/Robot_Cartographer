# Directories
SRC_DIR = src
INCLUDE_DIR = include
DATA_DIR = data
EXTERNAL_LIB = external_lib
TEST_DIR = test
OBJ_DIR = OBJ

# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = \
    -I $(INCLUDE_DIR)/$(DATA_DIR) \
    -I $(INCLUDE_DIR) \
    -I $(TEST_DIR) \
    -I $(EXTERNAL_LIB)/eigen \
    -I /usr/include/pcl-1.12 \
    -Wall -O2
#-std=c++17

# Libraries
LIBS =  -lpthread \
        -lncurses \
        -lm \
        -L/usr/lib \
        -lpcl_common -lpcl_io -lpcl_filters -lpcl_kdtree -lpcl_segmentation -lpcl_search 

# Source files
SRC_FILES = $(wildcard $(SRC_DIR)/Mapper.cpp) \
            $(wildcard $(SRC_DIR)/$(DATA_DIR)/*.cpp) \
            $(wildcard $(TEST_DIR)/*.cpp)

# Object files
OBJ_FILES = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(SRC_FILES))

# Output executable
TARGET = mapping.exe

# Default rule
all: $(TARGET)

# Rule to link the final executable
$(TARGET): $(OBJ_FILES)
	$(CXX) -o $@ $^ $(LIBS)

# Rule to compile .cpp files to .o files inside OBJ folder
$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(OBJ_DIR)/$(SRC_DIR)
	@mkdir -p $(OBJ_DIR)/$(SRC_DIR)/$(DATA_DIR)
	@mkdir -p $(OBJ_DIR)/$(TEST_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up
clean:
	rm -rf $(OBJ_DIR) $(TARGET)

# Rebuild rule
re: clean all

# Phony targets
.PHONY: all clean re
