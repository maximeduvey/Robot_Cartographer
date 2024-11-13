# Directories
SRC_DIR = src
INCLUDE_DIR = include
EXTERNAL_LIB=external_lib
TEST_DIR=test
# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = \
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
        -lpcl_common -lpcl_io -lpcl_filters -lpcl_kdtree -lpcl_segmentation


# Source files
SRC_FILES = $(wildcard $(SRC_DIR)/Mapper.cpp) \
            $(wildcard $(TEST_DIR)/*.cpp)

# Object files
OBJ_FILES = $(SRC_FILES:.cpp=.o)

# Output executable
TARGET = mapping.exe

# Default rule
all: $(TARGET)

# Rule to link the final executable
$(TARGET): $(OBJ_FILES)
	$(CXX) -o $@ $^ $(LIBS)

# Rule to compile .cpp files to .o files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up
clean:
	rm -f $(OBJ_FILES) $(TARGET)

# Rebuild rule
re: clean all

# Phony targets
.PHONY: all clean re
