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
    -I $(EXTERNAL_LIB)/pcl/common/include/ \
    -I $(EXTERNAL_LIB)/pcl/build/include/ \
    -I $(EXTERNAL_LIB)/pcl/filters/include/ \
    -I $(EXTERNAL_LIB)/pcl/search/include/ \
    -I $(EXTERNAL_LIB)/pcl/segmentation/include/ \
    -I $(EXTERNAL_LIB)/pcl/io/include/ \
    -I $(EXTERNAL_LIB)/pcl/kdtree/include/ \
    -I $(EXTERNAL_LIB)/pcl/visualization/include/ \
    -I $(EXTERNAL_LIB)/pcl/sample_consensus/include/ \
    -I /usr/include/vtk-7.1/ \
    -Wall -O0 -g
#-std=c++17

# Libraries
LDFLAGS =  -Wl,--no-as-needed \
        -L/usr/lib -L/usr/lib/x86_64-linux-gnu \
        -lvtkCommonCore-7.1 -lvtkCommonDataModel-7.1 -lvtkCommonMath-7.1 \
        -lvtkIOCore-7.1 -lvtkIOGeometry-7.1 \
        -lvtkRenderingCore-7.1 -lvtkRenderingOpenGL2-7.1 -lvtkRenderingFreeType-7.1 \
        -lvtkFiltersCore-7.1 -lvtkInteractionStyle-7.1 \
        -lpcl_common -lpcl_io -lpcl_filters -lpcl_kdtree -lpcl_segmentation -lpcl_search -lpcl_visualization \
        -lpthread \
        -lncurses \
        -lm



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
	$(CXX) -o $@ $^ $(LDFLAGS)

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
