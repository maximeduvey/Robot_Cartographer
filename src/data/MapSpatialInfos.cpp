#include <iostream>

#include <MapSpatialInfos.h>

MapSpatialInfos::MapSpatialInfos() : _grid_center_x(grid_lenght / 2), _grid_center_y(grid_width / 2) {}

// Overloaded Constructor
MapSpatialInfos::MapSpatialInfos(float length, float width, float resolution)
    : grid_lenght(length), grid_width(width), gridResolution(resolution)
{
    updateGridCenter();
}

// Getters for grid_center_x and grid_center_y
int MapSpatialInfos::get_grid_center_x() const { return _grid_center_x; }
int MapSpatialInfos::get_grid_center_y() const { return _grid_center_y; }

// Setters for grid_lenght and grid_width (with auto-update)
void MapSpatialInfos::setGridLenght(float length)
{
    grid_lenght = length;
    updateGridCenter();
}

void MapSpatialInfos::setGridWidth(float width)
{
    grid_width = width;
    updateGridCenter();
}

// Assignment Operator Overload
MapSpatialInfos &MapSpatialInfos::operator=(const MapSpatialInfos &other)
{
    if (this != &other)
    {
        grid_lenght = other.grid_lenght;
        grid_width = other.grid_width;
        gridResolution = other.gridResolution;
        updateGridCenter();
    }
    return *this;
}

// Utility function to print map information
void MapSpatialInfos::printInfo() const
{
    std::cout << "Grid Length: " << grid_lenght
              << ", Grid Width: " << grid_width
              << ", Grid Resolution: " << gridResolution
              << ", Grid Center X: " << get_grid_center_x()
              << ", Grid Center Y: " << get_grid_center_y() << std::endl;
}

// Update grid center based on current length and width
void MapSpatialInfos::updateGridCenter()
{
    _grid_center_x = static_cast<int>(grid_lenght / 2);
    _grid_center_y = static_cast<int>(grid_width / 2);
}
