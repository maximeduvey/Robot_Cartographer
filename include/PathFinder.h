#pragma once

/// @brief This class as it's name suggest centralized different way/algorithme to find a path for a specified object

#include "common_includes.h"
#include "CommonSpaceRepresentation.h"

class PathFinder
{
public:
    PathFinder(/* args */);
    virtual ~PathFinder();

private:
    bool isgGidCellOccupied(std::vector<std::vector<int>> grid, int grid_x, int grid_y);
};
