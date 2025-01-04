
#pragma once

#include <iostream>

#define MAPSPATIALINFOS_WIDTH 30000  // 3 Metre
#define MAPSPATIALINFOS_LENGHT 20000 // 2 Metre
#define MAPSPATIALINFOS_DEFAULT_GRID_RESOLUTION 1.0f 

/// @brief This class is all the infos related to a map
/// BECAREFUL: it can be a map the robot is moving on or a map used to represent what the robot is seeing
class MapSpatialInfos {
public:
    // Variables
    float grid_lenght = MAPSPATIALINFOS_LENGHT;
    float grid_width = MAPSPATIALINFOS_WIDTH;
    float gridResolution = MAPSPATIALINFOS_DEFAULT_GRID_RESOLUTION; // in cm

private:
    int _grid_center_x = grid_lenght / 2;
    int _grid_center_y = grid_width / 2;

public:
    // Constructor
    MapSpatialInfos() ;

    // Overloaded Constructor
    MapSpatialInfos(float length, float width, float resolution);

    // Getters for grid_center_x and grid_center_y
    int get_grid_center_x() const;
    int get_grid_center_y() const;

    // Setters for grid_lenght and grid_width (with auto-update)
    void setGridLenght(float length);
    void setGridWidth(float width);

    // Assignment Operator Overload
    MapSpatialInfos& operator=(const MapSpatialInfos& other);

    // Utility function to print map information
    void printInfo() const;

private:
    // Update grid center based on current length and width
    void updateGridCenter();
};
