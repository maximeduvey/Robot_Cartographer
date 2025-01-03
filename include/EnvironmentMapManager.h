#pragma once

#include <Lidar_LD06.h>

/// @brief The goal of this class it to manage the perception of the surrounding of the PAMI
/// (So to manage the lidar) and feed the mapper with the data, and use the pathfinder
/// to provide useful informations, suh as :
/// providing a path to a destination with possible obstacle on road.
/// providing obstacle alert 
class EnvironmentMapManager
{
private:
    Lidar_LD06 _lidar;
public:
    EnvironmentMapManager(/* args */);
    ~EnvironmentMapManager();

    void start();
    
};

