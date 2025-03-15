#pragma once

#include <mutex>
#include <RobotSpatialInfos.h>
#include <Eigen/Dense>

/// this singleton is a "fake" it is mean to be compiled and used in a stand-alone of the MappingAndPthfindingLib
/// but should not be compiled with complete project as the real one should be used
class SingletonGameState {
public:
    // Get the singleton instance
    static SingletonGameState& getInstance();

    // Initialize the viewer
    void initialize();

    RobotSpatialInfos getRobotInfos();
    void setRobotInfos(RobotSpatialInfos& robot);
    std::mutex _mutexRobotInfos;
    RobotSpatialInfos _robotInfos;



    Eigen::Vector3f getDestinationGoal();
    void setDestinationGoal(Eigen::Vector3f& robot);

    std::mutex _mutexDestinationGoal;
    Eigen::Vector3f _destinationGoal = { 0.0f , 0.0f, 0.0f };
};
