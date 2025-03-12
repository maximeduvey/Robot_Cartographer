#include "SingletonGameState.h"

SingletonGameState& SingletonGameState::getInstance()
{
    static SingletonGameState instance;
    return instance;
}

// Initialize the viewer
void SingletonGameState::initialize()
{

}

RobotSpatialInfos SingletonGameState::getRobotInfos()
{
    std::lock_guard<std::mutex> lock(_mutexRobotInfos);
    return _robotInfos;
}

void SingletonGameState::setRobotInfos(RobotSpatialInfos& robot)
{
    std::lock_guard<std::mutex> lock(_mutexRobotInfos);
    _robotInfos = robot;
}


Eigen::Vector3f SingletonGameState::getDestinationGoal()
{
    std::lock_guard<std::mutex> lock(_mutexDestinationGoal);
    return _destinationGoal;
}

void SingletonGameState::setDestinationGoal(Eigen::Vector3f& pos)
{
    std::lock_guard<std::mutex> lock(_mutexDestinationGoal);
    _destinationGoal = pos;
}


