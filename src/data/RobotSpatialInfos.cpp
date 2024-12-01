
#include "RobotSpatialInfos.h"

RobotSpatialInfos::RobotSpatialInfos()
{
    _previousRobotPositionTimestamp = std::time(nullptr);
}

RobotSpatialInfos::~RobotSpatialInfos()
{
}

std::pair<int, int> RobotSpatialInfos::getRobotSizeInGridCells(float gridResolution) const
{
    int robotLengthCells = static_cast<int>(std::ceil(size.x() / gridResolution));
    int robotWidthCells = static_cast<int>(std::ceil(size.y() / gridResolution));
    return {robotLengthCells, robotWidthCells};
}

// Equality operator for comparison
bool RobotSpatialInfos::operator==(const RobotSpatialInfos &other) const
{
    return center == other.center &&
           robotStartingPosition == other.robotStartingPosition &&
           _currentRobotAngle == other._currentRobotAngle &&
           size == other.size;
}

// Print operator for debugging
std::ostream &operator<<(std::ostream &os, const RobotSpatialInfos &info)
{
    os << "Current Position: (x:" << info.center.x() << ", y:" << info.center.y() << ", z:" << info.center.z() << ")"
       << "\nStarting Position: (x:" << info.robotStartingPosition.x() << ", y:" << info.robotStartingPosition.y() << ", z:" << info.robotStartingPosition.z() << ")"
       << "\nAngle: " << info._currentRobotAngle
       << "\nSize: (Length: " << info.size.x() << ", Width: " << info.size.y() << ", Height: " << info.size.z() << ")";
    return os;
}

/// @brief Return the movement from last position (last position is the position last time this function has been called)
/// I keep track of the time in case one day i need to calculate the speed
/// @return
Eigen::Vector3f RobotSpatialInfos::getMovementFromLastMeasure()
{
    Eigen::Vector3f movement = center - previousRobotPosition;
    _previousRobotPositionTimestamp = std::time(nullptr);
    return movement;
}