
#include "RobotSpatialInfos.h"

std::pair<int, int> RobotSpatialInfos::getRobotSizeInGridCells(float gridResolution) const
{
    int robotLengthCells = static_cast<int>(std::ceil(robotSize.x() / gridResolution));
    int robotWidthCells = static_cast<int>(std::ceil(robotSize.y() / gridResolution));
    return {robotLengthCells, robotWidthCells};
}

// Equality operator for comparison
bool RobotSpatialInfos::operator==(const RobotSpatialInfos &other) const
{
    return currentRobotPosition == other.currentRobotPosition &&
           robotStartingPosition == other.robotStartingPosition &&
           _currentRobotAngle == other._currentRobotAngle &&
           robotSize == other.robotSize;
}

// Print operator for debugging
std::ostream &operator<<(std::ostream &os, const RobotSpatialInfos &info)
{
    os << "Current Position: (x:" << info.currentRobotPosition.x() << ", y:" << info.currentRobotPosition.y() << ", z:" << info.currentRobotPosition.z() <<")"
       << "\nStarting Position: (x:" << info.robotStartingPosition.x() << ", y:" << info.robotStartingPosition.y() <<  ", z:" << info.robotStartingPosition.z() << ")"
       << "\nAngle: " << info._currentRobotAngle
       << "\nSize: (Length: " << info.robotSize.x()    << ", Width: " << info.robotSize.y() << ", Height: " << info.robotSize.z() << ")";
    return os;
}