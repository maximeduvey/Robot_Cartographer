
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
    os << "Current Position: (" << info.currentRobotPosition.x() << ", " << info.currentRobotPosition.y() << ")"
       << "\nStarting Position: (" << info.robotStartingPosition.x() << ", " << info.robotStartingPosition.y() << ")"
       << "\nAngle: " << info._currentRobotAngle
       << "\nSize: (Length: " << info.robotSize.x()
       << ", Width: " << info.robotSize.y()
       << ", Height: " << info.robotSize.z() << ")";
    return os;
}