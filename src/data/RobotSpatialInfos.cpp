
#include "RobotSpatialInfos.h"

RobotSpatialInfos::RobotSpatialInfos() : Object3D({0.0f, 0.0f, 0.0f}, {PAMI_ROBOT_SIZE_LENGTH, PAMI_ROBOT_SIZE_WIDTH, PAMI_ROBOT_SIZE_HEIGHT})
{
    _previousRobotPositionTimestamp = std::time(nullptr);
    std::cout << "Default Constructor: " << *this <<std::endl;
}

RobotSpatialInfos::RobotSpatialInfos(
      const Eigen::Vector3f &robotStartingPosition,
      const Eigen::Vector3f &obsize /* = {PAMI_ROBOT_SIZE_LENGTH, PAMI_ROBOT_SIZE_WIDTH, PAMI_ROBOT_SIZE_HEIGHT} */
      ) : Object3D(robotStartingPosition, size),
       _robotStartingPosition(robotStartingPosition), previousRobotPosition(robotStartingPosition)
{
    _previousRobotPositionTimestamp = std::time(nullptr);
    std::cout << "Constructor: " << *this <<std::endl;
}

RobotSpatialInfos::~RobotSpatialInfos()
{
}

std::pair<int, int> RobotSpatialInfos::getRobotSizeInGridCells(float gridResolution)
{
    std::lock_guard<std::mutex> lock(_mutexDataAccessor);
    int robotLengthCells = static_cast<int>(std::ceil(size.x() / gridResolution));
    int robotWidthCells = static_cast<int>(std::ceil(size.y() / gridResolution));
    return {robotLengthCells, robotWidthCells};
}

// Equality operator for comparison
bool RobotSpatialInfos::operator==(const RobotSpatialInfos &other)
{
    std::lock_guard<std::mutex> lock(_mutexDataAccessor);
    return center == other.center &&
           _robotStartingPosition == other._robotStartingPosition &&
           _currentRobotAngle == other._currentRobotAngle &&
           size == other.size;
}

// Custom copy constructor
RobotSpatialInfos::RobotSpatialInfos(const RobotSpatialInfos &other)
    : Object3D(other),
      previousRobotPosition(other.previousRobotPosition),
      _previousRobotPositionTimestamp(other._previousRobotPositionTimestamp)
{
    std::cout << "copy constructor: " << *this << ", other:" << other<<std::endl;
}

// Print operator for debugging
std::ostream &operator<<(std::ostream &os, const RobotSpatialInfos &info)
{
   // std::lock_guard<std::mutex> lock(_mutexDataAccessor);
    os << static_cast<Object3D>(info)
       << ", Starting Position: (x:" << info._robotStartingPosition.x() << ", y:" << info._robotStartingPosition.y() << ", z:" << info._robotStartingPosition.z() << ")"
       << ", Angle: " << info._currentRobotAngle;
    return os;
}

/// @brief Return the movement from last position (last position is the position last time this function has been called)
/// I keep track of the time in case one day i need to calculate the speed
/// @return
Eigen::Vector3f RobotSpatialInfos::getMovementFromLastMeasure()
{
    std::lock_guard<std::mutex> lock(_mutexDataAccessor);
    Eigen::Vector3f movement = center - previousRobotPosition;
    _previousRobotPositionTimestamp = std::time(nullptr);
    return movement;
}