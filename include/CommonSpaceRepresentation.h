#pragma once

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <mutex>
#include <vector>

#include <Eigen/Dense>
#include "MapSpatialInfos.h"

#define TAG (std::string("[") + std::string(__PRETTY_FUNCTION__) + std::string("] ")).c_str()

/// represent Distance between wheels in cm
/// TODO: !!! check and measure
/// debug info: wheel external distance right ot left 106mm, internal wheel dist 86mm, grip gauch 11mm, grip droit 10.5mm
#define TRACK_WIDTH_CM 10.5f
// 20

// in short: each measurement is "cone of view" on an angle start/end (ex: 234.87 / 243.59)
// that contain 12 measurement, see doc: Communication_Protocol.pdf page 5
#define LD06_TWELVE_STEP_ANGLE 12

#define PAMI_ROBOT_SIZE_LENGTH 90.0f
#define PAMI_ROBOT_SIZE_WIDTH 110.0f
#define PAMI_ROBOT_SIZE_HEIGHT 110.0f


#define POS_X 0
#define POS_Y 1

struct RobotSpatialInfos
{
public:
  Eigen::Vector2f currentRobotPosition = {0.0f, 0.0f};  // only 2d for now, we'll check later if 3D has meaning
  Eigen::Vector2f robotStartingPosition = {0.0f, 0.0f}; // useful to return to it
  float _currentRobotAngle = {0.0f};                    // in which direction the robot is looking
  Eigen::Vector3f robotSize = {
      PAMI_ROBOT_SIZE_LENGTH, PAMI_ROBOT_SIZE_WIDTH, PAMI_ROBOT_SIZE_HEIGHT}; // is he long? is he fat? is he tall ? (lenght, width, height)
};

/// @brief this class represent a point in space, it has a distance var when this point is relative to an object
/// for example the lidar detect a point "far away" set it's pos and distance relative to the robot/lidar
struct Point
{
  Eigen::Vector2f pos{0.0f, 0.0f};
  uint16_t distance = 0;
};

/// @brief half generic, the logic of list of point with the nbr of point is here
/// but for simplicity reason it has a default value (for now with have only lidarLD06 so let simplify to it)
class FieldPoints
{
  public :
  // lidarCycle allow us to measure if the data are perceiving is an additionnal one 
  // or an update that should erase the previous one  
  size_t lidarCycle = 0;
  std::vector<Point> points;
};

struct PolPoint
{
  float rho;
  float theta;
};