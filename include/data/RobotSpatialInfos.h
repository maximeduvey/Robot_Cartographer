
#pragma once


#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

#define PAMI_ROBOT_SIZE_LENGTH 4 // 90.0f mm
#define PAMI_ROBOT_SIZE_WIDTH  4 // 110.0f mm
#define PAMI_ROBOT_SIZE_HEIGHT 2 // 110.0 mm

struct RobotSpatialInfos
{
public:
  Eigen::Vector3f currentRobotPosition = {0.0f, 0.0f, 0.0f};  // only 2d for now, we'll check later if 3D has meaning
  Eigen::Vector3f robotStartingPosition = {0.0f, 0.0f, 0.0f}; // useful to return to it
  float _currentRobotAngle = {0.0f};                    // in which direction the robot is looking
  Eigen::Vector3f robotSize = {
      PAMI_ROBOT_SIZE_LENGTH,
      PAMI_ROBOT_SIZE_WIDTH,
      PAMI_ROBOT_SIZE_HEIGHT}; // is he long? is he fat? is he tall ? (lenght, width, height)

    std::pair<int, int> getRobotSizeInGridCells(float gridResolution) const;

    // Equality operator for comparison
    bool operator==(const RobotSpatialInfos& other) const;

    // Print operator for debugging
    friend std::ostream& operator<<(std::ostream& os, const RobotSpatialInfos& info);
};

