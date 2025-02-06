
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
#include <ctime>

#include <CommonSpaceRepresentation.h>

#define PAMI_ROBOT_SIZE_LENGTH 4 // 90.0f mm
#define PAMI_ROBOT_SIZE_WIDTH 4  // 110.0f mm
#define PAMI_ROBOT_SIZE_HEIGHT 2 // 110.0 mm

class RobotSpatialInfos : public Object3D
{
public:
  RobotSpatialInfos();
  RobotSpatialInfos(
      const Eigen::Vector3f &robotStartingPosition,
      const Eigen::Vector3f &obsize = {PAMI_ROBOT_SIZE_LENGTH,PAMI_ROBOT_SIZE_WIDTH, PAMI_ROBOT_SIZE_HEIGHT});
  virtual ~RobotSpatialInfos();
  RobotSpatialInfos(const RobotSpatialInfos &other);

  Eigen::Vector3f _robotStartingPosition = {0.0f, 0.0f, 0.0f}; // useful to return to it
  float _currentRobotAngle = {0.0f};                           // in which direction the robot is looking

  // is now managed by the inheritance to Object3D
  //  Eigen::Vector3f currentRobotPosition = {0.0f, 0.0f, 0.0f};  // only 2d for now, we'll check later if 3D has meaning
  /*  Eigen::Vector3f robotSize = {
        PAMI_ROBOT_SIZE_LENGTH,
        PAMI_ROBOT_SIZE_WIDTH,
        PAMI_ROBOT_SIZE_HEIGHT}; // is he long? is he fat? is he tall ? (lenght, width, height)
  */
  std::pair<int, int> getRobotSizeInGridCells(float gridResolution);

  bool operator==(const RobotSpatialInfos &other);
  RobotSpatialInfos& operator=(const RobotSpatialInfos& other);

  // Print operator for debugging
  friend std::ostream &operator<<(std::ostream &os, const RobotSpatialInfos &info);

  Eigen::Vector3f getMovementFromLastMeasure();

private:
  // this timestamp allow use to calculate the movement since the last previous robot Positition
  // and track detected object as same object seen previously, allowing us to calibrate ourself
  time_t _previousRobotPositionTimestamp;
  // robot previous position from last measurement done, allow to calculate the robot movement between lidar measure
  Eigen::Vector3f previousRobotPosition = {0.0f, 0.0f, 0.0f};
  std::mutex _mutexDataAccessor;
};
