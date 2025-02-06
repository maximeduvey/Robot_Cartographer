#pragma once

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <mutex>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

#define TAG (std::string("[") + std::string(__PRETTY_FUNCTION__) + std::string("] ")).c_str()

/// represent Distance between wheels in cm
/// TODO: !!! check and measure
/// debug info: wheel external distance right ot left 106mm, internal wheel dist 86mm, grip gauch 11mm, grip droit 10.5mm
#define TRACK_WIDTH_CM 10.5f
// 20

// in short: each measurement is "cone of view" on an angle start/end (ex: 234.87 / 243.59)
// that contain 12 measurement, see doc: Communication_Protocol.pdf page 5
#define LD06_TWELVE_STEP_ANGLE 12

#define POS_X 0
#define POS_Y 1

class Object3D
{
public:
  Eigen::Vector3f center; // Center of the object (x, y, z)
  Eigen::Vector3f size;   // Size of the object (length, width, height)
  int id;

  Object3D() : center({ 0,0,0 }), size(0, 0, 0), id(0) {}
  Object3D(const Eigen::Vector3f& pos) : center(pos), size({ 0,0,0 }), id(0) {}
  Object3D(const Eigen::Vector3f& pos, const Eigen::Vector3f& s) : center(pos), size(s), id(0) {}
  Object3D(const Object3D& copy) : center(copy.center), size(copy.size), id(copy.id) {}
  Object3D& operator=(const Object3D& other) {
    if (this != &other) {
      center = other.center;
      size = other.size;
      id = other.id;
    }
    return *this;
  }
  friend std::ostream& operator<<(std::ostream& os, const Object3D& info) {
    os << "id:" << info.id << ", center x:" << info.center.x() << ", y:" << info.center.y() << ", z:" << info.center.z()
      << ", size x:" << info.size.x() << ", y:" << info.size.y() << ", z:" << info.size.z();
    return os;
  }
};

/// @brief this class represent a point in space, it has a distance var when this point is relative to an object
/// for example the lidar detect a point "far away" set it's pos and distance relative to the robot/lidar
struct Point
{
  Eigen::Vector3f pos{ 0.0f, 0.0f, 0.0f };
  uint16_t distance = 0;
};

/// @brief half generic, the logic of list of point with the nbr of point is here
/// but for simplicity reason it has a default value (for now with have only lidarLD06 so let simplify to it)
class FieldPoints
{
public:
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