
#pragma once

#include "CommonSpaceRepresentation.h"

/// @brief The goal of the PositionController is to be the traduction and accessor of the robot infos (RobotSpatialInfos)
/// it has to manage both the traduction from motor raw data (and keep track of it) to "absolute" position traduction
class PositionController
{
private:
    /* data */
public:
    PositionController() = default;
    virtual ~PositionController()= default;;
    const RobotSpatialInfos getRobotSpatialInfos();

    // Thread-safe setter to replace the robot's spatial information
    void setRobotSpatialInfos(const RobotSpatialInfos &robotInfo);

    void updatePosition(float leftMotorCm, float rightMotorCm);

    // Thread-safe method to move the robot by a specified delta in x and y
    void translate(float deltaX, float deltaY); // only use it with vecorial value
    void translate(const Eigen::Vector2f &delta);

    // Thread-safe method to rotate the robot by a specified angle in degrees
    void rotate(float deltaAngle);
void updatePositionFromMotors(float leftMotorCm, float rightMotorCm);

private:
    RobotSpatialInfos _robot;
    std::mutex _mutexRobot;
};
