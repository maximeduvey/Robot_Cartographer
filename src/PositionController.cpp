
#include "PositionController.h"
#include "CommonSpaceRepresentation.h"

#include <cmath>
#include <Eigen/Dense>
#include <iostream>

const RobotSpatialInfos PositionController::getRobotSpatialInfos()
{
    std::lock_guard<std::mutex> lock(_mutexRobot);
    return _robot;
}

// Use carefully, normally only done when starting
void PositionController::setRobotSpatialInfos(const RobotSpatialInfos &robotInfo)
{
    std::lock_guard<std::mutex> lock(_mutexRobot);
    _robot = robotInfo;
}

/// Translate the robot’s position
void PositionController::translate(float deltaX, float deltaY)
{
    std::lock_guard<std::mutex> lock(_mutexRobot);
    _robot.center.x() += deltaX;
    _robot.center.y() += deltaY;
}

void PositionController::translate(const Eigen::Vector2f &delta)
{
    std::lock_guard<std::mutex> lock(_mutexRobot);
    _robot.center += delta;
}

// Rotate the robot’s orientation
void PositionController::rotate(float deltaAngle)
{
    std::lock_guard<std::mutex> lock(_mutexRobot);
    _robot._currentRobotAngle += deltaAngle;

    // Ensure angle is within 0-360 degrees for consistency
    if (_robot._currentRobotAngle >= 360.0f)
    {
        _robot._currentRobotAngle -= 360.0f;
    }
    else if (_robot._currentRobotAngle < 0.0f)
    {
        _robot._currentRobotAngle += 360.0f;
    }
}

/// @brief Function to update the robot's position based on the distance each wheel has traveled (mean to be called often)
/// @param leftMotorCm
/// @param rightMotorCm
void PositionController::updatePositionFromMotors(float leftMotorCm, float rightMotorCm)
{
    std::lock_guard<std::mutex> lock(_mutexRobot);

    // Calculate the distance moved by each wheel
    float deltaDistance = (leftMotorCm + rightMotorCm) / 2.0f;        // Average distance
    float deltaTheta = (rightMotorCm - leftMotorCm) / TRACK_WIDTH_CM; // Rotation angle in radians

    // Update robot position based on deltaDistance and deltaTheta
    float currentAngleRad = _robot._currentRobotAngle * M_PI / 180.0f; // Convert current angle to radians
    float newAngleRad = currentAngleRad + deltaTheta;

    // Calculate the new position in global coordinates
    if (fabs(deltaTheta) < 1e-5)
    { // Straight line movement (small rotation threshold)
        _robot.center.x() += deltaDistance * cos(currentAngleRad);
        _robot.center.y() += deltaDistance * sin(currentAngleRad);
    }
    else
    { // Arc movement
        float radius = deltaDistance / deltaTheta;
        _robot.center.x() += radius * (sin(newAngleRad) - sin(currentAngleRad));
        _robot.center.y() += radius * (cos(currentAngleRad) - cos(newAngleRad));
    }

    // Update the robot's angle in degrees, normalizing to [0, 360)
    _robot._currentRobotAngle = fmod((_robot._currentRobotAngle + deltaTheta * 180.0f / M_PI), 360.0f);
    if (_robot._currentRobotAngle < 0)
        _robot._currentRobotAngle += 360.0f;
    std::cout << TAG <<
     "_robot._currentRobotAngle:" << _robot._currentRobotAngle <<
     ", _robot.center (currentRobotPosition):" <<  _robot.center << std::endl;
}