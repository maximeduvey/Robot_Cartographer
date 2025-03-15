#pragma once

#include "Mapper.h"

class CreationTools
{
public:
    static std::vector<Point> generateTestPoints(Eigen::Vector3f decalage = {0.0f, 0.0f, 0.0f});
    static std::vector<Point> generateCloudOfPointsForObj(const Object3D &obj);

    static std::vector<Point> generateLineOfPointsForObj(const Object3D &obj);
    static std::vector<Point> generateLineOfPointsFromTo(const Eigen::Vector3f &start, const Eigen::Vector3f &end);
private:
};