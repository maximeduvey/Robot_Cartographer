#pragma once

#include "Mapper.h"
#include "CommonDebugFunction.h"

class CreationTools
{
public:
    static std::vector<Point> generateTestPoints(Eigen::Vector3f decalage = {0.0f, 0.0f, 0.0f});
    static std::vector<Point> generateCloudOfPointsForObj(const Object3D &obj);

private:
};