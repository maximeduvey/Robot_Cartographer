
#pragma once

#include "Mapper.h"

#include <iostream>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <iostream>
#include <fstream>
#include <string>

class CommonDebugFunction
{
public:
    static void savePointCloudToFile(
        const pcl::PointCloud<pcl::PointXYZ> &cloud,
        const std::string &filename);

    static void savePointsToFile(
        const std::vector<Point> &points,
        const std::string &filename);

    static void saveOccupancyGridToFile(
        const std::vector<std::vector<std::pair<bool, int>>> &map,
        const std::string &filename);

    static void saveOccupancyGridToFile(
        const std::vector<std::vector<int>> &grid,
        const std::string &filename);

    static void savePathToPointCloud(
        const std::vector<std::pair<int, int>> &path,
        const std::string &filename);

    static void visualizeOccupancyGridAndPath(
        const std::vector<std::vector<int>> &occupancyGrid,
        const std::vector<std::pair<int, int>> &path,
        const std::string &filename);
};
