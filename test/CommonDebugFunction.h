
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

#define RGB_RED 255, 0, 0
#define RGB_GREEN 0, 255, 0
#define RGB_BLUE 0, 0, 255
#define RGB_YELLOW 255, 255, 0
#define RGB_CYAN 0, 255, 255
#define RGB_MAGENTA 255, 0, 255
#define RGB_WHITE 255, 255, 255
#define RGB_BLACK 0, 0, 0
#define RGB_ORANGE 255, 165, 0
#define RGB_PURPLE 128, 0, 128
#define RGB_PINK 255, 192, 203
#define RGB_GRAY 128, 128, 128
#define RGB_BROWN 165, 42, 42
#define RGB_LIGHT_BLUE 173, 216, 230
#define RGB_DARK_GREEN 0, 100, 0

class CommonDebugFunction
{

public:
    static void display3dObject(std::vector<Object3D> objects);

    static void savePointCloudToFile(
        const std::vector<Eigen::Vector3f> &pathPoints,
        const std::vector<Object3D> &detectedObjects,
        const std::string &filename);

    static void savePointCloudToFile(
        const pcl::PointCloud<pcl::PointXYZ> &cloud,
        const std::string &filename);

    static void savePointsToFile(
        const std::vector<Point> &points,
        const std::string &filename);

    static pcl::PointXYZRGB eigneVecToPoint( const Eigen::Vector3f &point,  uint8_t r, uint8_t g, uint8_t b);

    static void saveOccupancyGridToFile(
        const std::vector<std::vector<std::pair<bool, int>>> &map,
        const std::string &filename,
        const Eigen::Vector3f &robotStart,
        const Eigen::Vector3f &goalPos);

    static void saveOccupancyGridToFile(
        const std::vector<std::vector<int>> &grid,
        const std::string &filename,
        const Eigen::Vector3f &robotStart,
        const Eigen::Vector3f &goalPos);

    static void savePathToPointCloud(
        const std::vector<std::pair<int, int>> &path,
        const std::string &filename);

    static void visualizeOccupancyGridAndPath(
        const std::vector<std::vector<int>> &occupancyGrid,
        const std::vector<std::pair<int, int>> &path,
        const std::string &filename);

    static void addPointsToCloud(
        const std::vector<Eigen::Vector3f> &pathPoints,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        uint8_t r = 255, uint8_t g = 255, uint8_t b = 255,
        const Eigen::Vector3f shifter = {0.0f, 0.0f, 0.0f});

/*     static void addPointToCloud(
        const Eigen::Vector3f &point,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        uint8_t r, uint8_t g, uint8_t b,
        const Eigen::Vector3f shifter = {0.0f, 0.0f, 0.0f}); */

    static pcl::PointXYZRGB addObjectToCloud(
        const Object3D &ob,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        uint8_t r = 255, uint8_t g = 255, uint8_t b = 255,
        const Eigen::Vector3f shifter = {0.0f, 0.0f, 0.0f});

    static void addObjectsToCloud(
        const std::vector<std::shared_ptr<Object3D>> &detectedObjects,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        uint8_t r = 255, uint8_t g = 255, uint8_t b = 255,
        const Eigen::Vector3f shifter = {0.0f, 0.0f, 0.0f});

    static void writeCloudWriter(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::string &filename);

    static void mergePointClouds(
        const pcl::PointCloud<pcl::PointXYZ> &inputCloud,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputCloud,
        uint8_t r = 255, uint8_t g = 255, uint8_t b = 255,
        const Eigen::Vector3f shifter = {0.0f, 0.0f, 0.0f});
/* 
    static void savePointCloudToFile(
        const pcl::PointCloud<pcl::PointXYZ> &cloud2,
        const std::vector<Eigen::Vector3f> &pathPoints,
        const std::vector<Object3D> &detectedObjects,
        const std::string &filename);

    static void savePointCloudToFile(
        const Object3D &robot,
        const pcl::PointCloud<pcl::PointXYZ> &cloud2,
        const std::vector<Eigen::Vector3f> &pathPoints,
        const std::vector<Object3D> &detectedObjects,
        const std::string &filename); */

    static void savePointCloudToFile(
        const Object3D &robot,
        const Eigen::Vector3f &destination,
        const pcl::PointCloud<pcl::PointXYZ> &cloud2,
        const std::vector<Eigen::Vector3f> &pathPoints,
        const std::vector<std::shared_ptr<Object3D>> &detectedObjects,
        const std::vector<SectorConeOfVision> &mapDetectedObject,
        const std::string &filename,
        const Eigen::Vector3f shifter = {0.0f, 0.0f, 0.0f});
};
