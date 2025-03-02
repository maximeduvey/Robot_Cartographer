#include "CommonDebugFunction.h"

#include "SingletonVisualizerManager.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <utility>
#include <iostream>

static int unic_id = 3000;

void CommonDebugFunction::display3dObject(std::vector<Object3D> objects)
{
    for (const auto& obj : objects)
    {
        std::cout << "Object Center: (" << obj.center.x() << ", " << obj.center.y() << ", " << obj.center.z() << ")\n";
        std::cout << "Object Size: (Length: " << obj.size.x()
            << ", Width: " << obj.size.y()
            << ", Height: " << obj.size.z() << ")\n";
    }
}

void CommonDebugFunction::savePointCloudToFile(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::string& filename)
{
    if (cloud.empty())
    {
        std::cerr << "Error: Point cloud is empty!" << std::endl;
        return;
    }

    std::ofstream outFile(filename);
    if (!outFile.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    std::cout << "Saving " << cloud.size() << " points to " << filename << std::endl;

    for (const auto& point : cloud)
    {
        outFile << point.x << " " << point.y << " " << point.z << "\n";
    }

    outFile.close();

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>(filename + ".pcd", cloud, false);

    std::cout << "Point cloud saved to " << filename << std::endl;
}

void CommonDebugFunction::savePathToPointCloud(const std::vector<std::pair<int, int>>& path, const std::string& filename)
{
    // Create a PCL PointCloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    for (const auto& point : path)
    {
        cloud->points.emplace_back(point.first, point.second, 0);
    }

    // Set the metadata for the point cloud
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // Save to a PCD file
    pcl::PCDWriter writer;
    if (writer.write<pcl::PointXYZ>(filename + ".pcd", *cloud, false) == -1)
    {
        std::cerr << "Error: Could not write PCD file: " << filename << std::endl;
    }
    else
    {
        std::cout << "Path saved as point cloud to " << filename << std::endl;
    }
}

// Visualize occupancy grid and path in PCL Viewer
void CommonDebugFunction::visualizeOccupancyGridAndPath(
    const std::vector<std::vector<int>>& occupancyGrid,
    const std::vector<std::pair<int, int>>& path,
    const std::string& filename)
{
    // Create PointCloud for occupancy grid
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (int y = 0; y < occupancyGrid.size(); ++y)
    {
        for (int x = 0; x < occupancyGrid[y].size(); ++x)
        {
            if (occupancyGrid[y][x] > 0)
            {
                objectCloud->points.emplace_back(x, y, 0, 255, 0, 0);
            }
        }
    }

    for (const auto& point : path)
    {
        objectCloud->points.emplace_back(point.first, point.second, 0, 0, 255, 0);
    }

    objectCloud->width = objectCloud->points.size();
    objectCloud->height = 1;
    objectCloud->is_dense = true;

    pcl::PCDWriter writer;
    if (writer.write<pcl::PointXYZRGB>(filename + ".pcd", *objectCloud, false) == -1)
    {
        std::cerr << "Error: Could not write PCD file: " << filename << std::endl;
    }
    else
    {
        std::cout << "Occupancy grid and path saved to " << filename << std::endl;
    }
}

void CommonDebugFunction::savePointsToFile(
    const std::vector<Point>& points,
    const std::string& filename)
{
    if (points.empty())
    {
        std::cerr << "Error: Point vector is empty!" << std::endl;
        return;
    }

    std::ofstream outFile(filename);
    if (!outFile.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // std::cout << "Saving " << points.size() << " points to " << filename << std::endl;

    for (const auto& point : points)
    {
        outFile << point.pos.x() << " " << point.pos.y() << "\n";
    }

    outFile.close();
    std::cout << "Points saved to " << filename << std::endl;
}

void CommonDebugFunction::saveOccupancyGridToFile(
    const std::vector<std::vector<std::pair<bool, int>>>& map,
    const std::string& filename,
    const Eigen::Vector3f& robotStart,
    const Eigen::Vector3f& goalPos)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    std::ofstream outFile(filename);
    if (!outFile.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // std::cout << TAG << " column:" << map.size() << ", row:" << map[0].size() << std::endl;
    int y = 0;
    for (const auto& row : map)
    {
        int x = 0;
        for (const auto& cell : row)
        {
            if (cell.second > 0)
                cloud->points.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.0f);
            outFile << (cell.second > 0 ? "[x]" : "[ ]") << " "; // Occupied or Free
            ++x;
        }
        ++y;
        outFile << "\n";
    }


    outFile.close();

    CommonDebugFunction::addObjectToCloud({ robotStart }, cloud, 255, 0, 0);
    CommonDebugFunction::addObjectToCloud({ goalPos }, cloud, 0, 0, 255);
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // Save to a PCD file
    pcl::PCDWriter writer;
    if (writer.write<pcl::PointXYZRGB>(filename + ".pcd", *cloud, false) == -1)
    {
        std::cout << "Failed to save Occupancy " << std::endl;
    }
    else
    {
        std::cout << "Occupancy grid saved to " << filename << std::endl;
    }
};

void CommonDebugFunction::saveOccupancyGridToFile(
    const std::vector<std::vector<int>>& grid,
    const std::string& filename,
    const Eigen::Vector3f& robotStart,
    const Eigen::Vector3f& goalPos)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (grid.empty() || grid[0].empty())
    {
        std::cerr << "Error: Occupancy grid is empty!" << std::endl;
        return;
    }

    std::ofstream outFile(filename);
    if (!outFile.is_open())
    {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return;
    }
    int y = 0;
    for (const auto& row : grid)
    {
        int x = 0;
        for (const auto& cell : row)
        {
            if (cell > 0)
                cloud->points.emplace_back(x, y, 0, 255, 255, 255);
            outFile << (cell > 0 ? "[x]" : "[ ]") << " "; // Occupied or Free
            ++x;
        }
        ++y;
        outFile << "\n";
    }

    outFile.close();

    CommonDebugFunction::addObjectToCloud({ robotStart }, cloud, 255, 0, 0);
    CommonDebugFunction::addObjectToCloud({ goalPos }, cloud, 0, 0, 255);
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // Save to a PCD file
    pcl::PCDWriter writer;
    if (writer.write<pcl::PointXYZRGB>(filename + ".pcd", *cloud, false) != -1)
    {

        std::cout << "Occupancy grid saved to " << filename << std::endl;
    }
    else
    {

        std::cout << "Failed to save occupancy grid " << filename << std::endl;
    }
}

pcl::PointXYZRGB CommonDebugFunction::eigneVecToPoint(const Eigen::Vector3f& point, uint8_t r, uint8_t g, uint8_t b)
{
    std::cout << TAG << std::endl;
    pcl::PointXYZRGB pclPoint;
    pclPoint.x = point.x();
    pclPoint.y = point.y();
    pclPoint.z = point.z();
    pclPoint.r = r;
    pclPoint.g = g;
    pclPoint.b = b;
    return pclPoint;
}

/* void CommonDebugFunction::addPointToCloud(
    const Eigen::Vector3f &point,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    uint8_t r, uint8_t g, uint8_t b)
{
    cloud->points.push_back(CommonDebugFunction::eigneVecToPoint(point, r, g, b));
} */

void CommonDebugFunction::addPointsToCloud(
    const std::vector<Eigen::Vector3f>& pathPoints,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    uint8_t r, uint8_t g, uint8_t b,
    const Eigen::Vector3f shifter/*  = {0.0f, 0.0f, 0.0f} */)
{
    std::cout << TAG << "Path containing :" << pathPoints.size() << std::endl;
    static int nbr = 10;
    for (const auto& point : pathPoints)
    {
        std::string str = std::string("[x:" + std::to_string(point[0]) + ",y:" + std::to_string(point[1]) + ",z:" + std::to_string(point[2]) + "]");
        std::cout << TAG << "Path " << str << std::endl;
        auto pclpoint = CommonDebugFunction::eigneVecToPoint((point + shifter), r, g, b);
        cloud->points.push_back(pclpoint);
        SingletonVisualizerManager::getInstance().addTextToPoint(pclpoint, str, r, g, b, ++nbr);
        //CommonDebugFunction::addPointToCloud(point, cloud, r, g, b);
    }
}

/// @brief return the center
/// @param object 
/// @param cloud 
/// @param r 
/// @param g 
/// @param b 
pcl::PointXYZRGB CommonDebugFunction::addObjectToCloud(
    const Object3D& ob,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    uint8_t r, uint8_t g, uint8_t b,
    const Eigen::Vector3f shifter /* = {0.0f, 0.0f, 0.0f} */)
{

    Object3D object(ob);
    //std::cout << TAG << " size :" << object.size << std::endl;
    object.center = object.center + shifter;
    // Represent the object by its center position
    pcl::PointXYZRGB pclPoint;
    pclPoint.x = object.center.x();
    pclPoint.y = object.center.y();
    pclPoint.z = object.center.z(); // Objects are in 3D space
    pclPoint.r = r;                 // Red color
    pclPoint.g = g;
    pclPoint.b = b;
    cloud->points.push_back(pclPoint);

    if (object.size.x() > 1 || object.size.y() > 1 || object.size.z() > 1 )
    {
        // Optionally, add points for object boundaries or corners
        Eigen::Vector3f halfSize = object.size * 0.5f;
        std::vector<Eigen::Vector3f> corners = {
            object.center + Eigen::Vector3f(-halfSize.x(), -halfSize.y(), -halfSize.z()),
            object.center + Eigen::Vector3f(halfSize.x(), -halfSize.y(), -halfSize.z()),
            object.center + Eigen::Vector3f(halfSize.x(), halfSize.y(), -halfSize.z()),
            object.center + Eigen::Vector3f(-halfSize.x(), halfSize.y(), -halfSize.z()),
            object.center + Eigen::Vector3f(-halfSize.x(), -halfSize.y(), halfSize.z()),
            object.center + Eigen::Vector3f(halfSize.x(), -halfSize.y(), halfSize.z()),
            object.center + Eigen::Vector3f(halfSize.x(), halfSize.y(), halfSize.z()),
            object.center + Eigen::Vector3f(-halfSize.x(), halfSize.y(), halfSize.z()),
        };

        for (const auto& corner : corners)
        {
            pcl::PointXYZRGB cornerPoint;
            cornerPoint.x = corner.x();
            cornerPoint.y = corner.y();
            cornerPoint.z = corner.z();
            cornerPoint.r = r;
            cornerPoint.g = g;
            cornerPoint.b = b;
            cloud->points.push_back(cornerPoint);
        }
    }
    return pclPoint;
}

void CommonDebugFunction::addObjectsToCloud(
    const std::vector<std::shared_ptr<Object3D>>& detectedObjects,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    uint8_t r, uint8_t g, uint8_t b,
    const Eigen::Vector3f shifter/*  = {0.0f, 0.0f, 0.0f }*/)
{
    //std::cout << TAG << std::endl;
    // Add detected objects with red color
    for (const auto& object : detectedObjects)
    {
        auto pt = CommonDebugFunction::addObjectToCloud(*object.get(), cloud, r, g, b, shifter);
        SingletonVisualizerManager::getInstance().addTextToPoint(pt, std::to_string(object.get()->id), RGB_YELLOW, ++unic_id);
    }
}

void CommonDebugFunction::writeCloudWriter(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std::string& filename)
{
    std::cout << TAG << std::endl;
    // Set cloud metadata
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // Write the point cloud to a PCD file
    pcl::PCDWriter writer;
    if (writer.write<pcl::PointXYZRGB>(filename + ".pcd", *cloud, false) == -1)
    {
        std::cerr << "Error: Could not write PCD file: " << filename << std::endl;
    }
    else
    {
        std::cout << "Point cloud saved to " << filename << std::endl;
    }
}

void CommonDebugFunction::mergePointClouds(const pcl::PointCloud<pcl::PointXYZ>& inputCloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outputCloud,
    uint8_t r, uint8_t g, uint8_t b,
    const Eigen::Vector3f shifter /* = {0.0f, 0.0f, 0.0f} */)
{
    std::cout << TAG << std::endl;
    for (const auto& point : inputCloud.points)
    {
        pcl::PointXYZRGB coloredPoint;
        coloredPoint.x = point.x + shifter.x();
        coloredPoint.y = point.y + shifter.y();
        coloredPoint.z = point.z + shifter.z();
        coloredPoint.r = r;
        coloredPoint.g = g;
        coloredPoint.b = b;

        outputCloud->points.push_back(coloredPoint);
    }
}
/* 
// Function to save path and detected objects to a PCD file
void CommonDebugFunction::savePointCloudToFile(
    const std::vector<Eigen::Vector3f>& pathPoints,
    const std::vector<Object3D>& detectedObjects,
    const std::string& filename)
{
    // Create a PointCloud to store all points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    CommonDebugFunction::addPointsToCloud(pathPoints, cloud, RGB_GREEN);
    CommonDebugFunction::addObjectsToCloud(detectedObjects, cloud, RGB_RED);
    CommonDebugFunction::writeCloudWriter(cloud, filename);
}

void CommonDebugFunction::savePointCloudToFile(
    const pcl::PointCloud<pcl::PointXYZ>& cloud2,
    const std::vector<Eigen::Vector3f>& pathPoints,
    const std::vector<Object3D>& detectedObjects,
    const std::string& filename)
{
    // Create a PointCloud to store all points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    CommonDebugFunction::mergePointClouds(cloud2, cloud, RGB_WHITE);
    CommonDebugFunction::addPointsToCloud(pathPoints, cloud, RGB_GREEN);
    CommonDebugFunction::addObjectsToCloud(detectedObjects, cloud, RGB_RED);
    CommonDebugFunction::writeCloudWriter(cloud, filename);
}

void CommonDebugFunction::savePointCloudToFile(
    const Object3D& robot,
    const pcl::PointCloud<pcl::PointXYZ>& cloud2,
    const std::vector<Eigen::Vector3f>& pathPoints,
    const std::vector<Object3D>& detectedObjects,
    const std::string& filename)
{
    // Create a PointCloud to store all points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    CommonDebugFunction::addObjectToCloud(robot, cloud, RGB_LIGHT_BLUE);
    CommonDebugFunction::mergePointClouds(cloud2, cloud, RGB_WHITE);
    CommonDebugFunction::addPointsToCloud(pathPoints, cloud, RGB_GREEN);
    CommonDebugFunction::addObjectsToCloud(detectedObjects, cloud, RGB_RED);
    CommonDebugFunction::writeCloudWriter(cloud, filename);
} */

void CommonDebugFunction::savePointCloudToFile(
    const Object3D& robot,
    const Eigen::Vector3f& destination,
    const pcl::PointCloud<pcl::PointXYZ>& cloud2,
    const std::vector<Eigen::Vector3f>& pathPoints,
    const std::vector<std::shared_ptr<Object3D>>& detectedObjects,
    const std::vector<SectorConeOfVision> &mapDetectedObject,
    const std::string& filename,
    const Eigen::Vector3f shifter /* = {0.0f, 0.0f, 0.0f} */)
{
    //std::cout << TAG << "A" << std::endl;
    //SingletonVisualizerManager::getInstance().clearPointCloud();
    //std::cout << TAG << "B" <<std::endl;
    // Create a PointCloud to store all points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    auto start = CommonDebugFunction::addObjectToCloud(robot, cloud, RGB_ORANGE, shifter);
    SingletonVisualizerManager::getInstance().addTextToPoint(start, "START", RGB_YELLOW, ++unic_id);
    // auto destPoint = CommonDebugFunction::eigneVecToPoint((destination + shifter), RGB_YELLOW);
    // cloud->points.push_back(destPoint);
    // SingletonVisualizerManager::getInstance().addTextToPoint(destPoint, "END", RGB_DARK_GREEN, ++unic_id);
    // CommonDebugFunction::mergePointClouds(cloud2, cloud, RGB_WHITE);
    // CommonDebugFunction::addPointsToCloud(pathPoints, cloud, RGB_GREEN, shifter);
    CommonDebugFunction::addObjectsToCloud(detectedObjects, cloud, RGB_PURPLE, shifter);
    for (const auto &sector : mapDetectedObject) {
        CommonDebugFunction::addObjectsToCloud(sector._detectedObjects, cloud, RGB_RED, shifter);
    }

     auto starttime = std::chrono::high_resolution_clock::now();
     SingletonVisualizerManager::getInstance().updatePointCloud(cloud);
     auto diff = std::chrono::high_resolution_clock::now() - starttime;
     std::cout << "Time execution SingletonVisualizerManager::getInstance().updatePointCloud(cloud): " << diff.count() << std::endl;
}