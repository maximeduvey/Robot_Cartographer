#include "CommonDebugFunction.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <utility>
#include <iostream>

void CommonDebugFunction::display3dObject(std::vector<Object3D> objects)
{
    for (const auto &obj : objects)
    {
        std::cout << "Object Center: (" << obj.center.x() << ", " << obj.center.y() << ", " << obj.center.z() << ")\n";
        std::cout << "Object Size: (Length: " << obj.size.x()
                  << ", Width: " << obj.size.y()
                  << ", Height: " << obj.size.z() << ")\n";
    }
}

void CommonDebugFunction::savePointCloudToFile(
    const pcl::PointCloud<pcl::PointXYZ> &cloud,
    const std::string &filename)
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

    for (const auto &point : cloud)
    {
        outFile << point.x << " " << point.y << " " << point.z << "\n";
    }

    outFile.close();

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>(filename + ".pcd", cloud, false);

    std::cout << "Point cloud saved to " << filename << std::endl;
}

void CommonDebugFunction::savePathToPointCloud(const std::vector<std::pair<int, int>> &path, const std::string &filename)
{
    // Create a PCL PointCloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    for (const auto &point : path)
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
    const std::vector<std::vector<int>> &occupancyGrid,
    const std::vector<std::pair<int, int>> &path,
    const std::string &filename)
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

    for (const auto &point : path)
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
    const std::vector<Point> &points,
    const std::string &filename)
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

    std::cout << "Saving " << points.size() << " points to " << filename << std::endl;

    for (const auto &point : points)
    {
        outFile << point.pos[0] << " " << point.pos[1] << "\n";
    }

    outFile.close();
    std::cout << "Points saved to " << filename << std::endl;
}

void CommonDebugFunction::saveOccupancyGridToFile(
    const std::vector<std::vector<std::pair<bool, int>>> &map,
    const std::string &filename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::ofstream outFile(filename);
    if (!outFile.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    std::cout << TAG << " column:" << map.size() << ", row:" << map[0].size() << std::endl;
    int y = 0;
    for (const auto &row : map)
    {
        int x = 0;
        for (const auto &cell : row)
        {
            cloud->points.emplace_back(x, y, 0);
            if (cell.second > 0)
            {
                outFile << "[x] "; // Occupied
            }
            else
            {
                outFile << "[ ] "; // Free
            }
        }
        outFile << "\n";
    }

    outFile.close();

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // Save to a PCD file
    pcl::PCDWriter writer;
    if (writer.write<pcl::PointXYZ>(filename + ".pcd", *cloud, false) == -1)

        std::cout << "Occupancy grid saved to " << filename << std::endl;
};

void CommonDebugFunction::saveOccupancyGridToFile(
    const std::vector<std::vector<int>> &grid,
    const std::string &filename)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
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
    for (const auto &row : grid)
    {
        int x = 0;
        for (const auto &cell : row)
        {
            if (cell > 0)
                cloud->points.emplace_back(x, y, 0);
            outFile << (cell > 0 ? "[x]" : "[ ]") << " "; // Occupied or Free
            ++x;
        }
        ++y;
        outFile << "\n";
    }

    outFile.close();

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // Save to a PCD file
    pcl::PCDWriter writer;
    if (writer.write<pcl::PointXYZ>(filename + ".pcd", *cloud, false) == -1)
        std::cout << "Occupancy grid saved to " << filename << std::endl;
}

// Function to save path and detected objects to a PCD file
void CommonDebugFunction::savePointCloudToFile(
    const std::vector<Eigen::Vector2f>& pathPoints,
    const std::vector<Object3D>& detectedObjects,
    const std::string& filename
) {
    // Create a PointCloud to store all points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Add path points with green color
    for (const auto& point : pathPoints) {
        pcl::PointXYZRGB pclPoint;
        pclPoint.x = point.x();
        pclPoint.y = point.y();
        pclPoint.z = 0.0f;  // Path points are 2D, so z = 0
        pclPoint.r = 0;
        pclPoint.g = 255;   // Green color
        pclPoint.b = 0;
        cloud->points.push_back(pclPoint);
    }

    // Add detected objects with red color
    for (const auto& object : detectedObjects) {
        // Represent the object by its center position
        pcl::PointXYZRGB pclPoint;
        pclPoint.x = object.position.x();
        pclPoint.y = object.position.y();
        pclPoint.z = object.position.z();  // Objects are in 3D space
        pclPoint.r = 255;  // Red color
        pclPoint.g = 0;
        pclPoint.b = 0;
        cloud->points.push_back(pclPoint);

        // Optionally, add points for object boundaries or corners
        Eigen::Vector3f halfSize = object.size * 0.5f;
        std::vector<Eigen::Vector3f> corners = {
            object.position + Eigen::Vector3f(-halfSize.x(), -halfSize.y(), -halfSize.z()),
            object.position + Eigen::Vector3f(halfSize.x(), -halfSize.y(), -halfSize.z()),
            object.position + Eigen::Vector3f(halfSize.x(), halfSize.y(), -halfSize.z()),
            object.position + Eigen::Vector3f(-halfSize.x(), halfSize.y(), -halfSize.z()),
            object.position + Eigen::Vector3f(-halfSize.x(), -halfSize.y(), halfSize.z()),
            object.position + Eigen::Vector3f(halfSize.x(), -halfSize.y(), halfSize.z()),
            object.position + Eigen::Vector3f(halfSize.x(), halfSize.y(), halfSize.z()),
            object.position + Eigen::Vector3f(-halfSize.x(), halfSize.y(), halfSize.z()),
        };

        for (const auto& corner : corners) {
            pcl::PointXYZRGB cornerPoint;
            cornerPoint.x = corner.x();
            cornerPoint.y = corner.y();
            cornerPoint.z = corner.z();
            cornerPoint.r = 255;
            cornerPoint.g = 0;
            cornerPoint.b = 0;
            cloud->points.push_back(cornerPoint);
        }
    }

    // Set cloud metadata
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // Write the point cloud to a PCD file
    pcl::PCDWriter writer;
    if (writer.write<pcl::PointXYZRGB>(filename + ".pcd", *cloud, false) == -1) {
        std::cerr << "Error: Could not write PCD file: " << filename << std::endl;
    } else {
        std::cout << "Point cloud saved to " << filename << std::endl;
    }
}