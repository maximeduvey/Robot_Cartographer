
#include "Mapper.h"

#include "CommonDebugFunction.h"

#include "CreationTools.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <iostream>

#include "SingletonVisualizerManager.h"

Mapper map;

void addPointCloudToVisualizer()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Generate a sine wave pattern
    for (float z = 0.0; z <= 10.0; z += 0.1)
    {
        for (float theta = 0.0; theta <= 2 * M_PI; theta += 0.1)
        {
            pcl::PointXYZ point;
            point.x = std::sin(theta); // X follows a sine wave
            point.y = std::cos(theta); // Y follows a cosine wave
            point.z = z;               // Z increases linearly
            cloud->points.push_back(point);
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1; // Unorganized point cloud
    cloud->is_dense = true;

    SingletonVisualizerManager::getInstance().updatePointCloud(cloud, "calculated cloud");
}

void addData()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    FieldPoints field;
    map._robotInfos.getMovementFromLastMeasure();

    field.points = CreationTools::generateTestPoints({-30, -30, 0});
/*     auto points = CreationTools::generateTestPoints({-50, -30, 0});
    field.points.insert(field.points.end(), points.begin(), points.end());
    
    points = CreationTools::generateTestPoints({-30, -50, 0});
    field.points.insert(field.points.end(), points.begin(), points.end());

    points = CreationTools::generateTestPoints({-30, -50, 0});
    field.points.insert(field.points.end(), points.begin(), points.end());

    points = CreationTools::generateTestPoints({-100, -150, 0});
    field.points.insert(field.points.end(), points.begin(), points.end()); */

    map.addDataToParse(field);
}

int main()
{
    std::cout << TAG << std::endl;
    auto vec = CreationTools::generateTestPoints();

    SingletonVisualizerManager::getInstance().initialize();

    map.startDataParsing();

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    FieldPoints field;
    field.points = vec;

    CommonDebugFunction::savePointsToFile(vec, "vec.log");
    map.addDataToParse(field);

    // CommonDebugFunction::saveOccupancyGridToFile(map._occupancy_grid, "output.log", {0,0,0},{0,0,0});

    // addPointCloudToVisualizer();
    std::thread(&addData).detach();
    while (!SingletonVisualizerManager::getInstance().wasStopped())
    {
        SingletonVisualizerManager::getInstance().spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
}
