
#include "Mapper.h"

#include "CommonDebugFunction.h"

#include "CreationTools.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <iostream>

int main()
{
    std::cout << TAG << std::endl;
    auto vec = CreationTools::generateTestPoints();

    Mapper map;
    map.startDataParsing();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    FieldPoints field;
    field.points = vec;

    CommonDebugFunction::savePointsToFile(vec, "vec.log");

    map.addDataToParse(field);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    map._robotInfos.getMovementFromLastMeasure();
    // map.addDataToParse(field);
    // CommonDebugFunction::saveOccupancyGridToFile(map._occupancy_grid, "output.log", {0,0,0},{0,0,0});

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::cout << "DEBUG 1" << std::endl;
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
    std::cout << "DEBUG 2" << std::endl;

    std::cout << "Generated point cloud with " << cloud->points.size() << " points." << std::endl;

    // Step 2: Visualize the Point Cloud
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    if (!viewer)
    {
        std::cerr << "Viewer pointer is null!" << std::endl;
    }
    else
    {
        viewer->setBackgroundColor(0, 0, 0);                         // Black background
        viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud"); // Add the point cloud
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
        viewer->addCoordinateSystem(1.0); // Add a coordinate system for reference
        viewer->initCameraParameters();
        std::cout << "DEBUG 3" << std::endl;

        // Step 3: Interactive Loop
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100); // Allow interaction with the visualization window
        }

        std::cout << "DEBUG 4" << std::endl;
    }
    while (true)
        ;

    return 0;
}
