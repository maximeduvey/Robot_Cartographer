#include "SingletonVisualizerManager.h"

#include "CommonSpaceRepresentation.h"

SingletonVisualizerManager &SingletonVisualizerManager::getInstance()
{
    static SingletonVisualizerManager instance;
    return instance;
}

// Initialize the viewer
void SingletonVisualizerManager::initialize()
{
    std::lock_guard<std::mutex> lock(viewerMutex);
    if (!viewer)
    {
        viewer = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
        viewer->setBackgroundColor(0, 0, 0); // Black background
        viewer->addCoordinateSystem(1.0);    // Add a coordinate system
        viewer->initCameraParameters();
    }
}

// Update the point cloud in the viewer
void SingletonVisualizerManager::updatePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &cloudID /*  = "sample cloud" */)
{
    std::lock_guard<std::mutex> lock(viewerMutex);
    if (viewer)
    {
        if (viewer->contains("cloud")) {
            viewer->removePointCloud("cloud");
        }
        if (!viewer->updatePointCloud(cloud, cloudID))
        {
            viewer->addPointCloud<pcl::PointXYZ>(cloud, cloudID);
        }
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudID);
    }
}

// Update the point cloud in the viewer
void SingletonVisualizerManager::updatePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::string &cloudID /*  = "sample cloud" */)
{
    std::lock_guard<std::mutex> lock(viewerMutex);
    std::cout << TAG << std::endl;
    if (viewer)
    {
        if (viewer->contains("cloud")) {
            viewer->removePointCloud("cloud");
        }
        if (!viewer->updatePointCloud(cloud, cloudID))
        {
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud, cloudID);
        }
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudID);
    }
    std::cout << TAG << " debug 2" << std::endl;
}

/// @brief 
/// @param point 
/// @param text 
/// @param r 
/// @param g 
/// @param b 
/// @param id 
void SingletonVisualizerManager::addTextToPoint(const pcl::PointXYZRGB &point, std::string text, uint8_t r, uint8_t g, uint8_t b, int id)
{
    std::lock_guard<std::mutex> lock(viewerMutex);
    viewer->addText3D<pcl::PointXYZRGB>(
        text,                 // Text to display
        point,                     // Position of the text
        0.4,                       // Text scale
        r,g,b,
       std::to_string(id) // Unique ID for each text
    );
}

void SingletonVisualizerManager::clearViewer()
{
    std::lock_guard<std::mutex> lock(viewerMutex);
    if (viewer)
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
    }
}

// Run the viewer loop (non-blocking)
void SingletonVisualizerManager::spinOnce(int delay /*  = 100 */)
{
    std::lock_guard<std::mutex> lock(viewerMutex);
    if (viewer)
    {
        viewer->spinOnce(delay);
    }
}

// Check if the viewer was stopped
bool SingletonVisualizerManager::wasStopped()
{
    std::lock_guard<std::mutex> lock(viewerMutex);
    // std::cout << TAG << (viewer ? (viewer->wasStopped() ? "stopped" : "running") : "viewer is dead") <<std::endl;
    return viewer ? viewer->wasStopped() : true;
}

void SingletonVisualizerManager::clearPointCloud() {
    std::lock_guard<std::mutex> lock(viewerMutex);
    viewer->removeAllPointClouds();
}

