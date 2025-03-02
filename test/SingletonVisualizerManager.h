
#pragma once

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <mutex>
#include <memory>
#include <iostream>
#include <Eigen/Dense>

class SingletonVisualizerManager {
public:
    // Get the singleton instance
    static SingletonVisualizerManager& getInstance();

    // Initialize the viewer
    void initialize();

    // Update the point cloud in the viewer
    void updatePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& cloudID = "sample cloud");
    void updatePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& cloudID = "sample cloud");

    void addTextToPoint(const pcl::PointXYZRGB &point, std::string text, uint8_t r, uint8_t g, uint8_t b, int id);

    void clearPointCloud();
    // Run the viewer loop (non-blocking)
    void spinOnce(int delay = 100);

    // Check if the viewer was stopped
    bool wasStopped();

    
    void clearViewer();

private:
    // Private constructor for singleton
    SingletonVisualizerManager() : viewer(nullptr) {}

    // No copying or assignment
    SingletonVisualizerManager(const SingletonVisualizerManager&) = delete;
    SingletonVisualizerManager& operator=(const SingletonVisualizerManager&) = delete;

    pcl::visualization::PCLVisualizer::Ptr viewer; // PCL visualizer
    std::mutex viewerMutex;                        // Protect access to the viewer
};
