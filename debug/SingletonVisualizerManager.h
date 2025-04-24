
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <mutex>
#include <memory>
#include <iostream>
#include <Eigen/Dense>
#include <thread>

#include "OpenGLRenderer.h"


class SingletonVisualizerManager {
public:
    // Get the singleton instance
    static SingletonVisualizerManager& getInstance();

    // Initialize the viewer
    void initialize();
    void start();

    void AddToDisplay(
        const Object3D& robot,
        const Eigen::Vector3f& destination,
        const pcl::PointCloud<pcl::PointXYZ>& cloud,
        const std::vector<Eigen::Vector3f>& pathPoints,
        const std::vector<std::shared_ptr<Object3D>>& refinedCurrentDetectedObjec,
        const std::vector<SectorConeOfVision> &mapDetectedObject,
        const Eigen::Vector3f shifter = {0.0f, 0.0f, 0.0f});

        void addShifterToViewer(const Eigen::Vector3f& destination);
private:
    // Private constructor for singleton
    SingletonVisualizerManager() {}

    // No copying or assignment
    SingletonVisualizerManager(const SingletonVisualizerManager&) = delete;
    SingletonVisualizerManager& operator=(const SingletonVisualizerManager&) = delete;

    //pcl::visualization::PCLVisualizer::Ptr viewer; // PCL visualizer
    
    std::mutex _viewerMutex;
    std::atomic<bool> _isViewerReady;
    std::shared_ptr<OpenGLRenderer> _viewer;
    std::thread _viewerThread;
    
};
