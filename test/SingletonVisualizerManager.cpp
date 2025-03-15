#include "SingletonVisualizerManager.h"

SingletonVisualizerManager &SingletonVisualizerManager::getInstance()
{
    static SingletonVisualizerManager instance;
    return instance;
}

// Initialize the viewer
void SingletonVisualizerManager::initialize()
{
    std::lock_guard<std::mutex> lock(viewerMutex);
    std::cout << TAG << std::endl;
    _viewer = std::make_shared<OpenGLRenderer>();
    _viewerThread = std::thread([viewer = _viewer]() {
        viewer->startProcess(); 
    });
    _viewerThread.detach();
    _viewer.get()->doInit();
}

void SingletonVisualizerManager::start()
{
    std::lock_guard<std::mutex> lock(viewerMutex);
    std::cout << TAG << std::endl;
    
    _viewer.get()->doRender();

}

void SingletonVisualizerManager::addShifterToViewer(const Eigen::Vector3f& destination)
{
    std::lock_guard<std::mutex> lock(viewerMutex);
    _viewer.get()->addShifterToViewer(destination);
}

void SingletonVisualizerManager::AddToDisplay(
    const Object3D& robot,
    const Eigen::Vector3f& destination,
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::vector<Eigen::Vector3f>& pathPoints,
    const std::vector<std::shared_ptr<Object3D>>& refinedCurrentDetectedObjec,
    const std::vector<SectorConeOfVision> &mapDetectedObject,
    const Eigen::Vector3f shifter /* = {0.0f, 0.0f, 0.0f} */)
{
    std::lock_guard<std::mutex> lock(viewerMutex);
    std::cout << TAG << std::endl;
    _viewer.get()->clear();
    const auto tmp = (destination + shifter);
    _viewer.get()->addObjectToView((robot + shifter), RGB_ORANGE);
    _viewer.get()->addPointToView(shifter, RGB_YELLOW);
    _viewer.get()->addPointToView(tmp, RGB_YELLOW);
    _viewer.get()->addPointsToView(cloud, RGB_WHITE, shifter);
    _viewer.get()->addPointsToView(pathPoints, RGB_GREEN, shifter);
    //_viewer.get()->addPointsToView(refinedCurrentDetectedObjec, RGB_PURPLE);
    for (const auto &sector : mapDetectedObject) {
        for (const std::shared_ptr<Object3D> &obj : sector._detectedObjects) {
        _viewer.get()->addObjectToView((*obj.get() + shifter), RGB_RED);
        }
    }
}

