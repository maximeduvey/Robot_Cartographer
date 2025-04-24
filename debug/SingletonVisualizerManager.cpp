#include "SingletonVisualizerManager.h"
#include <exception>

SingletonVisualizerManager &SingletonVisualizerManager::getInstance()
{
    static SingletonVisualizerManager instance;
    return instance;
}

// Initialize the viewer
void SingletonVisualizerManager::initialize()
{
    std::lock_guard<std::mutex> lock(_viewerMutex);
    _isViewerReady.store(false);
    std::cout << TAG << std::endl;
    try {
        _viewer = std::make_shared<OpenGLRenderer>();
        _viewerThread = std::thread([&viewer = _viewer, &isViewerReady = _isViewerReady]() {
            try {
                viewer->startProcess(); 
            } catch(std::exception &e) {
                isViewerReady.store(false);
            }

        });
        _viewerThread.detach();
        _viewer.get()->doInit();
        _isViewerReady.store(true);
    }
    catch (std::runtime_error &e) {
        std::cout << "RE Could not initialize Opengl viewer, so it is disable!" <<std::endl;
        _viewer = nullptr;
    }
    catch (std::exception &e) {
        std::cout << "E Could not initialize Opengl viewer, so it is disable!" <<std::endl;
        _viewer = nullptr;
    }
}

void SingletonVisualizerManager::start()
{
    std::lock_guard<std::mutex> lock(_viewerMutex);
    std::cout << TAG << std::endl;
    if (_isViewerReady.load() == false) return;
    _viewer.get()->doRender();

}

void SingletonVisualizerManager::addShifterToViewer(const Eigen::Vector3f& destination)
{
    std::lock_guard<std::mutex> lock(_viewerMutex);
    if (_isViewerReady.load() == false) return;
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
    std::lock_guard<std::mutex> lock(_viewerMutex);
    std::cout << TAG << std::endl;
    if (_isViewerReady.load() == false) return;
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

