#pragma once

#include <CommonSpaceRepresentation.h>
#include "CameraState.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <Eigen/Dense>
#include <GL/freeglut.h>

#include <mutex>
#include <atomic>

#define OPENGLRENDERER_MAX_ZOOM_OUT 300.0f
#define OPENGLRENDERER_MIN_ZOOM_OUT 1.0f

#define OPENGLRENDERER_MOUSE_SENSITIVITY 0.1f
#define OPENGLRENDERER_CAM_SPEED 0.5f

#define OPENGLRENDERER_MAX_PITCH 89.0f


class OpenGLRenderer {
public:
    enum OPENGLRENDERER_STATE : int 
    {
        UNKNOW = 0,
        INITIALIZING,
        INITIALIZED,
        READY_TO_RENDER,
        RENDERING,
        STOPING,
        STOPED,
    };

    OpenGLRenderer();
    ~OpenGLRenderer();

    /* function to call to start everything then call state change function */
    void startProcess();
    void doInit();
    void doRender();
    void stop();


    void addPointToView(const RGBPoint& point);
    void addPointToView(const Eigen::Vector3f& point, float r, float g, float b);
    void addPointsToView(const std::vector<RGBPoint>& points);
    void addPointsToView(const pcl::PointCloud<pcl::PointXYZ>& cloud, float r=1.0f, float g=1.0f, float b=1.0f);
    void addPointsToView(const std::vector<Eigen::Vector3f>& pathPoints, float r=1.0f, float g=1.0f, float b=1.0f);
    void addPointsToView(const std::vector<Point>& points, float r=1.0f, float g=1.0f, float b=1.0f);
    void addPointsToView(const Point &point, float r=1.0f, float g=1.0f, float b=1.0f);
    void addObjectToView(const Object3D& obj, float r=1.0f, float g=1.0f, float b=1.0f);
    void addObjectsToView(const std::vector<Object3D>& objs, const Eigen::Vector3f shifter, float r=1.0f, float g=1.0f, float b=1.0f);
    void clear();


    std::vector<Eigen::Vector3f> generateTestPointCloud();
private:
    GLFWwindow* _window;
    std::atomic<bool> _end = {false};
    std::atomic<OPENGLRENDERER_STATE> _state;

    std::mutex _mutexVectorPoints;
    std::vector<RGBPoint> _points;

    CameraState _camera;
    // Camera settings
/*     glm::vec3 _cameraPos ={0,0,50};
    glm::vec3 _cameraFront={0,0,0};
    glm::vec3 _cameraUp={0,0,0};
    float _yaw = 0, _pitch = 0;
    float _lastX = 0, _lastY = 0;
    float _fov = 175.0f; */
    bool _firstMouse = 0;

    bool _mouseRotating;

    //
    std::atomic<bool> _doClearView;

    void init();
    void renderLoop();

    void processInput();
    void renderText(float x, float y, const std::string& text, float r=1.0f, float g=1.0f, float b=1.0f);

    static void mouseCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
};


