#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <Eigen/Dense>
#include <GL/freeglut.h>

#define OPENGLRENDERER_MAX_ZOOM_OUT 300.0f
#define OPENGLRENDERER_MIN_ZOOM_OUT 1.0f

#define OPENGLRENDERER_MOUSE_SENSITIVITY 0.1f
#define OPENGLRENDERER_CAM_SPEED 0.5f

#define OPENGLRENDERER_MAX_PITCH 89.0f

class OpenGLRenderer {
public:
    OpenGLRenderer();
    ~OpenGLRenderer();

    void updatePointCloud(const std::vector<Eigen::Vector3f>& points);
    void renderLoop();

    std::vector<Eigen::Vector3f> generateTestPointCloud();
private:
    GLFWwindow* _window;
    std::vector<glm::vec3> _points;

    // Camera settings
    glm::vec3 _cameraPos ={0,0,50};
    glm::vec3 _cameraFront={0,0,0};
    glm::vec3 _cameraUp={0,0,0};
    float _yaw = 0, _pitch = 0;
    float _lastX = 0, _lastY = 0;
    float _fov = 0;
    bool _firstMouse = 0;

    bool _mouseRotating;

    void processInput();
    void renderText(float x, float y, const std::string& text);

    static void mouseCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
};
