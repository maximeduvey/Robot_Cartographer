#include "OpenGLRenderer.h"
#include <iostream>

OpenGLRenderer::OpenGLRenderer()
    : _cameraPos(glm::vec3(0.0f, 0.0f, 3.0f)),
      _cameraFront(glm::vec3(0.0f, 0.0f, -1.0f)),
      _cameraUp(glm::vec3(0.0f, 1.0f, 0.0f)),
      _yaw(-90.0f), _pitch(0.0f),
      _lastX(400), _lastY(300),
      _fov(45.0f), _firstMouse(true),
      _mouseRotating(false)
{
    if (!glfwInit()) throw std::runtime_error("Failed to initialize GLFW");

    _window = glfwCreateWindow(800, 600, "Lidar Point Cloud", NULL, NULL);
    if (!_window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
    glfwMakeContextCurrent(_window);
    glewInit();

    int argc = 1;
    char* argv[1] = { (char*)"" };  // Dummy arguments for GLUT
    glutInit(&argc, argv);

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);

    // Capture the cursor
    glfwSetInputMode(_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Set callbacks
    glfwSetWindowUserPointer(_window, this);
    glfwSetCursorPosCallback(_window, mouseCallback);
    glfwSetScrollCallback(_window, scrollCallback);
    glfwSetMouseButtonCallback(_window, mouseButtonCallback);
}

OpenGLRenderer::~OpenGLRenderer() {
    glfwDestroyWindow(_window);
    glfwTerminate();
}

void OpenGLRenderer::updatePointCloud(const std::vector<Eigen::Vector3f>& points) {
    _points.clear();
    for (const auto& p : points) {
        _points.push_back(glm::vec3(p.x(), p.y(), p.z()));
    }
}

void OpenGLRenderer::renderLoop() {
    while (!glfwWindowShouldClose(_window)) {
        processInput();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glm::mat4 view = glm::lookAt(_cameraPos, _cameraPos + _cameraFront, _cameraUp);
        glMultMatrixf(&view[0][0]);

        // Render point cloud
        glBegin(GL_POINTS);
        glColor3f(1.0f, 1.0f, 1.0f);
        for (const auto& p : _points) {
            glVertex3f(p.x, p.y, p.z);
        }
        glEnd();

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        // Convert screen width to get text position
        int width, height;
        glfwGetFramebufferSize(_window, &width, &height);
        float aspectRatio = (float)width / (float)height;
        gluPerspective(_fov, aspectRatio, 0.1f, 100.0f);

        // Display Camera Info (Top-Right Corner)
        glColor3f(1.0, 1.0, 1.0);  // White text
        renderText((width/2), height - 30, "Camera Pos: (" +
                   std::to_string(_cameraPos.x) + ", " +
                   std::to_string(_cameraPos.y) + ", " +
                   std::to_string(_cameraPos.z) + ")");
        renderText((width/2), height - 50, "Rotation (Yaw/Pitch): " +
                   std::to_string(_yaw) + " / " + std::to_string(_pitch));

        glfwSwapBuffers(_window);
        glfwPollEvents();
    }
}


void OpenGLRenderer::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    OpenGLRenderer* renderer = static_cast<OpenGLRenderer*>(glfwGetWindowUserPointer(window));

    if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
        if (action == GLFW_PRESS) {
            renderer->_mouseRotating = true;
        } else if (action == GLFW_RELEASE) {
            renderer->_mouseRotating = false;
        }
    }
}

void OpenGLRenderer::processInput() {
    const float cameraSpeed = OPENGLRENDERER_CAM_SPEED;
    bool moved = false;

    if (glfwGetKey(_window, GLFW_KEY_W) == GLFW_PRESS) {
        _cameraPos += cameraSpeed * _cameraFront;
        moved = true;
    }
    if (glfwGetKey(_window, GLFW_KEY_S) == GLFW_PRESS) {
        _cameraPos -= cameraSpeed * _cameraFront;
        moved = true;
    }
    if (glfwGetKey(_window, GLFW_KEY_A) == GLFW_PRESS) {
        _cameraPos -= glm::normalize(glm::cross(_cameraFront, _cameraUp)) * cameraSpeed;
        moved = true;
    }
    if (glfwGetKey(_window, GLFW_KEY_D) == GLFW_PRESS) {
        _cameraPos += glm::normalize(glm::cross(_cameraFront, _cameraUp)) * cameraSpeed;
        moved = true;
    }

    if (moved) {
        std::cout << "[Camera] Position: (" 
                  << _cameraPos.x << ", " 
                  << _cameraPos.y << ", " 
                  << _cameraPos.z << ")\n";
    }
}


void OpenGLRenderer::mouseCallback(GLFWwindow* window, double xpos, double ypos) {
    OpenGLRenderer* renderer = static_cast<OpenGLRenderer*>(glfwGetWindowUserPointer(window));

    if (!renderer->_mouseRotating) return; // Only rotate when middle mouse is pressed

    if (renderer->_firstMouse) {
        renderer->_lastX = xpos;
        renderer->_lastY = ypos;
        renderer->_firstMouse = false;
    }

    float xOffset = xpos - renderer->_lastX;
    float yOffset = renderer->_lastY - ypos; // Reversed y-axis
    renderer->_lastX = xpos;
    renderer->_lastY = ypos;

    const float sensitivity = OPENGLRENDERER_MOUSE_SENSITIVITY;
    xOffset *= sensitivity;
    yOffset *= sensitivity;

    renderer->_yaw += xOffset;
    renderer->_pitch += yOffset;

    if (renderer->_pitch > OPENGLRENDERER_MAX_PITCH) renderer->_pitch = OPENGLRENDERER_MAX_PITCH;
    if (renderer->_pitch < -OPENGLRENDERER_MAX_PITCH) renderer->_pitch = -OPENGLRENDERER_MAX_PITCH;

    glm::vec3 front;
    front.x = cos(glm::radians(renderer->_yaw)) * cos(glm::radians(renderer->_pitch));
    front.y = sin(glm::radians(renderer->_pitch));
    front.z = sin(glm::radians(renderer->_yaw)) * cos(glm::radians(renderer->_pitch));
    renderer->_cameraFront = glm::normalize(front);

    std::cout << "[Camera] Rotating: Yaw=" << renderer->_yaw << " | Pitch=" << renderer->_pitch << std::endl;
}


void OpenGLRenderer::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    OpenGLRenderer* renderer = static_cast<OpenGLRenderer*>(glfwGetWindowUserPointer(window));
    renderer->_fov -= (float)yoffset;
    if (renderer->_fov < OPENGLRENDERER_MIN_ZOOM_OUT) renderer->_fov = OPENGLRENDERER_MIN_ZOOM_OUT;
    if (renderer->_fov > OPENGLRENDERER_MAX_ZOOM_OUT) renderer->_fov = OPENGLRENDERER_MAX_ZOOM_OUT;

    std::cout << "[Camera] FOV (Zoom): " << renderer->_fov << std::endl;
}

std::vector<Eigen::Vector3f> OpenGLRenderer::generateTestPointCloud() {
    std::vector<Eigen::Vector3f> points;

    float size = 5.0f;
    float step = 0.5f;

    for (float x = -size; x <= size; x += step) {
        for (float y = -size; y <= size; y += step) {
            for (float z = -size; z <= size; z += step) {
                points.emplace_back(Eigen::Vector3f(x, y, z));
            }
        }
    }

    return points;
}

void OpenGLRenderer::renderText(float x, float y, const std::string& text) {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    int width, height;
    glfwGetFramebufferSize(_window, &width, &height);
    gluOrtho2D(0, width, 0, height);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glRasterPos2f(x, y);

    for (char c : text) {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, c);
    }

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}
