#include "OpenGLRenderer.h"
#include <iostream>

/**
 * WARNING , It must be the SAME THREAD  that do everything
 * creation of _windows then updating it, otherwise it will not be updated
 */

OpenGLRenderer::OpenGLRenderer()
    : _firstMouse(true),
      _mouseRotating(false)
{

}

OpenGLRenderer::~OpenGLRenderer() {
    glfwDestroyWindow(_window);
    glfwTerminate();
}

void OpenGLRenderer::startProcess()
{
    while (_state.load() < OPENGLRENDERER_STATE::STOPING)
    {
        if (_state.load() == OPENGLRENDERER_STATE::INITIALIZING)
        {
            init();
        } else if (_state.load() == OPENGLRENDERER_STATE::READY_TO_RENDER)
        {
            renderLoop();
        }
    }
}

void OpenGLRenderer::doInit()
{
    _state.store(OPENGLRENDERER_STATE::INITIALIZING);
}

void OpenGLRenderer::doRender()
{
    if (_state.load() != OPENGLRENDERER_STATE::INITIALIZED)
        throw std::runtime_error("doRender cannot be done in state other than INITIALIZED");
        _state.store(OPENGLRENDERER_STATE::READY_TO_RENDER);
}

void OpenGLRenderer::stop()
{
    _state.store(OPENGLRENDERER_STATE::STOPING);
}

void OpenGLRenderer::init()
{
    if (!glfwInit()) throw std::runtime_error("Failed to initialize GLFW !");

    _doClearView.store(false);

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

    _state.store(OPENGLRENDERER_STATE::INITIALIZED);
}

void OpenGLRenderer::renderLoop()
{
    std::cout << TAG << std::endl;
    while (!glfwWindowShouldClose(_window))
    {
        processInput();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glm::mat4 view = glm::lookAt(_camera.position,_camera.position + _camera.front, _camera.up);
        glMultMatrixf(&view[0][0]);

        // updating points
        _mutexVectorPoints.lock();
        glBegin(GL_POINTS);
        //std::cout << TAG << "_points:" << _points.size() << std::endl;
        for (const auto& p : _points)
        {
            glColor3f(p.color.x(), p.color.y(), p.color.z());
            glVertex3f(p.pos.x(), p.pos.y(), p.pos.z());
        }
        _mutexVectorPoints.unlock();
        glEnd();

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        // Convert screen width to get text position
        int width, height;
        glfwGetFramebufferSize(_window, &width, &height);
        float aspectRatio = (float)width / (float)height;
        gluPerspective(_camera.fov, aspectRatio, 0.1f, 100.0f);

        // Display Camera Info (Top-Right Corner)
        renderText(0, height - 30,
            "Camera Pos: (" + std::to_string(_camera.position.x) + ", " + std::to_string(_camera.position.y) + ", " + std::to_string(_camera.position.z) + ")",
            RGB_WHITE);
        renderText(0, height - 50,
            "Rotation (Yaw/Pitch): " + std::to_string(_camera.yaw) + " / " + std::to_string(_camera.pitch),
            RGB_WHITE);
            renderText(0, height - 70,
                "deepness (fov): " + std::to_string(_camera.fov),
                RGB_WHITE);

        glfwSwapBuffers(_window);
        glfwPollEvents();
    }
    std::cout << TAG << " !!!! QUITTING !!!" << std::endl;
    _state.store(OPENGLRENDERER_STATE::STOPED);
}

// #
// # CALLBACK FOR INPUT MANAGEMENT
// #Mouse, keyboard, etc

void OpenGLRenderer::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    OpenGLRenderer* renderer = static_cast<OpenGLRenderer*>(glfwGetWindowUserPointer(window));

    if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
        if (action == GLFW_PRESS) {
            renderer->_mouseRotating = true;
        }
        else if (action == GLFW_RELEASE) {
            renderer->_mouseRotating = false;
        }
    }
}

void OpenGLRenderer::processInput() {
    const float cameraSpeed = OPENGLRENDERER_CAM_SPEED;
    bool moved = false;

    // Get the right vector for left/right movement
    glm::vec3 right = glm::normalize(glm::cross(_camera.front, _camera.up));

    if (glfwGetKey(_window, GLFW_KEY_W) == GLFW_PRESS) {
        _camera.position += cameraSpeed * _camera.up;  // ✅ Move UP along world Y-axis
        moved = true;
    }
    if (glfwGetKey(_window, GLFW_KEY_S) == GLFW_PRESS) {
        _camera.position -= cameraSpeed * _camera.up;  // ✅ Move DOWN along world Y-axis
        moved = true;
    }
    if (glfwGetKey(_window, GLFW_KEY_A) == GLFW_PRESS) {
        _camera.position -= cameraSpeed * right;  // ✅ Move LEFT in the XZ plane
        moved = true;
    }
    if (glfwGetKey(_window, GLFW_KEY_D) == GLFW_PRESS) {
        _camera.position += cameraSpeed * right;  // ✅ Move RIGHT in the XZ plane
        moved = true;
    }
    if (glfwGetKey(_window, GLFW_KEY_R) == GLFW_PRESS) {  // ✅ Reset camera to initial state
        _camera.reset();
        moved = true;
    }

    if (moved) {
        std::cout << "[Camera] Position: ("
            <<_camera.position.x << ", "
            <<_camera.position.y << ", "
            <<_camera.position.z << ")\n";
    }
}




void OpenGLRenderer::mouseCallback(GLFWwindow* window, double xpos, double ypos) {
    OpenGLRenderer* renderer = static_cast<OpenGLRenderer*>(glfwGetWindowUserPointer(window));

    if (!renderer->_mouseRotating) return; // Only rotate when middle mouse is pressed

    if (renderer->_firstMouse) {
        renderer->_camera.lastX = xpos;
        renderer->_camera.lastY = ypos;
        renderer->_firstMouse = false;
    }

    float xOffset = xpos - renderer->_camera.lastX;
    float yOffset = renderer->_camera.lastY - ypos; // Reversed y-axis
    renderer->_camera.lastX = xpos;
    renderer->_camera.lastY = ypos;

    const float sensitivity = OPENGLRENDERER_MOUSE_SENSITIVITY;
    xOffset *= sensitivity;
    yOffset *= sensitivity;

    renderer->_camera.yaw += xOffset;
    renderer->_camera.pitch += yOffset;

    if (renderer->_camera.pitch > OPENGLRENDERER_MAX_PITCH) renderer->_camera.pitch = OPENGLRENDERER_MAX_PITCH;
    if (renderer->_camera.pitch < -OPENGLRENDERER_MAX_PITCH) renderer->_camera.pitch = -OPENGLRENDERER_MAX_PITCH;

    glm::vec3 front;
    front.x = cos(glm::radians(renderer->_camera.yaw)) * cos(glm::radians(renderer->_camera.pitch));
    front.y = sin(glm::radians(renderer->_camera.pitch));
    front.z = sin(glm::radians(renderer->_camera.yaw)) * cos(glm::radians(renderer->_camera.pitch));
    renderer->_camera.front = glm::normalize(front);

    std::cout << "[Camera] Rotating: Yaw=" << renderer->_camera.yaw << " | Pitch=" << renderer->_camera.pitch << std::endl;
}


void OpenGLRenderer::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    OpenGLRenderer* renderer = static_cast<OpenGLRenderer*>(glfwGetWindowUserPointer(window));
    renderer->_camera.fov -= (float)yoffset;
    if (renderer->_camera.fov < OPENGLRENDERER_MIN_ZOOM_OUT) renderer->_camera.fov = OPENGLRENDERER_MIN_ZOOM_OUT;
    if (renderer->_camera.fov > OPENGLRENDERER_MAX_ZOOM_OUT) renderer->_camera.fov = OPENGLRENDERER_MAX_ZOOM_OUT;

    std::cout << "[Camera] FOV (Zoom): " << renderer->_camera.fov << std::endl;
}


void OpenGLRenderer::addShifterToViewer(const Eigen::Vector3f& shifer)
{
    _camera.addShifter(shifer.x(),shifer.y(),shifer.z());
}

// #
// #
// #

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

void OpenGLRenderer::renderText(float x, float y, const std::string& text, float r/*=1.0f*/, float g/*=1.0f*/, float b/*=1.0f*/) {
    glColor3f(r,g,b);
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


// #
// # classic other class to points
// #

void OpenGLRenderer::addPointsToView(const pcl::PointCloud<pcl::PointXYZ>& cloud, float r, float g, float b, const Eigen::Vector3f shifter /* = {0.0f, 0.0f, 0.0f} */) {
    std::lock_guard<std::mutex> lock(_mutexVectorPoints);

    for (const auto& p : cloud.points) {
        RGBPoint point;
        point.pos = Eigen::Vector3f(p.x, p.y, p.z) + shifter;
        point.distance = static_cast<uint16_t>(p.getVector3fMap().norm());
        point.color = Eigen::Vector3f(r, g, b);
        _points.push_back(point);
    }
}

void OpenGLRenderer::addPointToView(const RGBPoint& point)
{
    std::lock_guard<std::mutex> lock(_mutexVectorPoints);
    _points.push_back(point);
}

void OpenGLRenderer::addPointsToView(const std::vector<RGBPoint>& points, const Eigen::Vector3f shifter /* = {0.0f, 0.0f, 0.0f} */)
{
    std::lock_guard<std::mutex> lock(_mutexVectorPoints);
    for (auto point : points)
    {
        point.pos += shifter;
        _points.push_back(point);
    }
}

void OpenGLRenderer::addPointToView(const Eigen::Vector3f& point, float r, float g, float b)
{
    std::lock_guard<std::mutex> lock(_mutexVectorPoints);
    _points.push_back(RGBPoint(point, Eigen::Vector3f(r, g, b)));
}


void OpenGLRenderer::addPointsToView(const std::vector<Point>& points, float r, float g, float b, const Eigen::Vector3f shifter /* = {0.0f, 0.0f, 0.0f} */)
{
    std::lock_guard<std::mutex> lock(_mutexVectorPoints);

    std::transform(points.begin(), points.end(), std::back_inserter(_points),
        [&](const Point& p) {
            return RGBPoint{ p.pos+shifter, Eigen::Vector3f(r, g, b)};
        });
}

void OpenGLRenderer::addPointsToView(const std::vector<Eigen::Vector3f>& pathPoints, float r, float g, float b, const Eigen::Vector3f shifter /* = {0.0f, 0.0f, 0.0f} */)
{
    std::lock_guard<std::mutex> lock(_mutexVectorPoints);

    for (const auto& p : pathPoints) {
        RGBPoint point;
        point.pos = p+shifter;
        point.color = Eigen::Vector3f(r, g, b);
        _points.push_back(point);
    }
}

void OpenGLRenderer::addObjectToView(const Object3D& obj, float r, float g, float b) {
    std::lock_guard<std::mutex> lock(_mutexVectorPoints);

    const auto corners = obj.getBoundingBoxCorners();
    for (int i = 0; i < corners.size(); i++) {
        RGBPoint point;
        point.pos = corners[i];
        point.color = Eigen::Vector3f(r, g, b);
        _points.push_back(point);
    }
}

void OpenGLRenderer::addObjectsToView(const std::vector<Object3D>& objs, const Eigen::Vector3f shifter, float r, float g, float b) {
    std::lock_guard<std::mutex> lock(_mutexVectorPoints);

    for (const auto obj : objs)
    {
        const auto corners = obj.getBoundingBoxCorners(shifter);
        for (int i = 0; i < corners.size(); i++) {
            RGBPoint point;
            point.pos = corners[i] + shifter;
            point.color = Eigen::Vector3f(r, g, b);
            _points.push_back(point);
        }
    }
}

void OpenGLRenderer::clear()
{
    std::lock_guard<std::mutex> lock(_mutexVectorPoints);
    _points.clear();
}

// #
// #
// #
