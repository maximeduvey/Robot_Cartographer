#pragma once
#include <glm/glm.hpp>

#define CAMERASTATE_PRESET_POS 0.0f,0.0f,50.0f
#define CAMERASTATE_PRESET_FRONT 0.0f,0.0f,-1.0f
#define CAMERASTATE_PRESET_UP 0.0f,1.0f,0.0f
#define CAMERASTATE_PRESET_FOV 75.0f
#define CAMERASTATE_PRESET_YAW -90.0f

class CameraState {
public:
    glm::vec3 position;
    glm::vec3 front;
    glm::vec3 up;
    float yaw;
    float pitch;
    float fov;
    float lastX = 0, lastY = 0;

    // Constructor with defaults
    CameraState()
        : position(glm::vec3(CAMERASTATE_PRESET_POS)),
          front(glm::vec3(CAMERASTATE_PRESET_FRONT)),
          up(glm::vec3(CAMERASTATE_PRESET_UP)),
          yaw(CAMERASTATE_PRESET_YAW), pitch(0.0f), fov(CAMERASTATE_PRESET_FOV),
          lastX(400.0f), lastY(300.0f)
          {
          }

    // Reset camera to initial values
    void reset() {
        position = glm::vec3(CAMERASTATE_PRESET_POS);
        yaw = CAMERASTATE_PRESET_YAW;
        up = glm::vec3(CAMERASTATE_PRESET_UP);
        pitch = 0.0f;
        fov = CAMERASTATE_PRESET_FOV;
        front = glm::vec3(
            cos(glm::radians(yaw)) * cos(glm::radians(pitch)),
            sin(glm::radians(pitch)),
            sin(glm::radians(yaw)) * cos(glm::radians(pitch))
        );
        lastX = 0;
        lastY = 0;
    }
    void set(const CameraState& state) {
        position = state.position;
        front = state.front;
        up = state.up;
        yaw = state.yaw;
        pitch = state.pitch;
        fov = state.fov;
        lastX = state.lastX;
        lastY = state.lastY;
    }
    
    CameraState& operator=(const CameraState& state) {
        if (this != &state) {  // Prevent self-assignment
            set(state);
        }
        return *this;
    }
};
