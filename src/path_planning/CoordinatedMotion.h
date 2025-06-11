// CoordinatedMotion.h
#pragma once
#include "DiagonalMotionController.h"
#include "extruder/ServoSystem.h"

class CoordinatedMotion {
public:
    CoordinatedMotion(DiagonalMotionController& planner,
                      ServoSystem& extruderServo);
    
    void moveLinear(float x, float y, float z, float e, float feedRate);
    void update();
    DiagonalMotionController& _planner;
    ServoSystem& _extruderServo;
    float targetX;
    float targetY;
    float targetZ;
    float lastFeedRate = 200.0f; // Default feed rate
    int last_x_Dir = 0; // Last direction of X axis motor, 1 for forward, -1 for backward, 0 for stopped
    int last_y_Dir = 0; // Last direction of Y axis motor, 1 for forward, -1 for backward, 0 for stopped
    int last_z_Dir = 0; // Last direction of Z axis motor, 1 for forward, -1 for backward, 0 for stopped
    
    int current_x_Dir = 0; // Current direction of X axis motor
    int current_y_Dir = 0; // Current direction of Y axis motor
    int current_z_Dir = 0; // Current direction of Z axis motor
    int current_e_Dir = 0; // Current direction of extruder servo motor
private:
    
};

// Implementation would handle more complex motion planning if needed
