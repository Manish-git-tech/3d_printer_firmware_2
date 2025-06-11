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

private:
    
};

// Implementation would handle more complex motion planning if needed
