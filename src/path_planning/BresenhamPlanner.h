// BresenhamPlanner.h
#pragma once
#include <Arduino.h>
#include "motion/AxisController.h" // Your existing axis control class
#include "extruder/ServoSystem.h"    // Your modified extruder class

class BresenhamPlanner {
public:
    BresenhamPlanner(AxisController& x, AxisController& y, AxisController& z, ServoSystem& e);
    
    void moveTo(float x, float y, float z, float e, float feedRate);
    bool isMoving();
    void update();
    void stop();
    float last_feedRate;
    int _xDir, _yDir, _zDir, _eDir; // Direction for each axis: 1 for forward, -1 for backward
private:
    AxisController& _x;
    AxisController& _y;
    AxisController& _z;
    ServoSystem& _e;

    long _xSteps, _ySteps, _zSteps, _eSteps;
    long _xStepCount, _yStepCount, _zStepCount, _eStepCount;
    long _accErrorX, _accErrorY, _accErrorZ, _accErrorE;
    long _maxSteps;
    float _stepInterval;
    unsigned long _lastStepTime;
    bool _moving;
    
    void _calculateSteps(float dx, float dy, float dz, float de, float feedRate);
};
