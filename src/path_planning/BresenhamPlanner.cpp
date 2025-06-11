// BresenhamPlanner.cpp
#include "BresenhamPlanner.h"

BresenhamPlanner::BresenhamPlanner(AxisController& x, AxisController& y, AxisController& z, ServoSystem& e)
    : _x(x), _y(y), _z(z), _e(e), _moving(false) {}

void BresenhamPlanner::moveTo(float x, float y, float z, float e, float feedRate) {
    stop();
    
    float dx = x - _x.getCurrentMM();
    float dy = y - _y.getCurrentMM();
    float dz = z - _z.getCurrentMM();
    float de = e - _e.getCurrentMM();
    
    _calculateSteps(fabs(dx), fabs(dy), fabs(dz), fabs(de), feedRate);
    
    // Determine direction for each axis (+1 or -1)
    _xDir = (dx >= 0) ? 1 : -1;
    _yDir = (dy >= 0) ? 1 : -1;
    _zDir = (dz >= 0) ? 1 : -1;
    _eDir = (de >= 0) ? 1 : -1;

    _lastStepTime = micros();
    _moving = true;
    last_feedRate = feedRate;
}

void BresenhamPlanner::_calculateSteps(float dx, float dy, float dz, float de, float feedRate) {
    // Convert mm to steps for each axis (assuming steps/mm configured in controllers)
    _xSteps = dx * _x.getStepsPerMM();
    _ySteps = dy * _y.getStepsPerMM();
    _zSteps = dz * _z.getStepsPerMM();
    _eSteps = de * _e.getStepsPerMM(); // Assuming extruder has steps/mm equivalent
    
    _maxSteps = max(max(_xSteps, _ySteps), max(_zSteps, _eSteps));
    
    // Calculate step interval in microseconds
    float longestDistance = max(max(dx, dy), max(dz, de));
    float totalTimeS = longestDistance / feedRate;
    _stepInterval = (totalTimeS * 1e6) / _maxSteps;
    
    // Reset counters
    _xStepCount = _yStepCount = _zStepCount = _eStepCount = 0;
    _accErrorX = _accErrorY = _accErrorZ = _accErrorE = _maxSteps / 2;
}

void BresenhamPlanner::update() {
    if(!_moving) return;
    Serial.print(" bresenham update: ");
    unsigned long now = micros();
    if(now - _lastStepTime < _stepInterval) return;

    // Bresenham algorithm for each axis
    if(_xStepCount < _xSteps) {
        _accErrorX -= _maxSteps;
        if(_accErrorX < 0) {
            _x.step(_xDir);
            Serial.print("X step: ");
            _accErrorX += _xSteps;
            _xStepCount++;
        }
    }
    if(_yStepCount < _ySteps) {
        _accErrorY -= _maxSteps;
        if(_accErrorY < 0) {
            _y.step(_yDir);
            Serial.print("Y step: ");
            _accErrorY += _ySteps;
            _yStepCount++;
        }
    }
    if(_zStepCount < _zSteps) {
        _accErrorZ -= _maxSteps;
        if(_accErrorZ < 0) {
            _z.step(_zDir);
            Serial.print("Z step: ");
            _accErrorZ += _zSteps;
            _zStepCount++;
        }
    }
    if(_eStepCount < _eSteps) {
        _accErrorE -= _maxSteps;
        if(_accErrorE < 0) {
            _e.step(_eDir);
            Serial.print("E step: ");
            _accErrorE += _eSteps;
            _eStepCount++;
        }
    }

    _lastStepTime = now;
    
    // Check if all axes completed
    if(_xStepCount >= _xSteps && 
       _yStepCount >= _ySteps && 
       _zStepCount >= _zSteps && 
       _eStepCount >= _eSteps) {
        stop();
    }
}

bool BresenhamPlanner::isMoving() { return _moving; }

void BresenhamPlanner::stop() {
    _moving = false;
    _x.stop();
    _y.stop();
    _z.stop();
    _e.stop();
}
