#include "CoordinatedMotion.h"


CoordinatedMotion::CoordinatedMotion(DiagonalMotionController& planner,
                                     ServoSystem& extruderServo)
    :_extruderServo(extruderServo),
     _planner(planner) {}

void CoordinatedMotion::moveLinear(float x, float y, float z, float e, float feedRate) {
    // Directly pass through to diagonal planner
    targetX = x;
    targetY = y;
    targetZ = z;
    float Es = e/_planner.time;
    _extruderServo.moveTo(e,Es); // Move extruder servo to the specified position with feed rate
    _planner.startMove(x, y, z,feedRate);
    lastFeedRate = feedRate; // Store the last feed rate used
}

void CoordinatedMotion::update() {
    // Propagate update calls to the planner
    _planner.update();
    _extruderServo.update();
}

