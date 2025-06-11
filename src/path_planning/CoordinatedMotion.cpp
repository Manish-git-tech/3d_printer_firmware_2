#include "CoordinatedMotion.h"
#include "config/motion_params.h"


CoordinatedMotion::CoordinatedMotion(DiagonalMotionController& planner,
                                     ServoSystem& extruderServo)
    :_extruderServo(extruderServo),
     _planner(planner) {}

void CoordinatedMotion::moveLinear(float x, float y, float z, float e, float feedRate) {
    // Directly pass through to diagonal planner
    targetX = x;
    targetY = y;
    targetZ = z;
    last_x_Dir = _planner.xAxis._motor.last_direction; // Store last direction of X motor
    last_y_Dir = _planner.yAxis._motor.last_direction; // Store last direction of Y motor
    last_z_Dir = _planner.zAxis._motor.last_direction; // Store last direction of Z motor
    
    current_x_Dir = (x > _planner.xAxis.getCurrentMM()) ? 1 : (x < _planner.xAxis.getCurrentMM() ? -1 : 0);
    current_y_Dir = (y > _planner.yAxis.getCurrentMM()) ? 1 : (y < _planner.yAxis.getCurrentMM() ? -1 : 0);
    current_z_Dir = (z > _planner.zAxis.getCurrentMM()) ? 1 : (z < _planner.zAxis.getCurrentMM() ? -1 : 0);
    current_e_Dir = (e > _extruderServo.getCurrentMM()) ? 1 : (e < _extruderServo.getCurrentMM() ? -1 : 0);

    if (current_x_Dir < 0) {
        x = x - X_BACKLASH_MM; // Add backlash compensation for X axis
    }
    if (current_y_Dir < 0) {
        y = y - Y_BACKLASH_MM; // Add backlash compensation for Y axis
    }
    if (current_z_Dir < 0) {
        z = z - Z_BACKLASH_MM; // Add backlash compensation for Z axis
    }
    if (current_e_Dir < 0) {
        e = e - EXTRUDER_BACKLASH_MM; // Add backlash compensation for extruder
    }

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

