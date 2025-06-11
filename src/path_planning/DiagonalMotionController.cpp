#include "DiagonalMotionController.h"
#include <cmath>

DiagonalMotionController::DiagonalMotionController(AxisController& x, AxisController& y, AxisController& z)
    : xAxis(x), yAxis(y), zAxis(z), moving(false) {}

void DiagonalMotionController::startMove(float target_x, float target_y, float target_z, float speed_mm_s) {
    start_x = xAxis.getCurrentMM();
    start_y = yAxis.getCurrentMM();
    start_z = zAxis.getCurrentMM();

    end_x = target_x;
    end_y = target_y;
    end_z = target_z;
    speed = speed_mm_s;

    dx = end_x - start_x;
    dy = end_y - start_y;
    dz = end_z - start_z;

    float distance = sqrt(dx*dx + dy*dy + dz*dz);
    total_segments = std::max(1, int(distance * step_resolution));
    current_segment = 0;

    time = distance / speed;

    speedx = speed / distance * dx; 
    speedy = speed / distance * dy;
    speedz = speed / distance * dz;
    moving = true;

    xAxis.moveTo(end_x,speedx);
    yAxis.moveTo(end_y,speedy);
    zAxis.moveTo(end_z,speedz);
}

void DiagonalMotionController::update() {
    // Call update on each axis to keep their PID loops running
    xAxis.update();
    yAxis.update();
    zAxis.update();
}
bool DiagonalMotionController::isMoving() const {
    return moving;
}
