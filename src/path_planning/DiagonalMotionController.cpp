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

    step_x = dx / total_segments;
    step_y = dy / total_segments;
    step_z = dz / total_segments;

    time = distance / speed;

    speedx = speed / distance * dx; 
    speedy = speed / distance * dy;
    speedz = speed / distance * dz;
    moving = true;

    // Command the first segment
    float next_x = start_x + step_x;
    float next_y = start_y + step_y;
    float next_z = start_z + step_z;
    xAxis.moveTo(next_x, speedx);
    yAxis.moveTo(next_y, speedy);
    zAxis.moveTo(next_z, speedz);
    
}

void DiagonalMotionController::update() {
    // Call update on each axis to keep their PID loops running
    xAxis.update();
    yAxis.update();
    zAxis.update();

    if (!moving) return;

    // Check if all axes have reached their current segment target
    float expected_x = start_x + step_x * (current_segment + 1);
    float expected_y = start_y + step_y * (current_segment + 1);
    float expected_z = start_z + step_z * (current_segment + 1);

    float tol = 0.01f; // mm tolerance

    if (fabs(xAxis.getCurrentMM() - expected_x) < tol &&
        fabs(yAxis.getCurrentMM() - expected_y) < tol &&
        fabs(zAxis.getCurrentMM() - expected_z) < tol) {

        current_segment++;
        if (current_segment >= total_segments) {
            moving = false;
            Serial.println("Ok");
            return;
        }

        // Command the next segment
        float next_x = start_x + step_x * (current_segment + 1);
        float next_y = start_y + step_y * (current_segment + 1);
        float next_z = start_z + step_z * (current_segment + 1);
        xAxis.moveTo(next_x, speedx);
        yAxis.moveTo(next_y, speedy);
        zAxis.moveTo(next_z, speedz);
    }
}


bool DiagonalMotionController::isMoving() const {
    return moving;
}
