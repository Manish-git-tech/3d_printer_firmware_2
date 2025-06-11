#pragma once
#include "motion/AxisController.h"

class DiagonalMotionController {
public:
    float step_resolution = 0.1; // segments per mm

    DiagonalMotionController(AxisController& x, AxisController& y, AxisController& z);

    void startMove(float target_x, float target_y, float target_z, float speed_mm_s);
    void update(); // Call this frequently from your main loop
    bool isMoving() const;
    float speedx;
    float speedy;
    float speedz;
    float time;

    AxisController& xAxis;
    AxisController& yAxis;
    AxisController& zAxis;

private:

    float start_x, start_y, start_z;
    float end_x, end_y, end_z;
    float speed;
    int total_segments;
    int current_segment;
    bool moving;

    float dx, dy, dz;
    float step_x, step_y, step_z;
};
