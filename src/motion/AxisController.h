#pragma once
#include "Encoder.h"
#include "MotorControl.h"
#include "CascadedPID.h"

// AxisController: controls one axis in mm and mm/s, using rotations as the internal unit
class AxisController {
public:
    AxisController(Encoder& encoder, MotorControl& motor,
                   float rotations_per_mm, int pulses_per_rev);

    void begin();
    void moveTo(float mm, float mm_per_sec);
    void update();
    void stop();
    float getCurrentMM();
    float getCurrentSpeedMM_S();
    void step(int direction); // +1 for forward, -1 for backward
    float getStepsPerMM() const;
    void setPositionGains(float kp, float ki, float kd);
    void setSpeedGains(float kp, float ki, float kd);
    float speed;
    
    CascadedPID _pid;

private:
    Encoder _encoder;
    MotorControl _motor;
    float _rotations_per_mm;
    int _pulses_per_rev;
    float _steps_per_mm;
};
