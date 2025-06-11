// CascadedPID.h
#pragma once

#include "Encoder.h"
#include "MotorControl.h"

// Cascaded PID controller for position + speed
class CascadedPID {
public:
    CascadedPID(Encoder* enc, MotorControl* motor, int pulsesPerRev);

    void setPositionGains(float kp, float ki, float kd);
    void setSpeedGains(float kp, float ki, float kd);

    void moveTo(float rotations, float maxSpeed);
    void stop();
    void home();

    void update(); // Call regularly in loop

    float getTarget();
    float getCurrent();
    float getSpeed();
    bool _active;
    float _maxSpeed; // Max speed in rotations/sec

private:
    Encoder* _enc;
    MotorControl* _motor;
    int _pulsesPerRev;

    // Position PID
    float _kp_pos, _ki_pos, _kd_pos;
    float _integral_pos, _prevError_pos;

    // Speed PID
    float _kp_spd, _ki_spd, _kd_spd;
    float _integral_spd, _prevError_spd;

    float _targetRot;
    
    unsigned long _lastTime;
};
