// CascadedPID.cpp
#include "CascadedPID.h"

CascadedPID::CascadedPID(Encoder* enc, MotorControl* motor, int pulsesPerRev)
    : _enc(enc), _motor(motor), _pulsesPerRev(pulsesPerRev),
      _kp_pos(10), _ki_pos(0.5), _kd_pos(20),
      _kp_spd(100), _ki_spd(10), _kd_spd(0),
      _integral_pos(0), _prevError_pos(0),
      _integral_spd(0), _prevError_spd(0),
      _targetRot(0), _maxSpeed(1), _active(false), _lastTime(0)
{}

void CascadedPID::setPositionGains(float kp, float ki, float kd) {
    _kp_pos = kp; _ki_pos = ki; _kd_pos = kd;
}
void CascadedPID::setSpeedGains(float kp, float ki, float kd) {
    _kp_spd = kp; _ki_spd = ki; _kd_spd = kd;
}

void CascadedPID::moveTo(float rotations, float maxSpeed) {
    _targetRot = rotations;
    _maxSpeed = abs(maxSpeed);
    _integral_pos = _integral_spd = 0;
    _prevError_pos = _prevError_spd = 0;
    _active = true;
    _lastTime = millis();
    Serial.print("CascadedPID: moveTo targetRot=");
    Serial.print(_targetRot);
    _start_time = millis();
    _start_pos = getCurrent();
}

void CascadedPID::stop() {
    _active = false;
    _motor->stop();
}
void CascadedPID::home() {
    _enc->reset();
    _targetRot = 0;
    stop();
}

float CascadedPID::getTarget() { return _targetRot; }
float CascadedPID::getCurrent() { return _enc->getRotations(); }
float CascadedPID::getSpeed() { 
    unsigned long now1 = millis();
    float ddt = now1 - _start_time;
    return (getCurrent()- _start_pos)*1000/ddt;
}

void CascadedPID::update() {
    if (!_active) return;
    unsigned long now = millis();
    float dt = (now - _lastTime) / 1000.0;
    if (dt < 0.02) return; // 50 Hz
    _lastTime = now;

    _enc->updateSpeed();
    float currentRot = _enc->getRotations();
    float currentSpd = getSpeed();

    // --- Outer loop: Position PID ---
    float posError = _targetRot - currentRot;
    if (abs(posError) < 0.02 && abs(currentSpd) < 0.05) {
        stop();
        return;
    }
    _integral_pos += posError * dt;
    _integral_pos = constrain(_integral_pos, -10, 10);
    float d_pos = (posError - _prevError_pos) / dt;
    float refSpeed = _kp_pos * posError + _ki_pos * _integral_pos + _kd_pos * d_pos;
    refSpeed = constrain(refSpeed, -_maxSpeed, _maxSpeed);
    _prevError_pos = posError;

    // --- Inner loop: Speed PID ---
    float spdError = refSpeed - currentSpd;
    _integral_spd += spdError * dt;
    _integral_spd = constrain(_integral_spd, -10, 10);
    float d_spd = (spdError - _prevError_spd) / dt;
    float out = _kp_spd * spdError + _ki_spd * _integral_spd + _kd_spd * d_spd;
    _prevError_spd = spdError;

    _motor->setPower((int)out);
}
