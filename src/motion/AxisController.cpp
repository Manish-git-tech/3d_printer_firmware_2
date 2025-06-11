#include "AxisController.h"

AxisController::AxisController(Encoder& _encoder, MotorControl& _motor,  float rotations_per_mm,int pulses_per_rev)
    : _encoder(_encoder),
      _motor(_motor),
      _pid(&_encoder, &_motor, pulses_per_rev),
      _rotations_per_mm(rotations_per_mm),
      _steps_per_mm(rotations_per_mm * pulses_per_rev){}

void AxisController::begin() {
    _encoder.begin();
    _motor.begin();
}

void AxisController::moveTo(float mm, float mm_per_sec) {
    speed  = mm_per_sec; // Store speed for later use
    float target_rotations = mm * _rotations_per_mm;
    float speed_rotations_sec = mm_per_sec * _rotations_per_mm;
    _pid.moveTo(target_rotations, speed_rotations_sec);
}

void AxisController::step(int direction) {
    // direction: +1 for forward, -1 for backward
    float current = _encoder.getRotations() / _rotations_per_mm;
    Serial.print("_pid.moveTo: initialised ");
    _pid.moveTo(current + direction / _steps_per_mm, 0); // move by one step in the correct direction
}

float AxisController::getStepsPerMM() const {
    return _steps_per_mm;
}

void AxisController::update() {
    _pid.update();
}

void AxisController::stop() {
    _pid.stop();
    speed = 0; // Reset speed when stopping
}

float AxisController::getCurrentMM() {
    return _encoder.getRotations() / _rotations_per_mm;
}

float AxisController::getCurrentSpeedMM_S() {
    return _encoder.getSpeed() / _rotations_per_mm;
}

void AxisController::setPositionGains(float kp, float ki, float kd) {
    _pid.setPositionGains(kp, ki, kd);
}

void AxisController::setSpeedGains(float kp, float ki, float kd) {
    _pid.setSpeedGains(kp, ki, kd);
}
