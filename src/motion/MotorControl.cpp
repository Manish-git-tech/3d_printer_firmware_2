// MotorControl.cpp
#include "MotorControl.h"

MotorControl::MotorControl(uint8_t in1, uint8_t in2) : _in1(in1), _in2(in2) {}

void MotorControl::begin() {
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    stop();
}

void MotorControl::setPower(int pwm) {
    int value = constrain(abs(pwm), 0, 255);
    if (value > 0 && value < 120) value = 120; // Minimum threshold for MX1508
    if (pwm > 0) {
        analogWrite(_in1, value);
        digitalWrite(_in2, LOW);
    } else if (pwm < 0) {
        digitalWrite(_in1, LOW);
        analogWrite(_in2, value);
    } else {
        stop();
    }
}

void MotorControl::stop() {
    analogWrite(_in1, 0);
    analogWrite(_in2, 0);
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
}
