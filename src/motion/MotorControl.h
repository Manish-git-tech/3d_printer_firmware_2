// MotorControl.h
#pragma once
#include <Arduino.h>

// Controls MX1508 H-bridge for one DC motor
class MotorControl {
public:
    MotorControl(uint8_t in1, uint8_t in2);

    void begin();
    void setPower(int pwm); // -255..255, sign = direction
    void stop();

private:
    uint8_t _in1, _in2;
};
