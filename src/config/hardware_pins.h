// hardware_pins.h
// All pin assignments for the 3D printer hardware

#pragma once

// X Axis Motor and Encoder
#define X_MOTOR_A_PWM_PIN      7     // PWM pin for MX1508 (direction pin)
#define X_MOTOR_B_PWM_PIN      6     // PWM pin for MX1508 (direction pin)
#define X_ENCODER_A_PIN      10     // Quadrature encoder channel A
#define X_ENCODER_B_PIN      11     // Quadrature encoder channel B

// Y Axis Motor and Encoder
#define Y_MOTOR_A_PWM_PIN      9     // PWM pin for MX1508 (direction pin)
#define Y_MOTOR_B_PWM_PIN      8     // PWM pin for MX1508 (direction pin)
#define Y_ENCODER_A_PIN      12     // Quadrature encoder channel A
#define Y_ENCODER_B_PIN      13     // Quadrature encoder channel B

// Z Axis Motor and Encoder
#define Z_MOTOR_A_PWM_PIN      3   // PWM pin for MX1508 (direction pin)
#define Z_MOTOR_B_PWM_PIN      2   // PWM pin for MX1508 (direction pin)
#define Z_ENCODER_A_PIN      14   // Quadrature encoder channel A
#define Z_ENCODER_B_PIN      15   // Quadrature encoder channel B

// Extruder Servo (360° mod) and Potentiometer
#define EXTRUDER_SERVO_PWM_PIN   16   // PWM pin for servo
#define EXTRUDER_POT_PIN         26   // Analog pin for potentiometer

// Hotend
#define HOTEND_HEATER_PIN    17   // PWM pin for MOSFET controlling nichrome wire
#define HOTEND_THERM_PIN     27   // Analog pin for thermistor

// Fan
#define FAN_PWM_PIN          18   // PWM pin for fan (MX1508 direction pin)