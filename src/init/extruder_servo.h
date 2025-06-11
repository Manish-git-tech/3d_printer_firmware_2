#include "extruder/ServoSystem.h"
#include "config/hardware_pins.h"
#include "config/motion_params.h"

// Initialize the extruder servo system
ServoSystem extruderServo(EXTRUDER_SERVO_PWM_PIN, EXTRUDER_POT_PIN, EXTRUDER_MM_TO_DEG, EXTRUDER_STEPS_PER_MM);


// Function to initialize the extruder servo
void initExtruderServo() {
    extruderServo.begin();
    
    // Set default PID gains for the extruder
    extruderServo.setPositionGains(EXTRUDER_PID_KP, EXTRUDER_PID_KI, EXTRUDER_PID_KD);
    
    // Optionally set initial position or speed
    extruderServo.moveTo(0.0f); // Start at 0 mm
    extruderServo.setup(EXTRUDER_MAX_SPEED_MM_S); // Set max speed to 60 mm/s
}