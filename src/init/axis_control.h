#include "config/hardware_pins.h"
#include "motion/AxisController.h"
#include "config/motion_params.h"
#include "init/motor.h"
#include "init/encoder.h"

extern AxisController xAxis;  // Declare as external
extern AxisController yAxis;  // Declare as external  
extern AxisController zAxis;  // Declare as external

// Function to initialize all axis controllers
void initAxisControllers() {
    xAxis.begin();
    yAxis.begin();
    zAxis.begin();
    
    // Set default PID gains for each axis
    xAxis.setPositionGains(X_PID_KP, X_PID_KI, X_PID_KD);
    xAxis.setSpeedGains(SX_PID_KD, SX_PID_KI, SX_PID_KP);

    yAxis.setPositionGains(Y_PID_KP, Y_PID_KI, Y_PID_KD);
    yAxis.setSpeedGains(SY_PID_KD, SY_PID_KI, SY_PID_KP);

    zAxis.setPositionGains(Z_PID_KP, Z_PID_KI, Z_PID_KD);
    zAxis.setSpeedGains(SZ_PID_KD, SZ_PID_KI, SZ_PID_KP);
}