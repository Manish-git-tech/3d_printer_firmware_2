// motion_params.h
// Motion system parameters: encoder settings, backlash, PID, etc.

#pragma once
#include <LittleFS.h>

// Encoder slits (Pulses Per Millimeter) for each axis
#define X_ENCODER_SLITS_PER_MM   120   // Example value, measure for your setup
#define Y_ENCODER_SLITS_PER_MM   120
#define Z_ENCODER_SLITS_PER_MM   200

//Encoder pulses per revolution (for multi-turn encoders)
#define X_PULSES_PER_REV         4
#define Y_PULSES_PER_REV         4
#define Z_PULSES_PER_REV         4

// Gear ratios (if any), set to 1.0 if direct drive
#define X_GEAR_RATIO             2.0   // Example: 2:1 gear ratio
#define Y_GEAR_RATIO             2.0
#define Z_GEAR_RATIO             3.0

// Backlash compensation (in mm)
#define X_BACKLASH_MM            0.5
#define Y_BACKLASH_MM            0.5
#define Z_BACKLASH_MM            0.7

// Motion limits (printable area, in mm)
#define X_MIN_POS                0
#define X_MAX_POS                200
#define Y_MIN_POS                0
#define Y_MAX_POS                200
#define Z_MIN_POS                0
#define Z_MAX_POS                180

#define X_DEFAULT_SPEED_MM_S        50.0f // Default speed for X axis in mm/s
#define Y_DEFAULT_SPEED_MM_S        50.0f // Default speed for Y axis in mm/s
#define Z_DEFAULT_SPEED_MM_S        50.0f // Default speed for Z axis in mm/s

// PID default values for each axis (can be tuned and updated via serial)
// Position PID for X axis
extern float X_PID_KP;
extern float X_PID_KI;
extern float X_PID_KD;

// Speed PID for X axis
extern float SX_PID_KP;
extern float SX_PID_KI;
extern float SX_PID_KD;

// Position PID for Y axis
extern float Y_PID_KP;
extern float Y_PID_KI;
extern float Y_PID_KD;

// Speed PID for Y axis
extern float SY_PID_KP;
extern float SY_PID_KI;
extern float SY_PID_KD;

// Position PID for Z axis
extern float Z_PID_KP;
extern float Z_PID_KI;
extern float Z_PID_KD;

// Speed PID for Z axis
extern float SZ_PID_KP;
extern float SZ_PID_KI;
extern float SZ_PID_KD;

// Extruder PID
extern float EXTRUDER_PID_KP;
extern float EXTRUDER_PID_KI;
extern float EXTRUDER_PID_KD;

// Function to load/save
bool load_motion_params();
bool save_motion_params();
void set_default_motion_params();

#define EXTRUDER_MM_TO_DEG 0.1f 
#define EXTRUDER_STEPS_PER_MM 10.0f 
#define EXTRUDER_MAX_SPEED_MM_S 60.0f // Maximum speed for extruder in mm/s
#define EXTRUDER_DEFAULT_SPEED_MM 10.0f // Default speed for extruder in mm


// Default feed rate for G0/G1 commands
#define DEFAULT_FEED_RATE 50.0f // Default feed rate in mm/s