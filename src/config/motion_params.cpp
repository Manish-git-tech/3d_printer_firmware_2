#include "motion_params.h"


// Position PID for X axis
float X_PID_KP = 2.5f;
float X_PID_KI = 0.1f;
float X_PID_KD = 0.05f;

// Speed PID for X axis
float SX_PID_KP = 2.5f;
float SX_PID_KI = 0.1f;
float SX_PID_KD = 0.05f;

// Position PID for Y axis
float Y_PID_KP = 2.5f;
float Y_PID_KI = 0.1f;
float Y_PID_KD = 0.05f;

// Speed PID for Y axis
float SY_PID_KP = 2.5f;
float SY_PID_KI = 0.1f;
float SY_PID_KD = 0.05f;

// Position PID for Z axis
float Z_PID_KP = 3.0f;
float Z_PID_KI = 0.15f;
float Z_PID_KD = 0.07f;

// Speed PID for Z axis
float SZ_PID_KP = 3.0f;
float SZ_PID_KI = 0.15f;
float SZ_PID_KD = 0.07f;

// Extruder
float EXTRUDER_PID_KP = 0.8f;
float EXTRUDER_PID_KI = 0.09f;
float EXTRUDER_PID_KD = 1.2f;

void set_default_motion_params() {
    X_PID_KP = 2.5f; X_PID_KI = 0.1f; X_PID_KD = 0.05f;
    SX_PID_KP = 2.5f; SX_PID_KI = 0.1f; SX_PID_KD = 0.05f;
    Y_PID_KP = 2.5f; Y_PID_KI = 0.1f; Y_PID_KD = 0.05f;
    SY_PID_KP = 2.5f; SY_PID_KI = 0.1f; SY_PID_KD = 0.05f;
    Z_PID_KP = 3.0f; Z_PID_KI = 0.15f; Z_PID_KD = 0.07f;
    SZ_PID_KP = 3.0f; SZ_PID_KI = 0.15f; SZ_PID_KD = 0.07f;
    EXTRUDER_PID_KP = 0.8f; EXTRUDER_PID_KI = 0.09f; EXTRUDER_PID_KD = 1.2f;
}

bool load_motion_params() {
    File f = LittleFS.open("/config/motion_params", "r");
    if (!f) return false;
    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.startsWith("X_PID_KP")) sscanf(line.c_str(), "X_PID_KP=%f", &X_PID_KP);
        if (line.startsWith("X_PID_KI")) sscanf(line.c_str(), "X_PID_KI=%f", &X_PID_KI);
        if (line.startsWith("X_PID_KD")) sscanf(line.c_str(), "X_PID_KD=%f", &X_PID_KD);
        if (line.startsWith("SX_PID_KP")) sscanf(line.c_str(), "SX_PID_KP=%f", &SX_PID_KP);
        if (line.startsWith("SX_PID_KI")) sscanf(line.c_str(), "SX_PID_KI=%f", &SX_PID_KI);
        if (line.startsWith("SX_PID_KD")) sscanf(line.c_str(), "SX_PID_KD=%f", &SX_PID_KD);

        if (line.startsWith("Y_PID_KP")) sscanf(line.c_str(), "Y_PID_KP=%f", &Y_PID_KP);
        if (line.startsWith("Y_PID_KI")) sscanf(line.c_str(), "Y_PID_KI=%f", &Y_PID_KI);
        if (line.startsWith("Y_PID_KD")) sscanf(line.c_str(), "Y_PID_KD=%f", &Y_PID_KD);
        if (line.startsWith("SY_PID_KP")) sscanf(line.c_str(), "SY_PID_KP=%f", &SY_PID_KP);
        if (line.startsWith("SY_PID_KI")) sscanf(line.c_str(), "SY_PID_KI=%f", &SY_PID_KI);
        if (line.startsWith("SY_PID_KD")) sscanf(line.c_str(), "SY_PID_KD=%f", &SY_PID_KD);

        if (line.startsWith("Z_PID_KP")) sscanf(line.c_str(), "Z_PID_KP=%f", &Z_PID_KP);
        if (line.startsWith("Z_PID_KI")) sscanf(line.c_str(), "Z_PID_KI=%f", &Z_PID_KI);
        if (line.startsWith("Z_PID_KD")) sscanf(line.c_str(), "Z_PID_KD=%f", &Z_PID_KD);
        if (line.startsWith("SZ_PID_KP")) sscanf(line.c_str(), "SZ_PID_KP=%f", &SZ_PID_KP);
        if (line.startsWith("SZ_PID_KI")) sscanf(line.c_str(), "SZ_PID_KI=%f", &SZ_PID_KI);
        if (line.startsWith("SZ_PID_KD")) sscanf(line.c_str(), "SZ_PID_KD=%f", &SZ_PID_KD);

        if (line.startsWith("EXTRUDER_PID_KP")) sscanf(line.c_str(), "EXTRUDER_PID_KP=%f", &EXTRUDER_PID_KP);
        if (line.startsWith("EXTRUDER_PID_KI")) sscanf(line.c_str(), "EXTRUDER_PID_KI=%f", &EXTRUDER_PID_KI);
        if (line.startsWith("EXTRUDER_PID_KD")) sscanf(line.c_str(), "EXTRUDER_PID_KD=%f", &EXTRUDER_PID_KD);
    }
    f.close();
    return true;
}

bool save_motion_params() {
    File f = LittleFS.open("/config/motion_params", "w");
    if (!f) return false;
    f.printf("X_PID_KP=%f\nX_PID_KI=%f\nX_PID_KD=%f\n", X_PID_KP, X_PID_KI, X_PID_KD);
    f.printf("SX_PID_KP=%f\nSX_PID_KI=%f\nSX_PID_KD=%f\n", SX_PID_KP, SX_PID_KI, SX_PID_KD);
    f.printf("Y_PID_KP=%f\nY_PID_KI=%f\nY_PID_KD=%f\n", Y_PID_KP, Y_PID_KI, Y_PID_KD);
    f.printf("SY_PID_KP=%f\nSY_PID_KI=%f\nSY_PID_KD=%f\n", SY_PID_KP, SY_PID_KI, SY_PID_KD);
    f.printf("Z_PID_KP=%f\nZ_PID_KI=%f\nZ_PID_KD=%f\n", Z_PID_KP, Z_PID_KI, Z_PID_KD);
    f.printf("SZ_PID_KP=%f\nSZ_PID_KI=%f\nSZ_PID_KD=%f\n", SZ_PID_KP, SZ_PID_KI, SZ_PID_KD);
    f.printf("EXTRUDER_PID_KP=%f\nEXTRUDER_PID_KI=%f\nEXTRUDER_PID_KD=%f\n", EXTRUDER_PID_KP, EXTRUDER_PID_KI, EXTRUDER_PID_KD);
    f.close();
    return true;
}
