#include "PIDGCodeHandler.h"
#include "config/motion_params.h"

PIDGCodeHandler::PIDGCodeHandler() {}

int PIDGCodeHandler::parseAndApply(const String& command) {
    if (command.startsWith("M2000")) {
        _parseM2000(command);
        save_motion_params();
        Serial.println("ok");
        return 1;
    }
    else if (command.startsWith("M2001")) {  // Add new command handler
        _reportCurrentPIDValues();
        Serial.println("ok");
        return 1;
    }
    return 0;
}

void PIDGCodeHandler::_parseM2000(const String& command) {
    // Example: M2000 X_PID_KP=1.1 X_PID_KI=0.2 X_PID_KD=0.03 SX_PID_KP=2.0 ...
    const char* keys[] = {
        "X_PID_KP", "X_PID_KI", "X_PID_KD",
        "SX_PID_KP", "SX_PID_KI", "SX_PID_KD",
        "Y_PID_KP", "Y_PID_KI", "Y_PID_KD",
        "SY_PID_KP", "SY_PID_KI", "SY_PID_KD",
        "Z_PID_KP", "Z_PID_KI", "Z_PID_KD",
        "SZ_PID_KP", "SZ_PID_KI", "SZ_PID_KD",
        "EXTRUDER_PID_KP", "EXTRUDER_PID_KI", "EXTRUDER_PID_KD"
    };
    float* vars[] = {
        &X_PID_KP, &X_PID_KI, &X_PID_KD,
        &SX_PID_KP, &SX_PID_KI, &SX_PID_KD,
        &Y_PID_KP, &Y_PID_KI, &Y_PID_KD,
        &SY_PID_KP, &SY_PID_KI, &SY_PID_KD,
        &Z_PID_KP, &Z_PID_KI, &Z_PID_KD,
        &SZ_PID_KP, &SZ_PID_KI, &SZ_PID_KD,
        &EXTRUDER_PID_KP, &EXTRUDER_PID_KI, &EXTRUDER_PID_KD
    };
    for (int i = 0; i < sizeof(keys)/sizeof(keys[0]); ++i) {
        float val = _parseValue(command, keys[i]);
        if (!isnan(val)) {
            *vars[i] = val;
        }
    }
    
}

float PIDGCodeHandler::_parseValue(const String& command, const String& key) {
    int idx = command.indexOf(key + "=");
    if (idx == -1) return NAN;
    int start = idx + key.length() + 1;
    int end = command.indexOf(' ', start);
    if (end == -1) end = command.length();
    return command.substring(start, end).toFloat();
}

void PIDGCodeHandler::_reportCurrentPIDValues() {
    Serial.printf(
        "X_PID_KP=%.4f X_PID_KI=%.4f X_PID_KD=%.4f "
        "SX_PID_KP=%.4f SX_PID_KI=%.4f SX_PID_KD=%.4f "
        "Y_PID_KP=%.4f Y_PID_KI=%.4f Y_PID_KD=%.4f "
        "SY_PID_KP=%.4f SY_PID_KI=%.4f SY_PID_KD=%.4f "
        "Z_PID_KP=%.4f Z_PID_KI=%.4f Z_PID_KD=%.4f "
        "SZ_PID_KP=%.4f SZ_PID_KI=%.4f SZ_PID_KD=%.4f "
        "EXTRUDER_PID_KP=%.4f EXTRUDER_PID_KI=%.4f EXTRUDER_PID_KD=%.4f\n",
        X_PID_KP, X_PID_KI, X_PID_KD,
        SX_PID_KP, SX_PID_KI, SX_PID_KD,
        Y_PID_KP, Y_PID_KI, Y_PID_KD,
        SY_PID_KP, SY_PID_KI, SY_PID_KD,
        Z_PID_KP, Z_PID_KI, Z_PID_KD,
        SZ_PID_KP, SZ_PID_KI, SZ_PID_KD,
        EXTRUDER_PID_KP, EXTRUDER_PID_KI, EXTRUDER_PID_KD
    );
}

