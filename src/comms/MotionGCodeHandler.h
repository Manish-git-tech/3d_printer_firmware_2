#pragma once
#include <Arduino.h>

struct MotionGCodeCommand {
    enum Type { G0, G1, G28, G112, UNSUPPORTED };
    Type type;
    
    // For G0/G1
    float x = NAN;
    float y = NAN;
    float z = NAN;
    float e = NAN;
    float f = NAN;
    
    // For G28
    bool home_x = false;
    bool home_y = false;
    bool home_z = false;
};

class MotionGCodeHandler {
public:
    MotionGCodeCommand parse(const String& command);
    
private:
    MotionGCodeCommand _parseG0G1(const String& command);
    MotionGCodeCommand _parseG28(const String& command);
    float _parseValue(const String& command, char parameter);
};
