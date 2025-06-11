#pragma once
#include <Arduino.h>

class PIDGCodeHandler {
public:
    PIDGCodeHandler();
    int parseAndApply(const String& command); // returns true if handled
    void _reportCurrentPIDValues();
private:
    void _parseM2000(const String& command);
    float _parseValue(const String& command, const String& key);
};
