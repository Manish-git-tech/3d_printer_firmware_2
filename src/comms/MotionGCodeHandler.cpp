#include "MotionGCodeHandler.h"

MotionGCodeCommand MotionGCodeHandler::parse(const String& command) {
    String cleanCmd = command;
    cleanCmd.trim();
    cleanCmd.toUpperCase();
    
    // Remove comments
    int commentIndex = cleanCmd.indexOf(';');
    if(commentIndex != -1) {
        cleanCmd = cleanCmd.substring(0, commentIndex);
    }

    if(cleanCmd.startsWith("G0")) return _parseG0G1(cleanCmd);
    if(cleanCmd.startsWith("G1")) return _parseG0G1(cleanCmd);
    if(cleanCmd.startsWith("G28")) return _parseG28(cleanCmd);
    
    return {MotionGCodeCommand::UNSUPPORTED};
}

MotionGCodeCommand MotionGCodeHandler::_parseG0G1(const String& command) {
    MotionGCodeCommand cmd;
    cmd.type = command.startsWith("G0") ? MotionGCodeCommand::G0 : MotionGCodeCommand::G1;
    
    cmd.x = _parseValue(command, 'X');
    cmd.y = _parseValue(command, 'Y');
    cmd.z = _parseValue(command, 'Z');
    cmd.e = _parseValue(command, 'E');
    cmd.f = _parseValue(command, 'F');
    
    return cmd;
}

MotionGCodeCommand MotionGCodeHandler::_parseG28(const String& command) {
    MotionGCodeCommand cmd;
    cmd.type = MotionGCodeCommand::G28;
    
    cmd.home_x = command.indexOf('X') != -1;
    cmd.home_y = command.indexOf('Y') != -1;
    cmd.home_z = command.indexOf('Z') != -1;
    
    // If no axes specified, home all
    if(!cmd.home_x && !cmd.home_y && !cmd.home_z) {
        cmd.home_x = cmd.home_y = cmd.home_z = true;
    }
    
    return cmd;
}

float MotionGCodeHandler::_parseValue(const String& command, char parameter) {
    int paramIndex = command.indexOf(parameter);
    if(paramIndex == -1) return NAN;
    
    int start = paramIndex + 1;
    int end = start;
    while(end < command.length() && 
          (isDigit(command[end]) || command[end] == '.' || command[end] == '-')) {
        end++;
    }
    
    return command.substring(start, end).toFloat();
}
