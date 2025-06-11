#include "comms/MotionGCodeHandler.h"
#include "init/motion.h"


MotionGCodeHandler motionParser;

int handleGSerialCommands(const String& command) {

        MotionGCodeCommand cmd = motionParser.parse(command);
        
        switch(cmd.type) {
            case MotionGCodeCommand::G0:
                if (isnan(cmd.x)) {
                    cmd.x = xAxis.getCurrentMM();
                }
                if (isnan(cmd.y)) {
                    cmd.y = yAxis.getCurrentMM();
                }
                if (isnan(cmd.z)) {
                    cmd.z = zAxis.getCurrentMM();
                }
                if (isnan(cmd.e)) {
                    cmd.e = extruderServo.getCurrentMM();
                }
                if (isnan(cmd.f)) {
                    cmd.f = motion.lastFeedRate; // last feedrate used
                }
                cmd.e = extruderServo.getCurrentMM();
                motion.moveLinear(cmd.x, cmd.y, cmd.z, cmd.e, cmd.f);
                Serial.print("Moving...  ");
                return 1; // G0 is non-blocking, just return
                break;
            case MotionGCodeCommand::G1:
                if (isnan(cmd.x)) {
                    cmd.x = xAxis.getCurrentMM();
                }
                if (isnan(cmd.y)) {
                    cmd.y = yAxis.getCurrentMM();
                }
                if (isnan(cmd.z)) {
                    cmd.z = zAxis.getCurrentMM();
                }
                if (isnan(cmd.e)) {
                    cmd.e = extruderServo.getCurrentMM();
                }
                if (isnan(cmd.f)) {
                    cmd.f = motion.lastFeedRate; // last feedrate used
                }
                motion.moveLinear(cmd.x, cmd.y, cmd.z, cmd.e, cmd.f);
                Serial.print("Moving...  ");
                return 1;
                break;
            case MotionGCodeCommand::G28:
                if (cmd.home_x) {
                   cmd.x = 0.0f; 
                }
                else {
                   cmd.x = xAxis.getCurrentMM(); // Keep current position if not homing
                }
                if (cmd.home_y) {
                   cmd.y = 0.0f; 
                }
                else {
                   cmd.y = yAxis.getCurrentMM(); // Keep current position if not homing
                }
                if (cmd.home_z) {
                     cmd.z = 0.0f; 
                 }
                 else {
                     cmd.z = zAxis.getCurrentMM(); // Keep current position if not homing
                }
                cmd.e = extruderServo.getCurrentMM();
                // Home axes
                motion.moveLinear(cmd.x, cmd.y, cmd.z, cmd.e , 200); // Move to home position
                Serial.print("Moving...  ");
                return 1;
                break;

            case MotionGCodeCommand::G112:
                xAxis.stop();
                yAxis.stop();
                zAxis.stop();

            case MotionGCodeCommand::UNSUPPORTED:
                return 0; // Unsupported command, do nothing
            default:
                return 0; // Unknown command, do nothing
        }
        return 0; // No command processed
    }
    