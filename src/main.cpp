//main.cpp
#include "comms_executor/MotionGCodeExecutor.h"
#include <Arduino.h>
#include "comms/PIDGCodeHandler.h"

class StatusReportHandler {
public:
    StatusReportHandler(unsigned long interval_ms = 250);

    void begin();
    void update();

    // These should be implemented by you:
    float getXPos();           // mm
    float getYPos();
    float getZPos();
    float getXSpeed();         // mm/s
    float getYSpeed();
    float getZSpeed();
    float getTemperature();    // °C
    String getExtruderStatus(); // "idle", "extruding", etc.
    String getAxisStatus(char axis); // 'X', 'Y', 'Z' -> "idle", "moving"
    String getHeaterStatus();  // "idle", "heating"
    String getFanStatus();     // "on", "off"
    int getFanSpeed();         // 0-100 (%)
    float getTargetTemperature(); // °C
    float gettargetX();        // mm 
    float gettargetY();
    float gettargetZ();
    float gettargetXSpeed();   // mm/s
    float gettargetYSpeed();
    float gettargetZSpeed();
    bool isMoving();
    float getExecutionTime();  // seconds
private:
    unsigned long _interval;
    unsigned long _lastReport = 0;
    void _sendStatus();
};



PIDGCodeHandler pidHandler;



StatusReportHandler::StatusReportHandler(unsigned long interval_ms)
    : _interval(interval_ms) {}

void StatusReportHandler::begin() {
    // Nothing needed for now
}

void StatusReportHandler::update() {
    unsigned long now = millis();
    if (now - _lastReport >= _interval) {
        _sendStatus();
        _lastReport = now;
    }
}

void StatusReportHandler::_sendStatus() {
    Serial.print("STATUS ");
    Serial.print("X:"); Serial.print(getXPos(), 2); Serial.print(" ");
    Serial.print("Y:"); Serial.print(getYPos(), 2); Serial.print(" ");
    Serial.print("Z:"); Serial.print(getZPos(), 2); Serial.print(" ");
    Serial.print("TX:"); Serial.print(gettargetX(), 2); Serial.print(" ");
    Serial.print("TY:"); Serial.print(gettargetY(), 2); Serial.print(" ");
    Serial.print("TZ:"); Serial.print(gettargetZ(), 2); Serial.print(" ");
    Serial.print("XS:"); Serial.print(getXSpeed(), 2); Serial.print(" ");
    Serial.print("YS:"); Serial.print(getYSpeed(), 2); Serial.print(" ");
    Serial.print("ZS:"); Serial.print(getZSpeed(), 2); Serial.print(" ");
    Serial.print("TXS:"); Serial.print(gettargetXSpeed(), 2); Serial.print(" ");
    Serial.print("TYS:"); Serial.print(gettargetYSpeed(), 2); Serial.print(" ");
    Serial.print("TZS:"); Serial.print(gettargetZSpeed(), 2); Serial.print(" ");
    Serial.print("TEMP:"); Serial.print(getTemperature(), 1); Serial.print(" ");
    Serial.print("TARGET_TEMP:"); Serial.print(getTargetTemperature(), 1); Serial.print(" ");
    Serial.print("EXTRUDER:"); Serial.print(getExtruderStatus()); Serial.print(" ");
    Serial.print("XSTAT:"); Serial.print(getAxisStatus('X')); Serial.print(" ");
    Serial.print("YSTAT:"); Serial.print(getAxisStatus('Y')); Serial.print(" ");
    Serial.print("ZSTAT:"); Serial.print(getAxisStatus('Z')); Serial.print(" ");
    Serial.print("HEATER:"); Serial.print(getHeaterStatus()); Serial.print(" ");
    Serial.print("FAN:"); Serial.print(getFanStatus()); Serial.print(" ");
    Serial.print("FANSPEED:"); Serial.print(getFanSpeed());Serial.print(" ");
    Serial.print("Printing_status:"); Serial.print(isMoving() ? "printing" : "idle"); 
    Serial.print(" Execution_time: "); Serial.print(getExecutionTime(), 2);  
    
    Serial.println();
}


float StatusReportHandler::getXPos()        { return xAxis.getCurrentMM(); }
float StatusReportHandler::getYPos()        { return yAxis.getCurrentMM(); }
float StatusReportHandler::getZPos()        { return zAxis.getCurrentMM(); }
float StatusReportHandler::getXSpeed()      { return xAxis.getCurrentSpeedMM_S(); }
float StatusReportHandler::getYSpeed()      { return yAxis.getCurrentSpeedMM_S(); }
float StatusReportHandler::getZSpeed()      { return zAxis.getCurrentSpeedMM_S(); }
float StatusReportHandler::getTemperature() { return 25.0f; } 
String StatusReportHandler::getExtruderStatus() { return extruderServo._moving ? "extruding" : "idle"; }
String StatusReportHandler::getAxisStatus(char axis) { switch (axis) { case 'X': if(xAxis._pid._active) return "moving"; else return "idle"; 
                                                            case 'Y': if(yAxis._pid._active) return "moving"; else return "idle"; 
                                                            case 'Z': if(zAxis._pid._active) return "moving"; else return "idle"; 
                                                            default: return "unknown"; } }
String StatusReportHandler::getHeaterStatus() { return "idle"; } 
String StatusReportHandler::getFanStatus()    { return "off"; }
int StatusReportHandler::getFanSpeed()        { return 0; }
float StatusReportHandler::getTargetTemperature() { return 0.0f; }
float StatusReportHandler::gettargetX() { return motion.targetX; }
float StatusReportHandler::gettargetY() { return motion.targetY; }
float StatusReportHandler::gettargetZ() { return motion.targetZ; }
float StatusReportHandler::gettargetXSpeed() { return xAxis.speed; }
float StatusReportHandler::gettargetYSpeed() { return yAxis.speed; }
float StatusReportHandler::gettargetZSpeed() { return zAxis.speed; }
bool StatusReportHandler::isMoving() {
    return xAxis._pid._active || yAxis._pid._active || zAxis._pid._active;
}
float StatusReportHandler::getExecutionTime() {return motion._planner.time;}
StatusReportHandler statusReportHandler(250); // 250 ms interval

void setup() {
    // Initialize hardware and controllers
    initAxisControllers();
    initExtruderServo();
    LittleFS.begin();
    load_motion_params();
    Serial.begin(115200);
}

void loop() {
    // Update axis controllers
    motion.update();
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        int handled = 0;
            if (cmd.startsWith("G")) {
                handled = handleGSerialCommands(cmd); // 1 if handled, 0 if not
            } else if (cmd.startsWith("M")) {
                handled = pidHandler.parseAndApply(cmd); // 1 if handled, 0 if not
                // Apply the new PID values to the respective axes
                xAxis.setPositionGains(X_PID_KP, X_PID_KI, X_PID_KD);
                xAxis.setSpeedGains(SX_PID_KP, SX_PID_KI, SX_PID_KD);
                yAxis.setPositionGains(Y_PID_KP, Y_PID_KI, Y_PID_KD);
                yAxis.setSpeedGains(SY_PID_KP, SY_PID_KI, SY_PID_KD);
                zAxis.setPositionGains(Z_PID_KP, Z_PID_KI, Z_PID_KD);
                zAxis.setSpeedGains(SZ_PID_KP, SZ_PID_KI, SZ_PID_KD);
                extruderServo.setPositionGains(EXTRUDER_PID_KP, EXTRUDER_PID_KI, EXTRUDER_PID_KD);
            }
            if (handled == 0) {
                Serial.println("Unknown command: " + cmd);
            }
    }
    statusReportHandler.update();
    
}