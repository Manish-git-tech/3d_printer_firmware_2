// ServoSystem.h
#pragma once
#include <Arduino.h>
#include <queue>

class ServoSystem {
public:
    ServoSystem(uint servo_pin, uint pot_pin, float mm_to_deg, float steps_per_mm);
    
    
    void begin();
    void update();
    void moveTo(float mm);
    void moveTo(float mm, float mm_per_sec);
    void step(int direction);  // +1 or -1
    void stop();
    float getCurrentMM();
    float getStepsPerMM();
    void printStatus();
    void setPositionGains(float Kp, float Ki, float Kd) {
        _Kp = Kp; _Ki = Ki; _Kd = Kd;
    }
    void setup(float max_speed_mm_s) {
        _max_speed_mm_s = max_speed_mm_s;
    }
    bool _moving = false;

private:
    // Hardware pins
    uint _servo_pin, _pot_pin;
    
    // Steps per mm conversion
    float _steps_per_mm;
    float _step_size_mm;
    float _mm_to_deg_factor;

    // Position tracking
    float _current_deg = 0;
    float _target_deg = 0;
    int _rotation_count = 0;
    

    // PID Control
    float _Kp, _Ki, _Kd ;
    float _integral = 0, _last_error = 0;
    
    // Filtering
    std::queue<float> _angle_buffer;
    float _filtered_angle = 0;
    
    // Hardware control
    void _update_hardware(float target_angle);
    float _read_potentiometer();
    float _apply_filter(float raw_angle);
    void _handle_rotation_counting(float new_angle);
    uint16_t _degrees_to_pwm(float degrees);
    float _target_mm = 0;
    float _max_speed_mm_s ;
    float _current_speed_mm_s = 0;
    float _last_mm = 0;
    unsigned long _last_speed_time = 0;
    void _cascaded_update();
};
