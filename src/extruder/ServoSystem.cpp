// ServoSystem.cpp
#include "ServoSystem.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

ServoSystem::ServoSystem(uint servo_pin, uint pot_pin, float mm_to_deg, float steps_per_mm) 
    : _servo_pin(servo_pin), _pot_pin(pot_pin), _mm_to_deg_factor(mm_to_deg) ,_step_size_mm(1.0f/steps_per_mm){}

void ServoSystem::begin() {
    adc_init();
    adc_gpio_init(_pot_pin);
    adc_select_input(_pot_pin - 26);
    
    gpio_set_function(_servo_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(_servo_pin);
    pwm_set_clkdiv(slice_num, 64.0f);
    pwm_set_wrap(slice_num, 19999);
    pwm_set_enabled(slice_num, true);
}

void ServoSystem::moveTo(float mm) {
    _target_deg = mm * _mm_to_deg_factor;
    _moving = true;
}
void ServoSystem::moveTo(float mm, float mm_per_sec) {
    _target_mm = mm;
    _max_speed_mm_s = fabs(mm_per_sec);
    _moving = true;
}

void ServoSystem::step(int direction) {
    _target_mm += direction * _step_size_mm;
    _moving = true;
}

void ServoSystem::stop() {
    _moving = false;
    pwm_set_gpio_level(_servo_pin, 0);
}

void ServoSystem::_cascaded_update() {
    // 1. Compute time since last update
    unsigned long now = millis();
    float dt = (now - _last_speed_time) / 1000.0;
    if (dt < 0.01) return;

    // 2. Compute current mm from potentiometer
    float current_mm = getCurrentMM();

    // 3. Compute speed (mm/s)
    _current_speed_mm_s = (current_mm - _last_mm) / dt;
    _last_mm = current_mm;
    _last_speed_time = now;

    // 4. Outer loop: position error
    float pos_error = _target_mm - current_mm;
    float direction = (pos_error > 0) ? 1 : -1;

    // 5. Determine reference speed (mm/s), limit to max
    float ref_speed = constrain(fabs(pos_error) / dt, 0, _max_speed_mm_s) * direction;

    // 6. Inner loop: speed PID (simplified PI for demo)
    float speed_error = ref_speed - _current_speed_mm_s;
    _integral += speed_error * dt;
    float pid_output = (_Kp * speed_error) + (_Ki * _integral);

    // 7. Update target position incrementally
    float next_mm = current_mm + pid_output * dt;
    // Clamp to not overshoot
    if ((direction > 0 && next_mm > _target_mm) || (direction < 0 && next_mm < _target_mm))
        next_mm = _target_mm;

    // 8. Convert mm to degrees and command servo
    float next_deg = next_mm * _mm_to_deg_factor;
    _update_hardware(next_deg);

    // 9. Stop if close to target
    if (fabs(pos_error) < 0.1) _moving = false;
}

float ServoSystem::getCurrentMM() {
    return _current_deg / _mm_to_deg_factor;
}
float ServoSystem::getStepsPerMM() {
    return _steps_per_mm; // Inverse of step size in mm
}

void ServoSystem::update() {
    if (!_moving) return;
    _cascaded_update();
    // Read and filter potentiometer
    float raw_angle = _read_potentiometer();
    _filtered_angle = _apply_filter(raw_angle);
    
    // Handle multi-rotation tracking
    _handle_rotation_counting(_filtered_angle);
    
    // Calculate total current position
    _current_deg = (_rotation_count * 360) + _filtered_angle;
    
    // PID calculations
    float error = _target_deg - _current_deg;
    _integral += error;
    _integral = constrain(_integral, -1000, 1000);
    float derivative = error - _last_error;
    
    // Calculate PID output
    float output = (_Kp * error) + (_Ki * _integral) + (_Kd * derivative);
    output = constrain(output, -90, 90); // Limit to ±90° from current
    
    // Apply output to servo
    float target_angle = _filtered_angle + output;
    _update_hardware(target_angle);
    
    _last_error = error;
}

float ServoSystem::_read_potentiometer() {
    uint16_t adc_value = adc_read();
    return (adc_value / 4095.0f) * 360.0f;
}

float ServoSystem::_apply_filter(float raw_angle) {
    // Median filter for noise reduction
    _angle_buffer.push(raw_angle);
    if(_angle_buffer.size() > 5) _angle_buffer.pop();
    
    // Copy to array for sorting
    float values[5];
    int i=0;
    std::queue<float> temp = _angle_buffer;
    while(!temp.empty()) {
        values[i++] = temp.front();
        temp.pop();
    }
    
    // Simple bubble sort
    for(int i=0; i<4; i++) {
        for(int j=0; j<4-i; j++) {
            if(values[j] > values[j+1]) {
                float temp = values[j];
                values[j] = values[j+1];
                values[j+1] = temp;
            }
        }
    }
    return values[2]; // Return median
}

void ServoSystem::_handle_rotation_counting(float new_angle) {
    // Detect rotation boundaries
    if(new_angle < 50 && _filtered_angle > 300) { // Forward wrap
        _rotation_count++;
    } else if(new_angle > 300 && _filtered_angle < 50) { // Backward wrap
        _rotation_count--;
    }
}

void ServoSystem::_update_hardware(float target_angle) {
    // Convert to 0-360 range
    target_angle = fmod(target_angle, 360);
    if(target_angle < 0) target_angle += 360;
    
    // Map to PWM (500-2500μs pulse width)
    float pulse_width = 500 + (target_angle / 180.0f) * 2000;
    uint16_t pwm_value = (pulse_width * 125) / 1000; // 12.5MHz clock
    
    pwm_set_gpio_level(_servo_pin, pwm_value);
}

void ServoSystem::printStatus() {
    Serial.print("Current: ");
    Serial.print(_current_deg);
    Serial.print("° (");
    Serial.print(getCurrentMM());
    Serial.print("mm) | Target: ");
    Serial.print(_target_deg);
    Serial.print("° | Rotations: ");
    Serial.println(_rotation_count);
}
