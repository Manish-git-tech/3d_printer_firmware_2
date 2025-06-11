// Encoder.h
#pragma once
#include <Arduino.h>

// Handles quadrature encoder with interrupts
class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB, int pulsesPerRev);

    void begin();
    long getPosition();           // Returns encoder tick count
    float getRotations();         // Returns rotations
    void reset();
    void updateSpeed();           // Call regularly to update speed
    float getSpeed();             // Returns speed in rotations/sec

    // ISR call; must be static for attachInterrupt
    static void handleInterruptA();
    static void handleInterruptB();

private:
    uint8_t _pinA, _pinB;
    int _pulsesPerRev;
    volatile long _position;
    volatile int8_t _lastEncoded;
    unsigned long _lastSpeedTime;
    long _lastPosition;
    float _speed; // rotations/sec

    static Encoder* instance; // For static ISR routing
    void encoderISR();        // Actual ISR logic
};
