// Encoder.cpp
#include "Encoder.h"

Encoder* Encoder::instance = nullptr;

Encoder::Encoder(uint8_t pinA, uint8_t pinB, int pulsesPerRev)
    : _pinA(pinA), _pinB(pinB), _pulsesPerRev(pulsesPerRev),
      _position(0), _lastEncoded(0), _lastSpeedTime(0), _lastPosition(0), _speed(0.0f)
{
    instance = this;
}

void Encoder::begin() {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    int MSB = digitalRead(_pinA);
    int LSB = digitalRead(_pinB);
    _lastEncoded = (MSB << 1) | LSB;
    attachInterrupt(digitalPinToInterrupt(_pinA), Encoder::handleInterruptA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_pinB), Encoder::handleInterruptB, CHANGE);
}

void Encoder::handleInterruptA() { if (instance) instance->encoderISR(); }
void Encoder::handleInterruptB() { if (instance) instance->encoderISR(); }

void Encoder::encoderISR() {
    int MSB = digitalRead(_pinA);
    int LSB = digitalRead(_pinB);
    int encoded = (MSB << 1) | LSB;
    int sum = (_lastEncoded << 2) | encoded;
    static const int8_t lookupTable[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };
    _position += lookupTable[sum];
    _lastEncoded = encoded;
}

long Encoder::getPosition() { return _position; }
float Encoder::getRotations() { return (float)_position / _pulsesPerRev; }
void Encoder::reset() { noInterrupts(); _position = 0; interrupts(); }
void Encoder::updateSpeed() {
    unsigned long now = millis();
    float dt = (now - _lastSpeedTime) / 1000.0;
    if (dt < 0.01) return;
    long delta = _position - _lastPosition;
    _speed = (float)delta / _pulsesPerRev / dt;
    _lastPosition = _position;
    _lastSpeedTime = now;
}
float Encoder::getSpeed() { return _speed; }
