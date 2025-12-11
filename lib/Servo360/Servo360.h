#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

class Servo360 {
public:
  bool begin(int pin, int stopUs = 1500, int minUs = 1000, int maxUs = 2000);

  // speed: -1..1 (minus = backward), 0 = stop
  void setSpeed(float speed);

  void stop() { setSpeed(0); }

  int stopUs() const { return _stopUs; }
  void setStopUs(int us) { _stopUs = us; }

private:
  Servo _s;
  int _pin = -1;
  int _stopUs = 1500;
  int _minUs = 1000;
  int _maxUs = 2000;
  int _maxDelta = 250; // speed limit
};
