#include "Servo360.h"

// --- Servo is not used in this project ---

bool Servo360::begin(int pin, int stopUs, int minUs, int maxUs) {
  _pin = pin;
  _stopUs = stopUs;
  _minUs = minUs;
  _maxUs = maxUs;

  _s.setPeriodHertz(50);
  _s.attach(_pin, _minUs, _maxUs);
  _s.writeMicroseconds(_stopUs);
  return true;
}

void Servo360::setSpeed(float speed) {
  if (speed > 1) speed = 1;
  if (speed < -1) speed = -1;

  int us = _stopUs + (int)(speed * _maxDelta);

  if (us < _minUs) us = _minUs;
  if (us > _maxUs) us = _maxUs;

  _s.writeMicroseconds(us);
}
