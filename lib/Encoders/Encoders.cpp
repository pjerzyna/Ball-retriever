#include "Encoders.h"

bool Encoders::begin(const Config& cfg) {
  _cfg = cfg;

  pinMode(_cfg.pinL, _cfg.usePullup ? INPUT_PULLUP : INPUT);
  pinMode(_cfg.pinR, _cfg.usePullup ? INPUT_PULLUP : INPUT);

  // ESP32: interrupts with an argument are passed on with 'this'
  attachInterruptArg(digitalPinToInterrupt(_cfg.pinL), Encoders::isrL, this, FALLING);
  attachInterruptArg(digitalPinToInterrupt(_cfg.pinR), Encoders::isrR, this, FALLING);

  _lastReportMs = millis();
  _ok = true;

  Serial.println(F("OK: Encoders ready."));
  return true;
}

void IRAM_ATTR Encoders::isrL(void* arg) {
  auto* self = static_cast<Encoders*>(arg);
  uint32_t now = micros();
  if (now - self->_lastUsL >= self->_cfg.minPulseUs) {
    self->_countL++;
    self->_totalL++;
    self->_lastUsL = now;
  }
}

void IRAM_ATTR Encoders::isrR(void* arg) {
  auto* self = static_cast<Encoders*>(arg);
  uint32_t now = micros();
  if (now - self->_lastUsR >= self->_cfg.minPulseUs) {
    self->_countR++;
    self->_totalR++;
    self->_lastUsR = now;
  }
}

void Encoders::reset() {
  if (!_ok) return;
  noInterrupts();
  _countL = 0;
  _countR = 0;
  //_totalL = 0; 
  //_totalR = 0; 
  interrupts();
}

void Encoders::update() {
  _newSample = false;
  if (!_ok) return;

  uint32_t now = millis();
  if (now - _lastReportMs < _cfg.reportMs) return;

  float dt = (now - _lastReportMs) / 1000.0f;
  _lastReportMs = now;

  noInterrupts();
  uint32_t pL = _countL; _countL = 0;
  uint32_t pR = _countR; _countR = 0;
  interrupts();

  _pulsesL = pL;
  _pulsesR = pR;

  _hzL = (dt > 0.0f) ? (pL / dt) : 0.0f;
  _hzR = (dt > 0.0f) ? (pR / dt) : 0.0f;

  _rpmL = (_cfg.pulsesPerRevL > 0.0f) ? (_hzL * 60.0f / _cfg.pulsesPerRevL) : 0.0f;
  _rpmR = (_cfg.pulsesPerRevR > 0.0f) ? (_hzR * 60.0f / _cfg.pulsesPerRevR) : 0.0f;

  const float circ = 2.0f * PI * _cfg.wheelRadius_m;  // [m]
  _vL = _rpmL * circ / 60.0f;  // [m/s]
  _vR = _rpmR * circ / 60.0f;  // [m/s]

  float vAvg  = 0.5f * (_vL + _vR);   // robot's average velocity [m/s]

  _newSample = true;
}

void Encoders::printDebug(Stream& s) const {
  if (!_ok) return;

  s.print(F("[ENC] L: imp=")); s.print(_pulsesL);
  s.print(F(" Hz="));         s.print(_hzL, 2);
  s.print(F(" RPM="));        s.print(_rpmL, 1);
  s.print(F(" v="));          s.print(_vL, 3);    // [m/s]

  s.print(F(" | R: imp="));   s.print(_pulsesR);
  s.print(F(" Hz="));         s.print(_hzR, 2);
  s.print(F(" RPM="));        s.println(_rpmR, 1);
  s.print(F(" v="));          s.print(_vR, 3);    // [m/s]

}
