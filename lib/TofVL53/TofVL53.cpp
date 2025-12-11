#include "TofVL53.h"

bool TofVL53::begin(TwoWire& wire, uint8_t addr) {
  _wire = &wire;

  bool ok = false;
  for (int i = 0; i < 5 && !ok; i++) {
    ok = _lox.begin(addr, false, _wire);
    if (!ok) delay(100);
  }

  if (!ok) {
    Serial.println(F("ERROR: VL53L0X doesn't respond at 0x29."));
    _ok = false;
    return false;
  }

  Serial.println(F("OK: VL53L0X ready (mm)."));
  _ok = true;
  _lastMs = millis();
  return true;
}

void TofVL53::update() {
  _newSample = false;
  if (!_ok) return;

  uint32_t now = millis();
  if (now - _lastMs < _periodMs) return;
  _lastMs = now;

  VL53L0X_RangingMeasurementData_t m;
  _lox.rangingTest(&m, false);

  if (m.RangeStatus != 4 && m.RangeMilliMeter > 0) {
    // valid measurement
    _mm    = m.RangeMilliMeter;
    _valid = true;

    // basic distance filter [mm]
    const float ALPHA = 0.5f;   // (0..1)
    if (_mmFilt == 0) {
      // first time is direct
      _mmFilt = _mm;
    } else {
      _mmFilt = (uint16_t)(ALPHA * (float)_mm +
                           (1.0f - ALPHA) * (float)_mmFilt);
    }
  } else {
    // error or out of range
    _mm    = 0;
    _valid = false;
    // _mmFilt is deliberately left unchanged  - keep last reasonable value 
  }

  _newSample = true;
}


void TofVL53::printDebug(Stream& s) const {
  if (!_ok) return;

  if (_valid) {
    s.print(F("[VL53] raw="));
    s.print(_mm);
    s.print(F(" mm  filt="));
    s.print(_mmFilt);
    s.println(F(" mm"));
  } else {
    s.println(F("[VL53] out of range / error"));
  }
}

