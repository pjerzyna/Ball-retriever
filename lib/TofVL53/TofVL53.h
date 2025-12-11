#pragma once
#include <Arduino.h>
#include <Wire.h>

// Name collision: 'sensor_t' (Adafruit vs esp_camera)
#define sensor_t adafruit_sensor_t
#include <Adafruit_VL53L0X.h>
#undef sensor_t

class TofVL53 {
public:
  bool begin(TwoWire& wire, uint8_t addr = 0x29);

  void setPeriodMs(uint32_t p) { _periodMs = p; }
  void update();                 

  bool ok() const { return _ok; }
  bool hasNew() const { return _newSample; }

  // last reading
  float distanceM() const { return _mm * 0.001f; }
  uint16_t distanceMmFiltered() const { return _mmFilt; }
  bool valid() const { return _valid; }   // true if measurement is OK

  void printDebug(Stream& s = Serial) const;

private:
  TwoWire* _wire = nullptr;
  Adafruit_VL53L0X _lox;
  bool _ok = false;

  uint32_t _periodMs = 100;  // by default 10 Hz
  uint32_t _lastMs = 0;
  bool _newSample = false;

  uint16_t _mm = 0;
  bool _valid = false;
  uint16_t _mmFilt = 0;
};
