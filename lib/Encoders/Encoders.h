#pragma once
#include <Arduino.h>

class Encoders {
public:
  struct Config {
    uint8_t pinL;
    uint8_t pinR;
    bool usePullup = false;          // LM393 zwykle ma pull-up → false
    float pulsesPerRevL = 20.0f;
    float pulsesPerRevR = 20.0f;
    uint32_t minPulseUs = 200;       // filtr czasu między impulsami
    uint32_t reportMs = 500;         // co ile liczyć RPM
  };

  bool begin(const Config& cfg);

  void update();              // wołaj w loop()
  bool hasNew() const { return _newSample; }

  float rpmL() const { return _rpmL; }
  float rpmR() const { return _rpmR; }
  float hzL()  const { return _hzL;  }
  float hzR()  const { return _hzR;  }

  uint32_t pulsesL() const { return _pulsesL; }
  uint32_t pulsesR() const { return _pulsesR; }

  void printDebug(Stream& s = Serial) const;
  void reset();

private:
  Config _cfg{};
  bool _ok = false;

  float _hzL = 0, _hzR = 0;
  float _rpmL = 0, _rpmR = 0;
  uint32_t _pulsesL = 0, _pulsesR = 0;
  bool _newSample = false;
  uint32_t _lastReportMs = 0;

  volatile uint32_t _countL = 0, _countR = 0;
  volatile uint32_t _lastUsL = 0, _lastUsR = 0;

  static void IRAM_ATTR isrL(void* arg);
  static void IRAM_ATTR isrR(void* arg);
};
