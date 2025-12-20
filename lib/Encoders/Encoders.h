#pragma once
#include <Arduino.h>

class Encoders {
public:
  struct Config {
    uint8_t pinL;
    uint8_t pinR;
    bool usePullup = false;          // LM393 has pull-up
    // Disc has 20 slots, and ISR triggers on FALLING edges.
    // In theory this could give ~20 pulses/rev (one edge per slot),
    // but in practice the opto+LM393 waveform and edge detection produce a different
    // effective count. Calibrated on real wheel rotations during motion.
    float pulsesPerRevL = 22.0f;     // effective pulses per wheel revolution (measured)
    float pulsesPerRevR = 22.0f;  
    uint32_t minPulseUs = 200;       // time filter between impulses 200=0.2ms
    uint32_t reportMs = 50;        

    float wheelRadius_m = 0.032f;  // 0.0125f radius of slot wheel; 0.032 radius of normal wheel with a tire
  };

  bool begin(const Config& cfg);

  void update();             
  bool hasNew() const { return _newSample; }

  float rpmL() const { return _rpmL; }
  float rpmR() const { return _rpmR; }
  float hzL()  const { return _hzL;  }
  float hzR()  const { return _hzR;  }

  float vL() const { return _vL; }   // [m/s]
  float vR() const { return _vR; }   // [m/s]

  uint32_t pulsesL() const { return _pulsesL; }
  uint32_t pulsesR() const { return _pulsesR; }

  uint32_t totalL() const { return _totalL; }
  uint32_t totalR() const { return _totalR; }


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
  volatile uint32_t _totalL = 0, _totalR = 0;

  float _vL = 0.0f, _vR = 0.0f; 

  static void IRAM_ATTR isrL(void* arg);
  static void IRAM_ATTR isrR(void* arg);
};
