#pragma once
#include <Arduino.h>
#include <LittleFS.h>

class TelemetryLogger {
public:
  struct Sample {
    uint32_t tMs;
    float ax_g, ay_g, az_g;
    float gz_dps;
    uint32_t pL, pR;
    float vL, vR;
  };

  struct Config {
    uint32_t periodMs = 50;
    size_t   maxSamples = 1000;
    bool     autoStart = true;
  };

  bool begin(const Config& cfg);
  void start();
  void stop();
  bool isActive() const;

  // dodajesz próbkę “z zewnątrz” (main czyta imu/enc i pakuje)
  void tick(uint32_t nowMs,
            float ax_g, float ay_g, float az_g,
            float gz_dps,
            uint32_t pL, uint32_t pR,
            float vL, float vR);

  // operacje na plikach
  bool saveToFlash();
  void listLogs(Stream& out);
  void dumpAllLogs(Stream& out);
  void eraseAllLogs(Stream& out);

  // diagnostyka
  size_t samplesCount() const { return _idx; }
  size_t capacity()     const { return _max; }
  
  const Sample& sampleAt(size_t i) const { return _buf[i]; }

private:
  String makeLogPath();

  Config _cfg{};
  Sample* _buf = nullptr;
  size_t  _max = 0;
  size_t  _idx = 0;

  bool     _fsOk = false; //true = do not format automatically 
  bool     _active = false;
  uint32_t _lastMs = 0;
};
