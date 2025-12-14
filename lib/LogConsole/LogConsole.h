#pragma once
#include <Arduino.h>
class TelemetryLogger;

class LogConsole {
public:
  struct Config {
    int btnPin = 0;
    bool useButton = true;
    uint32_t debounceMs = 30;   // <-- dodane
  };

  void begin(const Config& cfg, TelemetryLogger& logger, Stream& io);
  void update();

private:
  Config _cfg{};
  TelemetryLogger* _logger = nullptr;
  Stream* _io = nullptr;

  bool _lastRawBtn = true;      // <-- zmiana nazwy (czytelniej)
  bool _stableBtn  = true;      // <-- dodane
  uint32_t _lastDebounceMs = 0; // <-- dodane
};
