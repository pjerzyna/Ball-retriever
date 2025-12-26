#pragma once
#include <Arduino.h>
class DataLogger;

class LogConsole {
public:
  struct Config {
    int btnPin = 0;
    bool useButton = true;
    // uint32_t debounceMs = 30;
  };

  void begin(const Config& cfg, DataLogger& logger, Stream& io);
  void update();

private:
  Config _cfg{};
  DataLogger* _logger = nullptr;
  Stream* _io = nullptr;

  bool _lastRawBtn = true;    
  // bool _stableBtn  = true;      
  // uint32_t _lastDebounceMs = 0; 
};
