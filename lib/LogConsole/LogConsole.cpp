#include "LogConsole.h"
#include "TelemetryLogger.h"

void LogConsole::begin(const Config& cfg, TelemetryLogger& logger, Stream& io) {
  _cfg = cfg;
  _logger = &logger;
  _io = &io;

  if (_cfg.useButton) {
    pinMode(_cfg.btnPin, INPUT_PULLUP);
    _lastRawBtn = digitalRead(_cfg.btnPin);
    // _stableBtn  = _lastRawBtn;
    // _lastDebounceMs = millis();
  }
}

void LogConsole::update() {
  if (!_logger || !_io) return;

  // =========================
  // Button: (edge + debounce) - system better works without this mechanism
  // =========================
  // if (_cfg.useButton) {
  //   bool raw = digitalRead(_cfg.btnPin);

  //   if (raw != _lastRawBtn) {
  //     _lastRawBtn = raw;
  //     _lastDebounceMs = millis();
  //   }

  //   if ((millis() - _lastDebounceMs) >= _cfg.debounceMs) {
  //     if (raw != _stableBtn) {
  //       _stableBtn = raw;

  //       // falling edge => click
  //       if (_stableBtn == LOW) {
  //         _logger->stop();
  //         //_io->println("BTN: save log");
  //         bool ok = _logger->saveToFlash();
  //         _io->println(ok ? "OK" : "ERR");
  //       }
  //     }
  //   }
  // }

  // =========================
  // Button: simple edge (no debounce)
  // =========================
  if (_cfg.useButton) {
    bool raw = digitalRead(_cfg.btnPin);

    // falling edge: HIGH -> LOW
    if (_lastRawBtn == HIGH && raw == LOW) {
      _logger->stop();
      bool ok = _logger->saveToFlash();
    }

    _lastRawBtn = raw;
  }


  // =========================
  // Serial commands
  // =========================
  while (_io->available()) {
    char c = (char)_io->read();

    // Serial Monitor could send CR/LF
    if (c == '\n' || c == '\r') continue;

    switch (c) {
      case 'A':     // dump all logs
        _logger->dumpAllLogs(*_io);
        break;

      case 'x':     // erase logs
        _logger->eraseAllLogs(*_io);
        break;

      case 'l':     // list logs
        _logger->listLogs(*_io);
        break;

      case 's': {   // (not automatic) save RAM -> Flash
        _logger->stop();
        bool ok = _logger->saveToFlash();
        _io->println(ok ? "OK" : "ERR");
      } break;

      case 'h':     // help with commands
        _io->println("cmds: A=dump, x=erase, l=list, s=save, h=help");
        break;

      default:
        // ignoruj
        break;
    }
  }
}
