#pragma once
#include <Arduino.h>

class Motors;
class Detection;

class Fsm {
public:
  enum class State : uint8_t { IDLE, CHASE };

  struct Config {
    // detekcja / przejścia
    float    minConfidence = 0.4f;
    uint32_t holdMs        = 300;

    // parametry obrazu / kalibracja
    int   nnWidth       = 96;
    float calibCenterX  = 52.0f;
    uint16_t targetWidth = 20;

    // PD
    float kpTurn   = 35.0f;
    float kdErr    = 8.0f;
    float deadBand = 0.14f;
    float alphaErr = 0.6f;
    float alphaD   = 0.4f;

    // jazda
    int driveSpeed     = 120;
    int driveBaseSpeed = 80;
    int driveMaxSpeed  = 200;

    // debug
    bool printTransitions = true;
    uint32_t dbgPeriodMs  = 200;
  };

  Fsm() = default;

  void begin(Motors& motors, Detection& detection, const Config& cfg);
  void setState(State s);
  State state() const { return _state; }

  // Wywołuj w loop() po detection.update()
  void update(uint32_t now);

private:
  void stepIdle(uint32_t now);
  void stepChase(uint32_t now);

  void transition(State next, uint32_t now);

private:
  Motors* _motors = nullptr;
  Detection* _det = nullptr;
  Config _cfg;

  State _state = State::IDLE;
  State _prev  = (State)255;

  // CHASE runtime
  uint32_t _lastSeenMs = 0;
  bool     _haveTarget = false;

  float _errFilt  = 0.0f;
  float _prevErr  = 0.0f;
  float _derrFilt = 0.0f;

  uint32_t _lastDbgMs = 0;
};
