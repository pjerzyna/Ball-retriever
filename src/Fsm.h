#pragma once
#include <Arduino.h>

class Motors;
class Detection;
class TelemetryLogger;

class Fsm {
public:
  enum class State : uint8_t { IDLE, CHASE, CAPTURED, RETURN, HOME };

  struct Config {
    // detection / transitions
    float    minConfidence = 0.4f;
    uint32_t holdMs        = 300;

    // image parameters / callibration
    int      nnWidth       = 96;
    float    calibCenterX  = 52.0f;
    uint16_t targetWidth   = 20;

    // PD
    float kpTurn   = 35.0f;
    float kdErr    = 8.0f;
    float deadBand = 0.14f;
    float alphaErr = 0.6f;
    float alphaD   = 0.4f;

    // drive
    int driveSpeed     = 120;
    int driveBaseSpeed = 80;
    int driveMaxSpeed  = 200;

    // debug
    bool     printTransitions = true;
    uint32_t dbgPeriodMs      = 200;

    // IDLE -> CAPTURED (after some time)
    uint32_t idleToCapturedMs = 1200;   // 1.2s IDLE => CAPTURED
    bool     idleCapturedRequiresChase = true; // safety

    // RETURN (reproduction of logged path)
    uint32_t backMaxMs = 8000;     // safety: max return time
    float    backSpeedScale = 0.3f; // velocity scaling from log 
    float    backMinAbsSpeed = 0.0f; // dead band for very small velocities
    uint32_t periodMs = 50;         // logging period (for correct indexing)
    int backMaxPwm = 200;
    int backDeadbandPwm = 10;

    // RETURN speed control (m/s -> PWM)
    float backKp = 1500.0f;        // PWM per (m/s)
    float backKi = 250.0f;         // integral part
    int   backPwmMax = 200;        // max PWM
    int   backPwmMinMove = 80;     // min PWM 
    float backStopEps_mps = 0.02f; // below this speed we consider it zero


  };

  Fsm() = default;

  void begin(Motors& motors, Detection& detection, TelemetryLogger& logger, const Config& cfg);
  void setState(State s);
  State state() const { return _state; }

  void update(uint32_t now);
  void setMeasuredWheelV(float vL_mps, float vR_mps) { _measVL = vL_mps; _measVR = vR_mps; }


private:
  void stepIdle(uint32_t now);
  void stepChase(uint32_t now);
  void stepCaptured(uint32_t now);

  void transition(State next, uint32_t now);

  void stepBack(uint32_t now);

  void stepHome(uint32_t now);

private:
  Motors*    _motors = nullptr;
  Detection* _det    = nullptr;
  Config     _cfg;

  State _state = State::IDLE;
  State _prev  = (State)255;

  // CHASE runtime
  uint32_t _lastSeenMs  = 0;
  bool     _haveTarget  = false;

  float _errFilt  = 0.0f;
  float _prevErr  = 0.0f;
  float _derrFilt = 0.0f;

  uint32_t _lastDbgMs = 0;

  // IDLE timer + "armed" flag
  uint32_t _idleSinceMs = 0;
  bool     _armedCaptured = false;   // setting after "real" CHASE

  // BACK state
  TelemetryLogger* _logger = nullptr;

  int32_t  _backIdx = -1;          // actual sample index (we are counting down)
  uint32_t _backNextMs = 0;        // when next sample should be processed
  uint32_t _backStartedMs = 0;     // time of starting BACK
  bool     _wasChasing = false;    // in case we never chased, do not BACK
  bool _backActive = false;
  float _measVL = 0.0f, _measVR = 0.0f; // m/s 
  float _iErrL = 0.0f, _iErrR = 0.0f;   // integral 
};