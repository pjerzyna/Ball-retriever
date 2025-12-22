#pragma once
#include <Arduino.h>

class Motors;
class Detection;
class TelemetryLogger;

class Fsm {
public:
  enum class State : uint8_t { IDLE, CHASE, CAPTURED, BACK, HOME };

  struct Config {
    // detekcja / przejścia
    float    minConfidence = 0.4f;
    uint32_t holdMs        = 300;

    // parametry obrazu / kalibracja
    int      nnWidth       = 96;
    float    calibCenterX  = 52.0f;
    uint16_t targetWidth   = 20;

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
    bool     printTransitions = true;
    uint32_t dbgPeriodMs      = 200;

    // IDLE -> CAPTURED (po czasie)
    uint32_t idleToCapturedMs = 1200;   // np. 1.2s w IDLE => CAPTURED
    bool     idleCapturedRequiresChase = true; // bezpiecznik

    // BACK (odtwarzanie)
    uint32_t backMaxMs = 8000;     // bezpiecznik: maksymalny czas powrotu
    float    backSpeedScale = 0.3f; // skalowanie prędkości z logu (np. 0.9 jeśli za szybko)
    float    backMinAbsSpeed = 0.0f; // jeśli chcesz "martwą strefę" na bardzo małe prędkości
    uint32_t periodMs = 50;         // okres logowania w ms
    int backMaxPwm = 200;
    int backDeadbandPwm = 10;

    // BACK speed control (m/s -> PWM)
    float backKp = 1500.0f;      // PWM per (m/s) - startowo
    float backKi = 0.0f;         // jeśli chcesz PI (np. 200..600)
    int   backPwmMax = 200;      // ogranicz, żeby nie wystrzeliło
    int   backPwmMinMove = 80;   // minimalny PWM żeby ruszyć (tarcie statyczne)
    float backStopEps_mps = 0.02f; // poniżej tej prędkości uznajemy 0


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
  bool     _armedCaptured = false;   // ustawiane po "realnym" CHASE

  // BACK state
  TelemetryLogger* _logger = nullptr;

  int32_t  _backIdx = -1;          // indeks aktualnej próbki (idziemy w dół)
  uint32_t _backNextMs = 0;        // kiedy przejść do kolejnej próbki
  uint32_t _backStartedMs = 0;     // czas wejścia do BACK (bezpiecznik)
  bool     _wasChasing = false;    // żeby nie wchodzić w BACK bez "realnego" CHASE
  bool _backActive = false;
  float _measVL = 0.0f, _measVR = 0.0f; // m/s z enkoderów
  float _iErrL = 0.0f, _iErrR = 0.0f;   // całka (opcjonalnie)
};