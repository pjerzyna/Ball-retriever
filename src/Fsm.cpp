#include "Fsm.h"
#include <Arduino.h>
#include <math.h>

#include <Motors.h>
#include "Detection.h"

void Fsm::begin(Motors& motors, Detection& detection, const Config& cfg) {
  _motors = &motors;
  _det    = &detection;
  _cfg    = cfg;

  _state = State::IDLE;
  _prev  = (State)255;

  _lastSeenMs = 0;
  _haveTarget = false;

  _errFilt  = 0.0f;
  _prevErr  = 0.0f;
  _derrFilt = 0.0f;

  _lastDbgMs = 0;

  _idleSinceMs = 0;
  _armedCaptured = false;
}

void Fsm::setState(State s) {
  _state = s;
  _prev  = (State)255;
}

void Fsm::update(uint32_t now) {
  if (!_motors || !_det) return;

  if (_state != _prev) {
    if (_cfg.printTransitions) {
      const char* name =
        (_state == State::IDLE)     ? "IDLE" :
        (_state == State::CHASE)    ? "CHASE" :
        (_state == State::CAPTURED) ? "CAPTURED" : "?";
      Serial.print("[FSM] -> ");
      Serial.print(name);
      Serial.print("  t=");
      Serial.println(now);
    }

    // "on-enter" akcje stanów
    if (_state == State::IDLE) {
      _idleSinceMs = now;
    }

    _prev = _state;
  }

  switch (_state) {
    case State::IDLE:     stepIdle(now);     break;
    case State::CHASE:    stepChase(now);    break;
    case State::CAPTURED: stepCaptured(now); break;
  }
}

void Fsm::stepIdle(uint32_t now) {
  _motors->stop();

  // IDLE -> CAPTURED po czasie, ale tylko jeśli "uzbrojone" (żeby nie złapało na starcie)
  if (_cfg.idleToCapturedMs > 0) {
    const bool armedOk = (!_cfg.idleCapturedRequiresChase) || _armedCaptured;
    if (armedOk && _idleSinceMs != 0 && (now - _idleSinceMs) >= _cfg.idleToCapturedMs) {
      transition(State::CAPTURED, now);
      return;
    }
  }

  // IDLE -> CHASE, jeśli pojawi się sensowna detekcja
  bool gotFresh = false;
  Detection::Result r;
  if (_det->hasResult()) {
    r = _det->lastResult();
    if (r.valid && r.score >= _cfg.minConfidence) gotFresh = true;
  }

  if (gotFresh) {
    transition(State::CHASE, now);
  }
}

void Fsm::stepChase(uint32_t now) {
  bool gotFresh = false;
  Detection::Result r;

  if (_det->hasResult()) {
    r = _det->lastResult();
    if (r.valid && r.score >= _cfg.minConfidence) gotFresh = true;
  }

  if (gotFresh) {
    _lastSeenMs = now;
    _haveTarget = true;

    // uzbrój CAPTURED, bo realnie goniliśmy cel
    _armedCaptured = true;

    const float imgCenterX   = _cfg.nnWidth / 2.0f;
    const float calibCenterX = _cfg.calibCenterX;

    float cx  = r.x + r.width * 0.5f;
    float err = (cx - calibCenterX) / imgCenterX;
    err = -err;

    _errFilt = _cfg.alphaErr * err + (1.0f - _cfg.alphaErr) * _errFilt;

    float derr = _errFilt - _prevErr;
    _prevErr = _errFilt;
    _derrFilt = _cfg.alphaD * derr + (1.0f - _cfg.alphaD) * _derrFilt;

    float turnF = _cfg.kpTurn * _errFilt + _cfg.kdErr * _derrFilt;
    int turn = (int)turnF;

    int base = _cfg.driveSpeed;
    if (r.width > _cfg.targetWidth) base = _cfg.driveBaseSpeed; // jeśli chcesz, zostaw; jak nie chcesz, wywal

    if (fabs(_errFilt) < _cfg.deadBand) {
      _motors->setBoth(base, base);
    } else {
      int left  = base - turn;
      int right = base + turn;

      if (left  >  _cfg.driveMaxSpeed) left  =  _cfg.driveMaxSpeed;
      if (left  < -_cfg.driveMaxSpeed) left  = -_cfg.driveMaxSpeed;
      if (right >  _cfg.driveMaxSpeed) right =  _cfg.driveMaxSpeed;
      if (right < -_cfg.driveMaxSpeed) right = -_cfg.driveMaxSpeed;

      _motors->setBoth(left, right);
    }

    if (_cfg.dbgPeriodMs && (now - _lastDbgMs > _cfg.dbgPeriodMs)) {
      _lastDbgMs = now;
      Serial.print("[CHASE] score="); Serial.print(r.score, 2);
      Serial.print(" x="); Serial.print(r.x);
      Serial.print(" y="); Serial.print(r.y);
      Serial.print(" w="); Serial.print(r.width);
      Serial.print(" h="); Serial.print(r.height);
      Serial.print(" errFilt="); Serial.print(_errFilt, 3);
      Serial.print(" derrFilt="); Serial.println(_derrFilt, 3);
    }

  } else {
    // brak świeżej detekcji
    if (_haveTarget && (now - _lastSeenMs) < _cfg.holdMs) {
      // trzymamy poprzednie sterowanie
    } else {
      _haveTarget = false;
      _motors->stop();
      transition(State::IDLE, now);
    }
  }
}

void Fsm::stepCaptured(uint32_t now) {
  (void)now;
  _motors->stop(); // terminalnie: nie reaguje na nic
}

void Fsm::transition(State next, uint32_t now) {
  (void)now;
  if (next == _state) return;
  _state = next;
}
