#include <Fsm.h>
#include <Arduino.h>
#include <math.h>

#include <Motors.h>
#include <Detection.h>
#include <DataLogger.h>


void Fsm::begin(Motors& motors, Detection& detection, DataLogger& logger, const Config& cfg) {
  _motors = &motors;
  _det    = &detection;
  _logger = &logger;
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

  _backIdx = -1;
  _backNextMs = 0;
  _backStartedMs = 0;
  _wasChasing = false;
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
        (_state == State::CAPTURED) ? "CAPTURED" : 
        (_state == State::RETURN)   ? "RETURN" : 
        (_state == State::HOME)     ? "HOME" : "?";
      Serial.print("[FSM] -> ");
      Serial.print(name);
      Serial.print("  t=");
      Serial.println(now);
    }

    // on-enter actions
    if (_state == State::IDLE) {
      _idleSinceMs = now;
    }

    _prev = _state;
  }

  switch (_state) {
    case State::IDLE:     stepIdle(now);     break;
    case State::CHASE:    stepChase(now);    break;
    case State::CAPTURED: stepCaptured(now); break;
    case State::RETURN:     stepBack(now);     break;
    case State::HOME:     stepHome(now);     break;
  }
}

void Fsm::stepIdle(uint32_t now) {
  _motors->stop();

  // IDLE -> CAPTURED after some time
  if (_cfg.idleToCapturedMs > 0) {
    const bool armedOk = (!_cfg.idleCapturedRequiresChase) || _armedCaptured;
    if (armedOk && _idleSinceMs != 0 && (now - _idleSinceMs) >= _cfg.idleToCapturedMs) {
      transition(State::CAPTURED, now);
      return;
    }
  }

  // IDLE -> CHASE if target seen
  bool gotFresh = false;
  Detection::Result r;
  if (_det->hasResult()) {
    r = _det->lastResult();
    if (r.valid && r.score >= _cfg.minConfidence) gotFresh = true;
  }

  if (gotFresh) {
    transition(State::CHASE, now);
  }

  if (_backActive) {
  _motors->stop();
  return;
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
    _wasChasing = true;      

    // arm CAPTURED, because we had a real CHASE
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

    // doenst make any sense - with this particular camera we have const width
    int base = _cfg.driveSpeed;
    if (r.width > _cfg.targetWidth) base = _cfg.driveBaseSpeed; 

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
    // no fresh target
    if (_haveTarget && (now - _lastSeenMs) < _cfg.holdMs) {
      // keep previous command
    } else {
      _haveTarget = false;
      _motors->stop();
      transition(State::IDLE, now);
    }
  }
}

void Fsm::stepCaptured(uint32_t now) {
  // if CAPTURED
  //_motors->stop();

  // Safety mechanism: do not enter BACK if we never chased
  if (_cfg.idleCapturedRequiresChase && !_wasChasing) {
    transition(State::IDLE, now);
    return;
  }

  // _log->stop();

  // Prepare BACK
  const int32_t n = (int32_t)_logger->samplesCount();
  if (n < 2) { // not enough data
    transition(State::IDLE, now);
    return;
  }

  _backIdx = n - 1;        // start from the last sample
  _backNextMs = now;       // immediately
  _backStartedMs = now;    // time of starting BACK

  _backActive = true; // activate BACK state

  transition(State::RETURN, now);
}


void Fsm::transition(State next, uint32_t now) {
  (void)now;
  if (next == _state) return;
  _state = next;
}

static inline int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline int applyDeadband(int x, int db) {
  if (db <= 0) return x;
  if (x > -db && x < db) return 0;
  return x;
}


void Fsm::stepBack(uint32_t now) {
  if (now - _backStartedMs > _cfg.backMaxMs) {
    _motors->stop(false);
    _backActive = false;
    transition(State::HOME, now); //IDLE
    return;
  }

  if (_backIdx <= 0) {
    _motors->stop(false);
    _backActive = false;
    transition(State::HOME, now); //IDLE
    return;
  }

  if (now < _backNextMs) return;

  const auto& s  = _logger->sampleAt((size_t)_backIdx);
  const auto& sp = _logger->sampleAt((size_t)(_backIdx - 1));

  int32_t dtMs = (int32_t)s.tMs - (int32_t)sp.tMs;
  if (dtMs < 1) dtMs = (int32_t)_cfg.periodMs;
  float dt = dtMs / 1000.0f;

  // Set speeds to drive backwards [m/s]
  float vL_ref = fabsf(s.vL) * _cfg.backSpeedScale;
  float vR_ref = fabsf(s.vR) * _cfg.backSpeedScale;

  // Speed dead zone
  if (fabsf(vL_ref) < _cfg.backStopEps_mps) vL_ref = 0.0f;
  if (fabsf(vR_ref) < _cfg.backStopEps_mps) vR_ref = 0.0f;

  // Speed errors [m/s]
  float eL = vL_ref - _measVL;
  float eR = vR_ref - _measVR;

  // PI
  if (_cfg.backKi > 0.0f) {
    _iErrL += eL * dt;
    _iErrR += eR * dt;

    // anti-windup
    const float iMax = 0.5f; // tune
    if (_iErrL >  iMax) _iErrL =  iMax;
    if (_iErrL < -iMax) _iErrL = -iMax;
    if (_iErrR >  iMax) _iErrR =  iMax;
    if (_iErrR < -iMax) _iErrR = -iMax;
  }

  float uL = _cfg.backKp * eL + _cfg.backKi * _iErrL;
  float uR = _cfg.backKp * eR + _cfg.backKi * _iErrR;

  int pwmL = (int)lroundf(uL);
  int pwmR = (int)lroundf(uR);

  // If we want to drive, but the regulator gives too little PWM -> minimal kick
  if (vL_ref != 0.0f && abs(pwmL) < _cfg.backPwmMinMove) pwmL = (pwmL >= 0) ? _cfg.backPwmMinMove : -_cfg.backPwmMinMove;
  if (vR_ref != 0.0f && abs(pwmR) < _cfg.backPwmMinMove) pwmR = (pwmR >= 0) ? _cfg.backPwmMinMove : -_cfg.backPwmMinMove;

  // Limit
  if (pwmL >  _cfg.backPwmMax) pwmL =  _cfg.backPwmMax;
  if (pwmL < -_cfg.backPwmMax) pwmL = -_cfg.backPwmMax;
  if (pwmR >  _cfg.backPwmMax) pwmR =  _cfg.backPwmMax;
  if (pwmR < -_cfg.backPwmMax) pwmR = -_cfg.backPwmMax;

  _motors->setBoth(pwmL, pwmR);

  // RETURN debug
  if (_cfg.dbgPeriodMs && (now - _lastDbgMs > _cfg.dbgPeriodMs)) {
    _lastDbgMs = now;
    Serial.print("[BACK] idx="); Serial.print(_backIdx);
    Serial.print(" dt="); Serial.print(dtMs);
    Serial.print(" vRefL="); Serial.print(vL_ref, 3);
    Serial.print(" vMeasL="); Serial.print(_measVL, 3);
    Serial.print(" pwmL="); Serial.print(pwmL);
    Serial.print(" | vRefR="); Serial.print(vR_ref, 3);
    Serial.print(" vMeasR="); Serial.print(_measVR, 3);
    Serial.print(" pwmR="); Serial.println(pwmR);
  }

  _backNextMs = now + (uint32_t)dtMs;
  _backIdx--;
}

void Fsm::stepHome(uint32_t now) {
  (void)now;

  // robot in the base: it has to stand there and not react to anything
  _motors->stop(false);
}
