#include "Motors.h"

bool Motors::begin(const Pins& p, int freqHz, int resolutionBits) {
  _p = p;
  _freq = freqHz;
  _resBits = resolutionBits;
  _pwmMax = (1 << _resBits) - 1;

  pinMode(_p.in1A, OUTPUT);
  pinMode(_p.in2A, OUTPUT);
  pinMode(_p.in1B, OUTPUT);
  pinMode(_p.in2B, OUTPUT);

  ledcSetup(_p.chA, _freq, _resBits);
  ledcSetup(_p.chB, _freq, _resBits);
  ledcAttachPin(_p.pwmA, _p.chA);
  ledcAttachPin(_p.pwmB, _p.chB);

  stop(false);
  _ok = true;
  return true;
}

void Motors::setOne(int pwmPin, int in1, int in2, int ch, int speed) {
  if (!_ok) return;

  if (speed > _pwmMax) speed = _pwmMax;
  if (speed < -_pwmMax) speed = -_pwmMax;

  if (speed == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(ch, 0);
    return;
  }

  bool rev = (speed < 0);
  int pwm = abs(speed);

  if (rev) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  ledcWrite(ch, pwm);
}

void Motors::setA(int speed) {
  setOne(_p.pwmA, _p.in1A, _p.in2A, _p.chA, speed);
}
void Motors::setB(int speed) {
  setOne(_p.pwmB, _p.in1B, _p.in2B, _p.chB, speed);
}
void Motors::setBoth(int speedA, int speedB) {
  setA(speedA);
  setB(speedB);
}

void Motors::stop(bool brake) {
  if (!_ok) return;

  if (brake) {
    digitalWrite(_p.in1A, HIGH); digitalWrite(_p.in2A, HIGH);
    digitalWrite(_p.in1B, HIGH); digitalWrite(_p.in2B, HIGH);
  } else {
    digitalWrite(_p.in1A, LOW);  digitalWrite(_p.in2A, LOW);
    digitalWrite(_p.in1B, LOW);  digitalWrite(_p.in2B, LOW);
  }

  ledcWrite(_p.chA, 0);
  ledcWrite(_p.chB, 0);
}

// ------------------ TEST FSM ------------------

void Motors::enterState(TestState s) {
  _testState = s;
  _tState = millis();
  _entry = true; // sygnał: świeżo weszliśmy w stan
}

void Motors::startTest(uint16_t testMs, uint16_t stopMs,
                       uint8_t maxPwm, uint8_t midPwm,
                       uint16_t pauseMs) {
  _testMs = testMs;
  _stopMs = stopMs;
  _pauseMs = pauseMs;
  _testMax = maxPwm;
  _testMid = midPwm;

  _testActive = true;

  Serial.println("Rozpoczynanie pełnego testu silników A i B.");
  Serial.println("----------------------------------------");

  enterState(T_B_FWD); // zaczynamy od B do przodu
}

void Motors::updateTest() {
  if (!_testActive) return;

  uint32_t now = millis();
  uint32_t dt = now - _tState;

  switch (_testState) {

    // ---- SILNIK B ----
    case T_B_FWD:
      if (_entry) {
        Serial.println("Silnik B: Kierunek 1, Predkosc MAX");
        setB(+_testMax);
        _entry = false;
      }
      if (dt >= _testMs) enterState(T_B_STOP1);
      break;

    case T_B_STOP1:
      if (_entry) {
        Serial.println("Silnik B: STOP");
        setB(0);
        _entry = false;
      }
      if (dt >= _stopMs) enterState(T_B_REV);
      break;

    case T_B_REV:
      if (_entry) {
        Serial.println("Silnik B: Kierunek 2, Predkosc SREDNIA");
        setB(-_testMid);
        _entry = false;
      }
      if (dt >= _testMs) enterState(T_B_STOP2);
      break;

    case T_B_STOP2:
      if (_entry) {
        Serial.println("Silnik B: STOP (koniec testu)");
        setB(0);
        _entry = false;
      }
      if (dt >= _stopMs) enterState(T_A_FWD);
      break;

    // ---- SILNIK A ----
    case T_A_FWD:
      if (_entry) {
        Serial.println("Silnik A: Kierunek 1, Predkosc MAX");
        setA(+_testMax);
        _entry = false;
      }
      if (dt >= _testMs) enterState(T_A_STOP1);
      break;

    case T_A_STOP1:
      if (_entry) {
        Serial.println("Silnik A: STOP");
        setA(0);
        _entry = false;
      }
      if (dt >= _stopMs) enterState(T_A_REV);
      break;

    case T_A_REV:
      if (_entry) {
        Serial.println("Silnik A: Kierunek 2, Predkosc SREDNIA");
        setA(-_testMid);
        _entry = false;
      }
      if (dt >= _testMs) enterState(T_A_STOP2);
      break;

    case T_A_STOP2:
      if (_entry) {
        Serial.println("Silnik A: STOP (koniec testu)");
        setA(0);
        Serial.println("----------------------------------------");
        Serial.println("Czekanie na powtórzenie cyklu testowego...");
        _entry = false;
      }
      if (dt >= _stopMs) enterState(T_PAUSE);
      break;

    case T_PAUSE:
      if (_entry) {
        // nic nie robimy poza czekaniem
        _entry = false;
      }
      if (dt >= _pauseMs) enterState(T_B_FWD);
      break;

    default:
      _testActive = false;
      enterState(T_IDLE);
      break;
  }
}
