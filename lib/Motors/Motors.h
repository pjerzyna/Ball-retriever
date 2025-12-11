#pragma once
#include <Arduino.h>

class Motors {
public:
  struct Pins {
    int pwmA, in1A, in2A;
    int pwmB, in1B, in2B;
    int chA, chB; // LEDC channels
  };

  bool begin(const Pins& p, int freqHz = 20000, int resolutionBits = 8);

  // speed: -255..255 (minus = backward), 0 = stop
  void setA(int speed);
  void setB(int speed);
  void setBoth(int speedA, int speedB);

  void stop(bool brake = false);

  // --- non-blocking test ---
  void startTest(uint16_t testMs = 2000, uint16_t stopMs = 1000,
                 uint8_t maxPwm = 255, uint8_t midPwm = 150,
                 uint16_t pauseMs = 3000);
  void updateTest();  
  bool testRunning() const { return _testActive; }

  void setScale(float scaleA, float scaleB) {
    _scaleA = scaleA;
    _scaleB = scaleB;
  }

private:
  Pins _p{};
  bool _ok = false;

  int _freq = 20000;
  int _resBits = 8;
  int _pwmMax = 255;

  float _scaleA = 1.0f;
  float _scaleB = 1.0f;

  void setOne(int pwmPin, int in1, int in2, int ch, int speed);

  // ---------- TEST FSM ----------
  enum TestState {
    T_IDLE,
    T_B_FWD, T_B_STOP1, T_B_REV, T_B_STOP2,
    T_A_FWD, T_A_STOP1, T_A_REV, T_A_STOP2,
    T_PAUSE
  };

  TestState _testState = T_IDLE;
  bool _testActive = false;

  uint32_t _tState = 0;      // time of entry into the state
  bool _entry = false;       // state entry flag

  uint16_t _testMs = 2000, _stopMs = 1000, _pauseMs = 3000;
  uint8_t _testMax = 255, _testMid = 150;

  void enterState(TestState s);
};
