#pragma once
#include <Arduino.h>
#include <Wire.h>

// Kolizja nazw: Adafruit ma sensor_t jak esp_camera.
// Robimy lokalny rename tylko na czas includa.
#define sensor_t adafruit_sensor_t
#include <Adafruit_MPU6050.h>
#undef sensor_t

class ImuMPU {
public:
  // Domyślne adresy MPU (AD0=0x68 lub 0x69)
  bool begin(TwoWire& wire, uint8_t addr0 = 0x68, uint8_t addr1 = 0x69);

  // wywołuj często w loop(); zrobi odczyt co _periodMs
  void update();

  bool ok() const { return _ok; }

  // ustaw okres odczytu (ms) np. 50 = 20Hz
  void setPeriodMs(uint32_t p) { _periodMs = p; }

  // czy w ostatnim update był świeży pomiar
  bool hasNew() const { return _newSample; }

  // ostatnie wartości po biasach:
  // akcelerometr w g
  float ax_g() const { return _ax_g; }
  float ay_g() const { return _ay_g; }
  float az_g() const { return _az_g; }

  // żyroskop w deg/s
  float gx_dps() const { return _gx_dps; }
  float gy_dps() const { return _gy_dps; }
  float gz_dps() const { return _gz_dps; }

  float temperature_C() const { return _tempC; }

  // ekspozycja biasów (np. do logów)
  const float* accBiasG() const { return _acc_bias_g; }
  const float* gyroBiasDps() const { return _gyro_bias_dps; }

  // ręczna kalibracja (możesz odpalić np. po starcie)
  void calibrate(uint16_t samples = 600, uint16_t settle_ms = 600);

  // szybki debug jak w Twoim mainie
  void printDebug(Stream& s = Serial) const;

private:
  TwoWire* _wire = nullptr;
  Adafruit_MPU6050 _mpu;
  bool _ok = false;

  uint32_t _periodMs = 100; // 10 Hz
  uint32_t _lastMs = 0;
  bool _newSample = false;

  // biasy
  float _acc_bias_g[3]  = {0,0,0};
  float _gyro_bias_dps[3] = {0,0,0};

  // ostatnie próbki po korekcji
  float _ax_g=0, _ay_g=0, _az_g=0;
  float _gx_dps=0, _gy_dps=0, _gz_dps=0;
  float _tempC=0;

  static constexpr float RAD2DEG = 57.2957795f; // 180/pi
  static constexpr float ONE_G   = 9.80665f;
};
