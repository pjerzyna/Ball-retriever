#pragma once
#include <Arduino.h>
#include <Wire.h>

// Name collision: 'sensor_t' (Adafruit vs esp_camera)
// The name change is only made locally at the time of including
#define sensor_t adafruit_sensor_t
#include <Adafruit_MPU6050.h>
#undef sensor_t

class ImuMPU {
public:
  // Default addresses of MPU (AD0=0x68 or 0x69)
  bool begin(TwoWire& wire, uint8_t addr0 = 0x68, uint8_t addr1 = 0x69);

  void update();

  bool ok() const { return _ok; }

  void setPeriodMs(uint32_t p) { _periodMs = p; }

  // if there was a new measurement recently
  bool hasNew() const { return _newSample; }

  // values after adding the bias:
  // accelerometer [g]
  float ax_g() const { return _ax_g; }
  float ay_g() const { return _ay_g; }
  float az_g() const { return _az_g; }

  // gyroscope [deg/s]
  float gx_dps() const { return _gx_dps; }
  float gy_dps() const { return _gy_dps; }
  float gz_dps() const { return _gz_dps; }

  float temperature_C() const { return _tempC; }

  // bias exposure (e.g. for logs)
  const float* accBiasG() const { return _acc_bias_g; }
  const float* gyroBiasDps() const { return _gyro_bias_dps; }

  // !!! Callibration is very important to get clear results !!!
  void calibrate(uint16_t samples = 600, uint16_t settle_ms = 600);

  void printDebug(Stream& s = Serial) const;

private:
  TwoWire* _wire = nullptr;
  Adafruit_MPU6050 _mpu;
  bool _ok = false;

  uint32_t _periodMs = 100; // 10 Hz
  uint32_t _lastMs = 0;
  bool _newSample = false;

  // bias table
  float _acc_bias_g[3]  = {0,0,0};
  float _gyro_bias_dps[3] = {0,0,0};

  // final samples 
  float _ax_g=0, _ay_g=0, _az_g=0;
  float _gx_dps=0, _gy_dps=0, _gz_dps=0;
  float _tempC=0;

  static constexpr float RAD2DEG = 57.2957795f; // 180/pi
  static constexpr float ONE_G   = 9.80665f;
};
