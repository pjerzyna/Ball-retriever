#include "ImuMPU.h"

bool ImuMPU::begin(TwoWire& wire, uint8_t addr0, uint8_t addr1) {
  _wire = &wire;

  _ok = false;
  for (int i=0; i<5 && !_ok; i++) {
    _ok = _mpu.begin(addr0, _wire);
    if (!_ok) _ok = _mpu.begin(addr1, _wire);
    if (!_ok) delay(100);
  }

  if (!_ok) {
    Serial.println(F("ERROR: MPU6050 doesn't respond (0x68/0x69)."));
    return false;
  }

  _mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  _mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrate(600, 600);

  Serial.println(F("OK: MPU6050 ready (acc+gyro)."));
  return true;
}

void ImuMPU::calibrate(uint16_t samples, uint16_t settle_ms) {
  if (!_ok) return;

  delay(settle_ms);

  double sum_ax=0, sum_ay=0, sum_az=0;
  double sum_gx=0, sum_gy=0, sum_gz=0;

  for (uint16_t i=0; i<samples; ++i) {
    sensors_event_t a, g, t;
    _mpu.getEvent(&a, &g, &t);

    double ax_g = a.acceleration.x / ONE_G;
    double ay_g = a.acceleration.y / ONE_G;
    double az_g = a.acceleration.z / ONE_G;

    double gx_dps = g.gyro.x * RAD2DEG;
    double gy_dps = g.gyro.y * RAD2DEG;
    double gz_dps = g.gyro.z * RAD2DEG;

    sum_ax += ax_g; sum_ay += ay_g; sum_az += az_g;
    sum_gx += gx_dps; sum_gy += gy_dps; sum_gz += gz_dps;

    delay(2);
  }

  double mean_ax = sum_ax / samples;
  double mean_ay = sum_ay / samples;
  double mean_az = sum_az / samples;

  double mean_gx = sum_gx / samples;
  double mean_gy = sum_gy / samples;
  double mean_gz = sum_gz / samples;

  _acc_bias_g[0] = (float)mean_ax;
  _acc_bias_g[1] = (float)mean_ay;
  _acc_bias_g[2] = (float)(mean_az - 1.0);

  _gyro_bias_dps[0] = (float)mean_gx;
  _gyro_bias_dps[1] = (float)mean_gy;
  _gyro_bias_dps[2] = (float)mean_gz;

  Serial.println(F("IMU calibrated."));
  Serial.print(F("acc_bias_g = "));  
  Serial.print(_acc_bias_g[0],3); Serial.print(" ");
  Serial.print(_acc_bias_g[1],3); Serial.print(" ");
  Serial.println(_acc_bias_g[2],3);

  Serial.print(F("gyro_bias_dps = ")); 
  Serial.print(_gyro_bias_dps[0],2); Serial.print(" ");
  Serial.print(_gyro_bias_dps[1],2); Serial.print(" ");
  Serial.println(_gyro_bias_dps[2],2);
}

void ImuMPU::update() {
  _newSample = false;
  if (!_ok) return;

  uint32_t now = millis();
  if (now - _lastMs < _periodMs) return;
  _lastMs = now;

  sensors_event_t a, g, t;
  _mpu.getEvent(&a, &g, &t);

  _ax_g = a.acceleration.x / ONE_G - _acc_bias_g[0];
  _ay_g = a.acceleration.y / ONE_G - _acc_bias_g[1];
  _az_g = a.acceleration.z / ONE_G - _acc_bias_g[2];

  _gx_dps = g.gyro.x * RAD2DEG - _gyro_bias_dps[0];
  _gy_dps = g.gyro.y * RAD2DEG - _gyro_bias_dps[1];
  _gz_dps = g.gyro.z * RAD2DEG - _gyro_bias_dps[2];

  _tempC = t.temperature;
  _newSample = true;
}

void ImuMPU::printDebug(Stream& s) const {
  if (!_ok) return;

  s.print(F("[MPU] Acc[g]: "));
  s.print(_ax_g, 3); s.print(' ');
  s.print(_ay_g, 3); s.print(' ');
  s.print(_az_g, 3);

  s.print(F("  |  Gyr[deg/s]: "));
  s.print(_gx_dps, 1); s.print(' ');
  s.print(_gy_dps, 1); s.print(' ');
  s.print(_gz_dps, 1);

  s.print(F("  |  T[C]: "));
  s.print(_tempC, 1);
  s.println();
}
