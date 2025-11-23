#pragma once
#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"   // fmt2rgb888

class VisionBall {
public:
  struct HsvThresh {
    int hMin = 22;  // OpenCV scale: H 0..179
    int hMax = 60;
    int sMin = 18;  // 0..255
    int vMin = 24;  // 0..255
  };

  struct Params {
    int sampleStride = 6;        // co ile pikseli próbkujemy
    int periodMs = 100;          // ~10 Hz
    HsvThresh hsv;
  };

  // wersja domyślna
  bool begin();
  // wersja z parametrami
  bool begin(const Params& p);

  void update();                 // wołaj w tasku/loopie

  bool hasNew() const { return _newFrame; }
  bool seen() const { return _seen; }
  float areaRatio() const { return _area; }
  float cxNorm() const { return _cxNorm; }
  int width() const { return _W; }
  int height() const { return _H; }

private:
  Params _p;
  bool _newFrame = false;
  bool _seen = false;
  float _area = 0.0f;
  float _cxNorm = 0.0f;

  uint32_t _lastMs = 0;

  uint8_t* _rgb = nullptr;
  int _W = 0, _H = 0;
  void ensureBuf(int W, int H);

  static inline void rgbToHsvOcv(uint8_t r, uint8_t g, uint8_t b, int& H, int& S, int& V);
};
