#include "VisionBall.h"
#include <math.h>

bool VisionBall::begin() {
  Params def;          // użyje domyślnych wartości z structa
  return begin(def);
}

bool VisionBall::begin(const Params& p) {
  _p = p;
  _lastMs = millis();
  return true;
}

void VisionBall::ensureBuf(int W, int H) {
  if (_rgb && _W == W && _H == H) return;
  if (_rgb) { free(_rgb); _rgb = nullptr; }
  _rgb = (uint8_t*) heap_caps_malloc(W * H * 3, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  _W = W; _H = H;
}

inline void VisionBall::rgbToHsvOcv(uint8_t r, uint8_t g, uint8_t b, int& H, int& S, int& V) {
  float rf = r/255.0f, gf = g/255.0f, bf = b/255.0f;
  float cmax = fmaxf(rf, fmaxf(gf, bf));
  float cmin = fminf(rf, fminf(gf, bf));
  float delta = cmax - cmin;

  float h = 0.0f;
  if (delta > 1e-6f) {
    if (cmax == rf)      h = fmodf(((gf - bf)/delta), 6.0f);
    else if (cmax == gf) h = ((bf - rf)/delta) + 2.0f;
    else                 h = ((rf - gf)/delta) + 4.0f;
    h *= 60.0f;
    if (h < 0) h += 360.0f;
  }
  int s = (cmax <= 1e-6f) ? 0 : (int)roundf((delta / cmax) * 255.0f);
  int v = (int)roundf(cmax * 255.0f);

  H = (int)roundf(h / 2.0f); // 0..179
  S = s; V = v;
}

void VisionBall::update() {
  _newFrame = false;
  uint32_t now = millis();
  if (now - _lastMs < (uint32_t)_p.periodMs) return;
  _lastMs = now;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;

  int W = fb->width, H = fb->height;
  ensureBuf(W, H);

  bool ok = _rgb && fmt2rgb888(fb->buf, fb->len, fb->format, _rgb);
  esp_camera_fb_return(fb);
  if (!ok) return;

  uint32_t greenCnt = 0;
  uint64_t sumX = 0;
  uint32_t samples = 0;

  for (int y = 0; y < H; y += _p.sampleStride) {
    int row = y * W;
    for (int x = 0; x < W; x += _p.sampleStride) {
      int idx = (row + x) * 3;
      uint8_t r = _rgb[idx + 0];
      uint8_t g = _rgb[idx + 1];
      uint8_t b = _rgb[idx + 2];

      int h,s,v;
      rgbToHsvOcv(r,g,b,h,s,v);

      if (h >= _p.hsv.hMin && h <= _p.hsv.hMax &&
          s >= _p.hsv.sMin && v >= _p.hsv.vMin) {
        greenCnt++;
        sumX += x;
      }
      samples++;
    }
  }

  _area = (samples > 0) ? (float)greenCnt / (float)samples : 0.0f;
  _seen = (greenCnt > 0);

  if (_seen) {
    float cx = (float)sumX / (float)greenCnt;
    _cxNorm = (cx - (W * 0.5f)) / (W * 0.5f);
    if (_cxNorm < -1) _cxNorm = -1;
    if (_cxNorm >  1) _cxNorm =  1;
  } else {
    _cxNorm = 0.0f;
  }

  _newFrame = true;
}
