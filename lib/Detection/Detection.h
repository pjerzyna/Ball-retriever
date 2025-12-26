#pragma once

#include <Arduino.h>

// A simple class that handles FOMO detection with Edge Impulse
class Detection {
public:
    struct Config {
        bool     debug_nn           = false;    // passed to run_classifier (internal EI debug)
        bool     log                = false;
        uint32_t periodMs           = 150;      // every how many ms to do detection in update(); 0 means every time
        float    confidenceThreshold = 0.6f;    // the threshold from which the result is considered valid
    };

    // High-level result - extracted from ei_impulse_result_t
    struct Result {
        bool     valid     = false;  // Is there any detection above the threshold
        float    score     = 0.0f;   // certainty (0..1)
        uint16_t x         = 0;      // bbox coordinates (in EI input pixels, 96x96)
        uint16_t y         = 0;
        uint16_t width     = 0;
        uint16_t height    = 0;
    };

    Detection() = default;

    // initialization, the camera is already configured in main.cpp
    bool begin(const Config& cfg);

    // internally keeps track of periodMs (if > 0)
    void update();

    // forces immediate execution of one detection (ignores periodMs)
    // returns true if the inference was successful (not necessarily that something was detected)
    bool runOnce();

    // access to the latest result
    bool   hasResult()   const { return _hasResult; }
    Result lastResult()  const { return _lastResult; }

private:
    // pipeline: frame grab + EI + best bbox extraction
    bool runInferenceOnce(Result& out);

    // callback for Edge Impulse (static, without this)
    static int eiCameraGetData(size_t offset, size_t length, float *out_ptr);

    Config   _cfg{};
    bool     _initialized = false;
    bool     _hasResult   = false;
    Result   _lastResult{};
    uint32_t _lastRunMs   = 0;
};
