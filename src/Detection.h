#pragma once

#include <Arduino.h>

// Prosta klasa obsługująca detekcję FOMO z Edge Impulse
class Detection {
public:
    struct Config {
        bool     debug_nn           = false;  // przekazywane do run_classifier (wewnętrzny debug EI)
        uint32_t periodMs           = 0;      // co ile ms robić detekcję w update(); 0 = za każdym razem
        float    confidenceThreshold = 0.5f;  // próg pewności, od którego wynik uznajemy za ważny
    };

    // Wynik „wysokopoziomowy” – wyciągnięty z ei_impulse_result_t
    struct Result {
        bool     valid     = false;  // czy jest jakaś detekcja powyżej progu
        float    score     = 0.0f;   // pewność (0..1)
        uint16_t x         = 0;      // współrzędne bboxa (w pikselach wejścia EI, np. 96x96)
        uint16_t y         = 0;
        uint16_t width     = 0;
        uint16_t height    = 0;
    };

    Detection() = default;

    // inicjalizacja – zakładamy, że kamera jest już skonfigurowana w main.cpp
    bool begin(const Config& cfg);

    // wołasz w loop() – wewnętrznie pilnuje periodMs (jeśli > 0)
    void update();

    // wymusza natychmiastowe wykonanie jednej detekcji (ignoruje periodMs)
    // zwraca true, jeśli inference się udał (niekoniecznie, że coś wykryto)
    bool runOnce();

    // dostęp do ostatniego wyniku
    bool   hasResult()   const { return _hasResult; }
    Result lastResult()  const { return _lastResult; }

private:
    // właściwy pipeline: pobranie klatki + EI + wyciągnięcie najlepszego bboxa
    bool runInferenceOnce(Result& out);

    // callback dla Edge Impulse (statyczny, bez this)
    static int eiCameraGetData(size_t offset, size_t length, float *out_ptr);

    Config   _cfg{};
    bool     _initialized = false;
    bool     _hasResult   = false;
    Result   _lastResult{};
    uint32_t _lastRunMs   = 0;
};
