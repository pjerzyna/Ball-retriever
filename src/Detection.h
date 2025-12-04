#pragma once

#include <Arduino.h>

class Detection {
public:
    struct Config {
        bool debug_nn = false;  // czy wypisywać debug z run_classifier
        // tu możesz dorzucać kolejne parametry w przyszłości
    };

    Detection() = default;

    // inicjalizacja – zakładamy, że kamera jest już skonfigurowana w main.cpp
    bool begin(const Config& cfg);

    // wywołujesz raz na iterację pętli – robi jedną klatkę detekcji
    void update();

private:
    Config _cfg;
    bool _initialized = false;
};

