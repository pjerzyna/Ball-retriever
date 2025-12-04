#pragma once

// inicjalizacja buforów itd. (kamera jest inicjalizowana w main.cpp)
bool detection_init();

// jedna klatka detekcji + print na Serial/ei_printf
void detection_run_once();
