#include "Detection.h"

#include <Tennis-Ball-detection_inferencing.h>
#include "esp_camera.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"

// parametry bufora kamery (zgodnie z Twoją konfiguracją)
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS   240
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS   240
#define EI_CAMERA_FRAME_BYTE_SIZE         3   // RGB888

// duży bufor na pełny obraz RGB888 (po konwersji z RGB565)
// (globalne w tym pliku, nie na stosie)
static uint8_t snapshot_buf[EI_CAMERA_RAW_FRAME_BUFFER_COLS *
                            EI_CAMERA_RAW_FRAME_BUFFER_ROWS *
                            EI_CAMERA_FRAME_BYTE_SIZE];

// bufor na zeskalowany obraz 96x96 RGB888
static uint8_t resized_buf[EI_CLASSIFIER_INPUT_WIDTH *
                           EI_CLASSIFIER_INPUT_HEIGHT *
                           EI_CAMERA_FRAME_BYTE_SIZE];

// EI będzie stąd czytał dane
int Detection::eiCameraGetData(size_t offset, size_t length, float *out_ptr)
{
    // offset i length są w "pikselach", każdy piksel to 24b: RRGGBB
    size_t pixel_ix    = offset * 3;
    size_t pixels_left = length;
    size_t out_ix      = 0;

    while (pixels_left--) {
        uint8_t r = resized_buf[pixel_ix + 0];
        uint8_t g = resized_buf[pixel_ix + 1];
        uint8_t b = resized_buf[pixel_ix + 2];

        // pakujemy RGB w jedno 24-bitowe słowo (jak w Twojej wersji)
        out_ptr[out_ix] = (float)((r << 16) | (g << 8) | b);

        out_ix++;
        pixel_ix += 3;
    }
    return 0;
}

bool Detection::begin(const Config& cfg)
{
    _cfg         = cfg;
    _initialized = true;
    _hasResult   = false;
    _lastRunMs   = 0;

    // kamera jest inicjalizowana w main.cpp; tutaj mógłbyś dodać sanity-checki
    return true;
}

void Detection::update()
{
    if (!_initialized) {
        return;
    }

    // prosty scheduler na millis() – jeśli ustawiono periodMs
    if (_cfg.periodMs > 0) {
        uint32_t now = millis();
        if ((uint32_t)(now - _lastRunMs) < _cfg.periodMs) {
            return; // jeszcze za wcześnie
        }
        _lastRunMs = now;
    }

    Result r;
    if (runInferenceOnce(r)) {
        _lastResult = r;
        _hasResult  = r.valid;
    }
}

bool Detection::runOnce()
{
    if (!_initialized) {
        return false;
    }

    Result r;
    bool ok = runInferenceOnce(r);
    if (ok) {
        _lastResult = r;
        _hasResult  = r.valid;
    }
    return ok;
}

bool Detection::runInferenceOnce(Result& out)
{
    out = Result{}; // wyzeruj wynik

    // pobierz ramkę z kamery (RGB565 240x240 z Twojej konfiguracji)
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

    // konwersja z RGB565 do RGB888 do dużego bufora snapshot_buf
    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_RGB565, snapshot_buf);

    // ramkę ODDAJEMY jak najszybciej
    esp_camera_fb_return(fb);

    if (!converted) {
        ei_printf("Conversion to RGB888 failed\n");
        return false;
    }

    // przeskalowanie z 240x240 do 96x96
    ei::image::processing::crop_and_interpolate_rgb888(
        snapshot_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        resized_buf,
        EI_CLASSIFIER_INPUT_WIDTH,
        EI_CLASSIFIER_INPUT_HEIGHT);

    // sygnał dla EI (będzie czytał przez eiCameraGetData)
    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data     = &Detection::eiCameraGetData;

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, _cfg.debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return false;
    }
    if (_cfg.log) {
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);
    }

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    if (_cfg.log) {
    ei_printf("Object detection bounding boxes:\r\n");
    }

    float bestScore = _cfg.confidenceThreshold;
    bool  found     = false;
    uint16_t bestX = 0, bestY = 0, bestW = 0, bestH = 0;

    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        if (1) { //_cfg.log
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                  bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
        }

        if (bb.value >= bestScore) {
            bestScore = bb.value;
            found     = true;
            bestX     = bb.x;
            bestY     = bb.y;
            bestW     = bb.width;
            bestH     = bb.height;
        }
    }

    if (found) {
        out.valid  = true;
        out.score  = bestScore;
        out.x      = bestX;
        out.y      = bestY;
        out.width  = bestW;
        out.height = bestH;
    }

#else   // klasyfikacja 1D, gdybyś kiedyś zmienił model

    ei_printf("Predictions:\r\n");
    float bestScore = _cfg.confidenceThreshold;
    bool  found     = false;

    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        float v = result.classification[i].value;
        const char *label = ei_classifier_inferencing_categories[i];

        ei_printf("  %s: %.5f\r\n", label, v);

        if (v >= bestScore) {
            bestScore = v;
            found     = true;
        }
    }

    if (found) {
        out.valid = true;
        out.score = bestScore;
        // współrzędne zostają 0 – w klasyfikacji nie mają sensu
    }

#endif

#if EI_CLASSIFIER_HAS_ANOMALY
    if (_cfg.log) {
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
    }
#endif

    return true;
}
