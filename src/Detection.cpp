#include "Detection.h"
#include <Tennis-Ball-detection_inferencing.h>
#include "esp_camera.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"


#define EI_CAMERA_RAW_FRAME_BUFFER_COLS   240
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS   240
#define EI_CAMERA_FRAME_BYTE_SIZE         3   // RGB888

static bool debug_nn = false;

// duży bufor na pełny obraz RGB888 (po konwersji z RGB565)
static uint8_t snapshot_buf[EI_CAMERA_RAW_FRAME_BUFFER_COLS *
                            EI_CAMERA_RAW_FRAME_BUFFER_ROWS *
                            EI_CAMERA_FRAME_BYTE_SIZE];

// bufor na zeskalowany obraz 96x96 RGB888
static uint8_t resized_buf[EI_CLASSIFIER_INPUT_WIDTH *
                           EI_CLASSIFIER_INPUT_HEIGHT *
                           EI_CAMERA_FRAME_BYTE_SIZE];

// EI będzie stąd czytał dane
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // offset i length są w "pikselach", każdy piksel to 24b: RRGGBB
    size_t pixel_ix    = offset * 3;
    size_t pixels_left = length;
    size_t out_ix      = 0;

    while (pixels_left--) {
        // BGR -> RGB jeśli trzeba, u Ciebie RGB565 -> RGB888 już zrobiliśmy,
        // więc tu po prostu pakujemy R,G,B w 24-bitową liczbę:
        uint8_t r = resized_buf[pixel_ix + 0];
        uint8_t g = resized_buf[pixel_ix + 1];
        uint8_t b = resized_buf[pixel_ix + 2];

        out_ptr[out_ix] = (r << 16) | (g << 8) | b;

        out_ix++;
        pixel_ix += 3;
    }
    return 0;
}

bool detection_init()
{
    // tu nic wielkiego – kamera jest już zrobiona w main.cpp
    // można kiedyś dodać jakieś checki
    return true;
}

void detection_run_once()
{
    // pobierz ramkę z kamery (RGB565 240x240 z Twojej konfiguracji)
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ei_printf("Camera capture failed\n");
        return;
    }

    //Serial.print("fb->width = ");  Serial.println(fb->width);
    //Serial.print("fb->height = "); Serial.println(fb->height);
    //Serial.print("fb->len = ");    Serial.println(fb->len);

    // konwersja z RGB565 do RGB888 do dużego bufora snapshot_buf
    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_RGB565, snapshot_buf);
    esp_camera_fb_return(fb);

    if (!converted) {
        ei_printf("Conversion to RGB888 failed\n");
        return;
    }

    // przeskalowanie z 240x240 do 96x96
    ei::image::processing::crop_and_interpolate_rgb888(
        snapshot_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        resized_buf,
        EI_CLASSIFIER_INPUT_WIDTH,
        EI_CLASSIFIER_INPUT_HEIGHT);

    // sygnał dla EI (będzie czytał przez ei_camera_get_data)
    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    //signal.total_length = EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
    signal.get_data     = &ei_camera_get_data;

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                  bb.label,
                  bb.value,
                  bb.x,
                  bb.y,
                  bb.width,
                  bb.height);
    }
#else
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif
}
