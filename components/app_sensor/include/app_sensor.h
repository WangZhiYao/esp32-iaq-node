#pragma once

#include "esp_err.h"

typedef struct {
    float    temperature; // °C
    float    humidity;    // %RH
    uint16_t eco2;        // ppm
    uint16_t tvoc;        // ppb
    uint8_t  aqi;         // 1–5
} sensor_data_t;

esp_err_t app_sensor_init(void);
esp_err_t app_sensor_read(sensor_data_t *out);
esp_err_t app_sensor_deinit(void);
