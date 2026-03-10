#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AHT21_I2C_ADDR 0x38

typedef struct {
    float temperature; // °C
    float humidity;    // %RH
} aht21_data_t;

esp_err_t aht21_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *out_handle);
esp_err_t aht21_read(i2c_master_dev_handle_t handle, aht21_data_t *out);

#ifdef __cplusplus
}
#endif
