#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENS160_I2C_ADDR       0x53
#define ENS160_RESET_DELAY_MS 10   // datasheet §11: 10ms after OPMODE_RESET

/** Air Quality Index (1=Excellent, 2=Good, 3=Moderate, 4=Poor, 5=Unhealthy) */
typedef struct {
    uint16_t eco2;   // Equivalent CO2 in ppm
    uint16_t tvoc;   // Total VOC in ppb
    uint8_t  aqi;    // AQI-UBA (1–5)
} ens160_data_t;

esp_err_t ens160_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *out_handle);
esp_err_t ens160_set_temp_hum(i2c_master_dev_handle_t handle, float temperature, float humidity);
esp_err_t ens160_read(i2c_master_dev_handle_t handle, ens160_data_t *out);

#ifdef __cplusplus
}
#endif
