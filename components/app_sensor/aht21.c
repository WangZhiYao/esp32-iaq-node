#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "aht21.h"

#define TAG "aht21"

// AHT21 commands (datasheet §8)
#define CMD_INIT        0xBE
#define CMD_TRIGGER     0xAC
#define CMD_SOFT_RESET  0xBA
#define STATUS_BUSY     0x80
#define STATUS_CALIB    0x08

esp_err_t aht21_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *out_handle)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = AHT21_I2C_ADDR,
        .scl_speed_hz    = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &cfg, out_handle), TAG, "add device failed");

    // Wait for power-on stabilization (datasheet: 40ms after power-on)
    vTaskDelay(pdMS_TO_TICKS(40));

    // Check calibration status; initialize if not calibrated
    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(i2c_master_receive(*out_handle, &status, 1, 100), TAG, "read status failed");

    if (!(status & STATUS_CALIB)) {
        uint8_t init_cmd[3] = {CMD_INIT, 0x08, 0x00};
        ESP_RETURN_ON_ERROR(i2c_master_transmit(*out_handle, init_cmd, 3, 100), TAG, "init cmd failed");
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ESP_OK;
}

esp_err_t aht21_read(i2c_master_dev_handle_t handle, aht21_data_t *out)
{
    // Trigger measurement
    uint8_t trig[3] = {CMD_TRIGGER, 0x33, 0x00};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(handle, trig, 3, 100), TAG, "trigger failed");

    // Wait for measurement to complete (datasheet: 80ms typical)
    vTaskDelay(pdMS_TO_TICKS(80));

    // Poll busy bit
    uint8_t buf[7] = {0};
    for (int i = 0; i < 5; i++) {
        ESP_RETURN_ON_ERROR(i2c_master_receive(handle, buf, 7, 100), TAG, "read failed");
        if (!(buf[0] & STATUS_BUSY)) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (buf[0] & STATUS_BUSY) {
        ESP_LOGE(TAG, "sensor busy timeout");
        return ESP_ERR_TIMEOUT;
    }

    // Humidity: bits[19:0] of bytes 1..3 (upper 20 bits)
    uint32_t raw_hum = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
    // Temperature: bits[19:0] of bytes 3..5 (lower 20 bits)
    uint32_t raw_temp = ((uint32_t)(buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];

    out->humidity    = (float)raw_hum  / 1048576.0f * 100.0f;
    out->temperature = (float)raw_temp / 1048576.0f * 200.0f - 50.0f;

    return ESP_OK;
}
