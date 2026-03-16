#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "ens160.h"

#define TAG "ens160"

// Register map (ENS160 datasheet §9)
#define REG_PART_ID     0x00
#define REG_OPMODE      0x10
#define REG_CONFIG      0x11
#define REG_COMMAND     0x12
#define REG_TEMP_IN     0x13
#define REG_RH_IN       0x15
#define REG_DATA_STATUS 0x20
#define REG_DATA_AQI    0x21
#define REG_DATA_TVOC   0x22
#define REG_DATA_ECO2   0x24

#define OPMODE_DEEP_SLEEP 0x00
#define OPMODE_IDLE       0x01
#define OPMODE_STANDARD   0x02
#define OPMODE_RESET      0xF0

// DATA_STATUS bits (datasheet §10.6)
#define STATUS_NEWDAT     (1 << 1)  // new data available in DATA_x registers
#define STATUS_VALIDITY   (0x0C)    // validity flag bits[3:2]
#define STATUS_NORMAL     (0x00)    // 0b00: normal operation
#define STATUS_WARMUP     (0x04)    // 0b01: warm-up phase
#define STATUS_INITIAL    (0x08)    // 0b10: initial start-up phase
#define STATUS_INVALID    (0x0C)    // 0b11: invalid output

static esp_err_t ens160_write_reg(i2c_master_dev_handle_t handle, uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[len + 1];
    buf[0] = reg;
    for (size_t i = 0; i < len; i++) buf[i + 1] = data[i];
    return i2c_master_transmit(handle, buf, len + 1, 100);
}

static esp_err_t ens160_read_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(handle, &reg, 1, data, len, 100);
}

esp_err_t ens160_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *out_handle)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = ENS160_I2C_ADDR,
        .scl_speed_hz    = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &cfg, out_handle), TAG, "add device failed");

    // Reset to power-on state: write RESET opmode, wait 10ms, then go to IDLE
    uint8_t mode = OPMODE_RESET;
    ESP_RETURN_ON_ERROR(ens160_write_reg(*out_handle, REG_OPMODE, &mode, 1), TAG, "reset failed");
    vTaskDelay(pdMS_TO_TICKS(ENS160_RESET_DELAY_MS));

    mode = OPMODE_IDLE;
    ESP_RETURN_ON_ERROR(ens160_write_reg(*out_handle, REG_OPMODE, &mode, 1), TAG, "set idle failed");

    // Verify part ID after reset (should be 0x0160)
    uint8_t id_buf[2];
    ESP_RETURN_ON_ERROR(ens160_read_reg(*out_handle, REG_PART_ID, id_buf, 2), TAG, "read part id failed");
    uint16_t part_id = (uint16_t)(id_buf[1] << 8 | id_buf[0]);
    if (part_id != 0x0160) {
        ESP_LOGE(TAG, "unexpected part id: 0x%04x", part_id);
        return ESP_ERR_NOT_FOUND;
    }

    // Switch to standard measurement mode (1 second per sample)
    mode = OPMODE_STANDARD;
    ESP_RETURN_ON_ERROR(ens160_write_reg(*out_handle, REG_OPMODE, &mode, 1), TAG, "set standard failed");

    // Allow sensor to stabilize (first readings may be in "warm-up" state)
    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}

esp_err_t ens160_set_temp_hum(i2c_master_dev_handle_t handle, float temperature, float humidity)
{
    // ENS160 expects temperature as (T_celsius + 273.15) * 64, humidity as RH * 512, both uint16_t LE
    // Registers TEMP_IN (0x13) and RH_IN (0x15) are consecutive — write all 4 bytes in one transaction
    uint16_t t_raw = (uint16_t)((temperature + 273.15f) * 64.0f);
    uint16_t h_raw = (uint16_t)(humidity * 512.0f);

    uint8_t buf[4] = {
        t_raw & 0xFF, (t_raw >> 8) & 0xFF,
        h_raw & 0xFF, (h_raw >> 8) & 0xFF,
    };
    return ens160_write_reg(handle, REG_TEMP_IN, buf, 4);
}

esp_err_t ens160_read(i2c_master_dev_handle_t handle, ens160_data_t *out)
{
    uint8_t status;
    ESP_RETURN_ON_ERROR(ens160_read_reg(handle, REG_DATA_STATUS, &status, 1), TAG, "read status failed");

    ESP_LOGD(TAG, "DATA_STATUS=0x%02x", status);

    // NEWDAT is the only gate for reading — Validity is informational only.
    // The sensor outputs data during Warm-Up and Initial Start-Up phases too;
    // blocking on Validity causes permanent stall after the first warm-up cycle.
    if (!(status & STATUS_NEWDAT)) {
        return ESP_ERR_NOT_FINISHED;
    }

    uint8_t validity = status & STATUS_VALIDITY;
    if (validity != STATUS_NORMAL) {
        const char *phase = (validity == STATUS_WARMUP)  ? "warm-up"  :
                            (validity == STATUS_INITIAL) ? "initial start-up" : "invalid";
        ESP_LOGW(TAG, "validity: %s, skipping data", phase);
        return ESP_ERR_INVALID_STATE;
    }

    // Read all data registers in one transaction (0x21..0x27, 7 bytes)
    // matching the official driver's Ens16x_Update() which reads dataBuffer[7] at once.
    // Splitting into multiple reads risks clearing NEWDAT mid-transaction.
    uint8_t buf[7];
    ESP_RETURN_ON_ERROR(ens160_read_reg(handle, REG_DATA_AQI, buf, 7), TAG, "read data failed");

    // buf offsets relative to REG_DATA_AQI (0x21):
    // [0]=AQI, [1..2]=TVOC LE, [3..4]=ECO2 LE, [5]=AQI_S, [6]=T_OUT
    out->aqi  = buf[0] & 0x07;
    out->tvoc = (uint16_t)(buf[2] << 8 | buf[1]);
    out->eco2 = (uint16_t)(buf[4] << 8 | buf[3]);

    if (out->eco2 == 0 && out->tvoc == 0 && out->aqi == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}
