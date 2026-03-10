#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "app_sensor.h"
#include "aht21.h"
#include "ens160.h"

#define TAG "app_sensor"

#define I2C_PORT    I2C_NUM_0
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_aht21;
static i2c_master_dev_handle_t s_ens160;

esp_err_t app_sensor_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port            = I2C_PORT,
        .sda_io_num          = I2C_SDA_PIN,
        .scl_io_num          = I2C_SCL_PIN,
        .clk_source          = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt   = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus), TAG, "bus init failed");

    ESP_RETURN_ON_ERROR(aht21_init(s_bus, &s_aht21),   TAG, "aht21 init failed");
    ESP_RETURN_ON_ERROR(ens160_init(s_bus, &s_ens160), TAG, "ens160 init failed");

    ESP_LOGI(TAG, "sensors ready");
    return ESP_OK;
}

esp_err_t app_sensor_read(sensor_data_t *out)
{
    // Read AHT21 first — temperature/humidity are used to compensate ENS160
    aht21_data_t aht;
    ESP_RETURN_ON_ERROR(aht21_read(s_aht21, &aht), TAG, "aht21 read failed");

    // Feed T/RH compensation into ENS160
    esp_err_t ret = ens160_set_temp_hum(s_ens160, aht.temperature, aht.humidity);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ens160 set_temp_hum: %s", esp_err_to_name(ret));
    }

    ens160_data_t ens;
    ret = ens160_read(s_ens160, &ens);
    if (ret == ESP_ERR_NOT_FINISHED) {
        return ret; // still warming up, caller will retry next cycle
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "ens160 read failed");

    out->temperature = aht.temperature;
    out->humidity    = aht.humidity;
    out->eco2        = ens.eco2;
    out->tvoc        = ens.tvoc;
    out->aqi         = ens.aqi;

    return ESP_OK;
}

esp_err_t app_sensor_deinit(void)
{
    i2c_master_bus_rm_device(s_aht21);
    i2c_master_bus_rm_device(s_ens160);
    i2c_del_master_bus(s_bus);
    return ESP_OK;
}
