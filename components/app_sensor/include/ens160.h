#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** ENS160 I2C address (ADDR pin low) */
#define ENS160_I2C_ADDR       0x53
/** Datasheet §11: minimum delay after writing OPMODE_RESET before next I2C access */
#define ENS160_RESET_DELAY_MS 10

/**
 * Operating modes written to REG_OPMODE (0x10).
 *
 * DEEP_SLEEP  — lowest power; algorithm state lost on wake, avoid for periodic sampling.
 * IDLE        — hotplate off, algorithm state preserved; use between samples to cut self-heating.
 * STANDARD    — normal 1 Hz measurement cycle; must be active for at least 1 s before reading.
 * RESET       — software reset; transitions sensor back to power-on state.
 */
#define OPMODE_DEEP_SLEEP 0x00
#define OPMODE_IDLE       0x01
#define OPMODE_STANDARD   0x02
#define OPMODE_RESET      0xF0

/** Measurement output from ENS160. */
typedef struct {
    uint16_t eco2;   // Equivalent CO2 in ppm  (400–65000)
    uint16_t tvoc;   // Total VOC in ppb        (0–65000)
    uint8_t  aqi;    // AQI-UBA index           (1=Excellent … 5=Unhealthy)
} ens160_data_t;

/**
 * Add ENS160 to an existing I2C master bus and bring it to IDLE mode.
 *
 * Performs a software reset, verifies the part ID (0x0160), then leaves the
 * sensor in IDLE so the hotplate is off until the first measurement is needed.
 *
 * @param bus        Initialized I2C master bus handle.
 * @param out_handle Receives the new device handle on success.
 * @return ESP_OK, ESP_ERR_NOT_FOUND (wrong part ID), or an I2C error code.
 */
esp_err_t ens160_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *out_handle);

/**
 * Write ambient temperature and relative humidity for internal compensation.
 *
 * Should be called with fresh AHT21 readings before every ens160_read() to
 * improve eCO2/TVOC accuracy. Writes REG_TEMP_IN and REG_RH_IN in one transaction.
 *
 * @param handle      Device handle from ens160_init().
 * @param temperature Ambient temperature in °C.
 * @param humidity    Relative humidity in %RH.
 */
esp_err_t ens160_set_temp_hum(i2c_master_dev_handle_t handle, float temperature, float humidity);

/**
 * Read the latest air quality data from the sensor.
 *
 * Checks DATA_STATUS before reading:
 *   - Returns ESP_ERR_NOT_FINISHED if NEWDAT is not set (no fresh sample yet).
 *   - Returns ESP_ERR_INVALID_STATE if the validity flag indicates warm-up or
 *     initial start-up (data fields in @p out are not populated in this case).
 *   - Returns ESP_OK and populates @p out on success.
 *
 * The sensor must be in STANDARD mode for at least 1 s before calling this.
 *
 * @param handle Device handle from ens160_init().
 * @param out    Output buffer; valid only when ESP_OK is returned.
 */
esp_err_t ens160_read(i2c_master_dev_handle_t handle, ens160_data_t *out);

/**
 * Write an operating mode directly to REG_OPMODE.
 *
 * Prefer ens160_init() for initialization. Use this only when you need direct
 * mode control (e.g. custom power management sequences).
 *
 * @param handle Device handle from ens160_init().
 * @param mode   One of the OPMODE_* constants.
 */
esp_err_t ens160_set_opmode(i2c_master_dev_handle_t handle, uint8_t mode);

#ifdef __cplusplus
}
#endif
