/**
 * @file main.c
 * @brief Reads temperature, pressure, humidity and gas resistance from the
 *        BME690 sensor in forced mode, one measurement at a time
 *
 * Connect the breakout board to the I2C pins of your board, or use a Qwiic
 * cable.
 *
 * Product used is www.solde.red/333411
 *
 * @author Soldered Electronics
 */

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soldered_bme690.h"

static const char *TAG = "BME690_FORCED";

// Change these to match how your breakout is wired
#define PIN_NUM_SDA GPIO_NUM_21
#define PIN_NUM_SCL GPIO_NUM_22

// Heater temperature in degrees Celsius and heating duration in milliseconds
#define HEATER_TEMP 300
#define HEATER_DUR  100

void app_main(void)
{
    bme690_t sensor;

    // The I2C bus belongs to the application, not to the driver, so that other
    // Qwiic devices can share it. Create it first, then hand it to the driver.
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_NUM_SDA,
        .scl_io_num = PIN_NUM_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    // Start the sensor on the default I2C address (0x76). Use
    // BME69X_I2C_ADDR_HIGH if the address jumper on the board is soldered.
    esp_err_t err = bme690_init(&sensor, bus, BME69X_I2C_ADDR_LOW);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BME690 initialization failed: %s (%s)", esp_err_to_name(err), bme690_status_string(&sensor));
        return;
    }

    // Default oversampling for temperature, pressure and humidity
    bme690_set_tph_default(&sensor);

    // IIR filter for the temperature and pressure readings
    bme690_set_filter(&sensor, BME69X_FILTER_SIZE_3);

    // Heat the gas sensor plate to HEATER_TEMP degrees Celsius for HEATER_DUR
    // milliseconds
    bme690_set_heater_prof(&sensor, HEATER_TEMP, HEATER_DUR);

    if (bme690_check_status(&sensor) == BME690_ERROR) {
        ESP_LOGE(TAG, "BME690 configuration failed: %s", bme690_status_string(&sensor));
        return;
    }

    ESP_LOGI(TAG, "Timestamp(ms), Temperature(C), Pressure(Pa), Humidity(%%), Gas resistance(Ohm), Status");

    while (1) {
        bme690_data_t data;

        // Trigger a single measurement
        bme690_set_op_mode(&sensor, BME69X_FORCED_MODE);

        // Wait for the measurement to finish. bme690_get_meas_dur() only covers
        // the temperature, pressure and humidity part, so the heating duration
        // has to be added on top of it.
        uint32_t meas_dur_us = bme690_get_meas_dur(&sensor, BME69X_FORCED_MODE) + (HEATER_DUR * 1000);
        vTaskDelay(pdMS_TO_TICKS(meas_dur_us / 1000 + 1));

        if (bme690_fetch_data(&sensor)) {
            bme690_get_data(&sensor, &data);

            ESP_LOGI(TAG, "%lld, %.2f, %.2f, %.2f, %.2f, 0x%02X", esp_timer_get_time() / 1000, data.temperature,
                     data.pressure, data.humidity, data.gas_resistance, data.status);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
