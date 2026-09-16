/**
 * @file main.c
 * @brief Runs the built-in self test of the BME690 sensor and prints the
 *        result
 *
 * The sensor is left in sleep mode afterwards, so it has to be reconfigured
 * before it is used again.
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soldered_bme690.h"

static const char *TAG = "BME690_SELF_TEST";

// Change these to match how your breakout is wired
#define PIN_NUM_SDA GPIO_NUM_21
#define PIN_NUM_SCL GPIO_NUM_22

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

    esp_err_t err = bme690_init(&sensor, bus, BME69X_I2C_ADDR_LOW);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BME690 initialization failed: %s (%s)", esp_err_to_name(err), bme690_status_string(&sensor));
        return;
    }

    ESP_LOGI(TAG, "Unique ID: 0x%08lX", (unsigned long)bme690_get_unique_id(&sensor));

    ESP_LOGI(TAG, "Running the self test, this takes a few seconds...");

    if (bme690_self_test(&sensor) == BME69X_OK) {
        ESP_LOGI(TAG, "Self test passed.");
    } else {
        ESP_LOGE(TAG, "Self test failed: %s", bme690_status_string(&sensor));
    }
}
