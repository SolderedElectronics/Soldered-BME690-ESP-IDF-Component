/**
 * @file main.c
 * @brief Runs the BME690 in sequential mode, where the sensor steps through a
 *        heater profile on its own, sleeping between the measurements
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

static const char *TAG = "BME690_SEQUENTIAL";

// Change these to match how your breakout is wired
#define PIN_NUM_SDA GPIO_NUM_21
#define PIN_NUM_SCL GPIO_NUM_22

#define HEATER_LEN 10

// Heater temperature profile in degrees Celsius. It stays in scope for the
// whole run, the sensor API only keeps a pointer to it.
static uint16_t heater_temp[HEATER_LEN] = {320, 100, 100, 100, 200, 200, 200, 320, 320, 320};

// Heating duration profile in milliseconds
static uint16_t heater_dur[HEATER_LEN] = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150};

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

    bme690_set_tph_default(&sensor);

    // Sleep duration between two measurements of the profile
    bme690_set_seq_sleep(&sensor, BME69X_ODR_250_MS);

    bme690_set_heater_prof_seq(&sensor, heater_temp, heater_dur, HEATER_LEN);
    bme690_set_op_mode(&sensor, BME69X_SEQUENTIAL_MODE);

    if (bme690_check_status(&sensor) == BME690_ERROR) {
        ESP_LOGE(TAG, "BME690 configuration failed: %s", bme690_status_string(&sensor));
        return;
    }

    ESP_LOGI(TAG,
             "Timestamp(ms), Temperature(C), Pressure(Pa), Humidity(%%), Gas resistance(Ohm), Status, Gas index");

    while (1) {
        bme690_data_t data;
        uint8_t n_fields_left = 0;

        if (bme690_fetch_data(&sensor)) {
            do {
                n_fields_left = bme690_get_data(&sensor, &data);

                ESP_LOGI(TAG, "%lld, %.2f, %.2f, %.2f, %.2f, 0x%02X, %u", esp_timer_get_time() / 1000,
                         data.temperature, data.pressure, data.humidity, data.gas_resistance, data.status,
                         data.gas_index);
            } while (n_fields_left);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
