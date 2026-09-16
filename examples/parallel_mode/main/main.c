/**
 * @file main.c
 * @brief Runs the BME690 in parallel mode, where the gas sensor sweeps through
 *        a heater profile while temperature, pressure and humidity are
 *        measured continuously
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

static const char *TAG = "BME690_PARALLEL";

// Change these to match how your breakout is wired
#define PIN_NUM_SDA GPIO_NUM_21
#define PIN_NUM_SCL GPIO_NUM_22

// Duration of one heater profile time base in milliseconds
#define MEAS_DUR 140

#define HEATER_LEN 10

// Heater temperature profile in degrees Celsius. It stays in scope for the
// whole run, the sensor API only keeps a pointer to it.
static uint16_t heater_temp[HEATER_LEN] = {320, 100, 100, 100, 200, 200, 200, 320, 320, 320};

// Multipliers of the shared heater duration, one per profile step
static uint16_t heater_mul[HEATER_LEN] = {5, 2, 10, 30, 5, 5, 5, 5, 5, 5};

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

    // The shared heating duration is the total measurement duration minus the
    // time needed for the temperature, pressure and humidity measurement.
    uint16_t shared_heatr_dur = MEAS_DUR - (bme690_get_meas_dur(&sensor, BME69X_PARALLEL_MODE) / 1000);

    bme690_set_heater_prof_par(&sensor, heater_temp, heater_mul, shared_heatr_dur, HEATER_LEN);
    bme690_set_op_mode(&sensor, BME69X_PARALLEL_MODE);

    if (bme690_check_status(&sensor) == BME690_ERROR) {
        ESP_LOGE(TAG, "BME690 configuration failed: %s", bme690_status_string(&sensor));
        return;
    }

    ESP_LOGI(TAG, "Timestamp(ms), Temperature(C), Pressure(Pa), Humidity(%%), Gas resistance(Ohm), Status, Gas "
             "index, Meas index");

    while (1) {
        bme690_data_t data;
        uint8_t n_fields_left = 0;

        if (bme690_fetch_data(&sensor)) {
            do {
                n_fields_left = bme690_get_data(&sensor, &data);

                // Skip the fields which hold no valid measurement
                if (data.status == BME690_VALID_DATA) {
                    ESP_LOGI(TAG, "%lld, %.2f, %.2f, %.2f, %.2f, 0x%02X, %u, %u", esp_timer_get_time() / 1000,
                             data.temperature, data.pressure, data.humidity, data.gas_resistance, data.status,
                             data.gas_index, data.meas_index);
                }
            } while (n_fields_left);
        }

        // The shortest heater step of the profile lasts two time bases, so
        // polling twice per time base is fast enough not to miss one.
        vTaskDelay(pdMS_TO_TICKS(MEAS_DUR / 2));
    }
}
