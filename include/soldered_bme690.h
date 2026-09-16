/**
 * @file soldered_bme690.h
 * @brief Public API for the soldered-bme690 component
 *
 * ESP-IDF driver for the Soldered BME690 environmental sensor breakout board
 * over I2C. It is a thin, snake_case wrapper around the Bosch BME69x Sensor
 * API, which lives unmodified in bme69x/.
 *
 * The I2C bus belongs to the application, not to this driver, so that other
 * Qwiic devices can share it. Create it with i2c_new_master_bus() and hand the
 * handle to bme690_init().
 *
 * @author Soldered Electronics
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bme69x.h"
#include "bme69x_defs.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

/** Returned by bme690_check_status() when the last call failed */
#define BME690_ERROR INT8_C(-1)

/** Returned by bme690_check_status() when the last call raised a warning */
#define BME690_WARNING INT8_C(1)

/** I2C clock the sensor is driven at unless bme690_init_with_config() says otherwise */
#define BME690_DEFAULT_SCL_SPEED_HZ 400000

/** How long a single I2C transaction may take before it is given up on */
#define BME690_I2C_TIMEOUT_MS 1000

/** Default temperature over-sampling, used by bme690_set_tph_default() */
#define BME690_DEFAULT_OS_TEMP BME69X_OS_2X

/** Default pressure over-sampling, used by bme690_set_tph_default() */
#define BME690_DEFAULT_OS_PRES BME69X_OS_16X

/** Default humidity over-sampling, used by bme690_set_tph_default() */
#define BME690_DEFAULT_OS_HUM BME69X_OS_1X

/** Number of data fields the sensor buffers internally, one per parallel measurement */
#define BME690_NUM_FIELDS 3

/**
 * @brief New data, gas measurement valid and heater stable, all at once
 *
 * The status byte of a field read in parallel or sequential mode equals this
 * when the field holds a usable gas measurement.
 */
#define BME690_VALID_DATA (BME69X_NEW_DATA_MSK | BME69X_GASM_VALID_MSK | BME69X_HEAT_STAB_MSK)

/**
 * @brief One measured field of the BME690
 *
 * Alias of the Bosch API data structure, holding the compensated temperature in
 * degrees Celsius, the pressure in pascals, the relative humidity in percent and
 * the gas resistance in ohms, plus the status byte and the profile indices.
 */
typedef struct bme69x_data bme690_data_t;

/**
 * @brief Optional settings of a BME690, passed to bme690_init_with_config()
 */
typedef struct {
    uint8_t i2c_addr;                  /**< I2C address, BME69X_I2C_ADDR_LOW or BME69X_I2C_ADDR_HIGH */
    uint32_t scl_speed_hz;             /**< I2C clock in Hz, 0 for ::BME690_DEFAULT_SCL_SPEED_HZ */
    bme69x_delay_us_fptr_t idle_task;  /**< Delay or idle callback, NULL for the built-in one */
} bme690_config_t;

/**
 * @brief Handle for one BME690 breakout board
 *
 * Create one per breakout board. All fields are managed by the driver; treat
 * the struct as opaque and read state through the accessor functions.
 */
typedef struct {
    i2c_master_dev_handle_t i2c_dev; /**< I2C device handle, created by bme690_init() */
    uint8_t i2c_addr;                /**< I2C address the sensor answers on */

    int8_t status; /**< Bosch Sensor API result code of the last executed call */

    struct bme69x_dev bme6;                       /**< Bosch API device structure */
    struct bme69x_conf conf;                      /**< Over-sampling, IIR filter and ODR */
    struct bme69x_heatr_conf heatr_conf;          /**< Gas heater configuration */
    struct bme69x_data data[BME690_NUM_FIELDS];   /**< Local buffer of the last fetched fields */
    uint8_t n_fields;                             /**< Number of new fields in the local buffer */
    uint8_t i_fields;                             /**< Index of the next field to be read out */
    uint8_t last_op_mode;                         /**< Last operation mode which was set */
} bme690_t;

/**
 * @brief Attach a BME690 to an already initialized I2C bus
 *
 * Adds the sensor as a device on `bus` and initializes it, which reads out the
 * calibration data and leaves the sensor in sleep mode. The bus itself must
 * already exist, created with i2c_new_master_bus(); this leaves it free to be
 * shared with other devices.
 *
 * Runs at ::BME690_DEFAULT_SCL_SPEED_HZ; use bme690_init_with_config() to pick
 * a different clock or a custom idle callback.
 *
 * @param[out] dev Handle to initialize
 * @param[in] bus I2C bus the breakout is wired to, previously initialized with
 *                i2c_new_master_bus()
 * @param[in] i2c_addr I2C address of the sensor, BME69X_I2C_ADDR_LOW unless the
 *                     address jumper on the board is soldered
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument,
 *         ESP_ERR_NOT_FOUND if the sensor did not answer, or the error returned
 *         by i2c_master_bus_add_device(). On ESP_ERR_NOT_FOUND the Bosch API
 *         result is left in `dev->status` and bme690_status_string() describes it
 */
esp_err_t bme690_init(bme690_t *dev, i2c_master_bus_handle_t bus, uint8_t i2c_addr);

/**
 * @brief Attach a BME690 to an already initialized I2C bus with custom settings
 *
 * Same as bme690_init(), but lets you pick the I2C clock and replace the
 * built-in delay callback. A custom idle callback can yield to an RTOS task
 * instead of blocking while the sensor measures.
 *
 * @param[out] dev Handle to initialize
 * @param[in] bus I2C bus the breakout is wired to, previously initialized with
 *                i2c_new_master_bus()
 * @param[in] config Settings to apply, see ::bme690_config_t
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL argument,
 *         ESP_ERR_NOT_FOUND if the sensor did not answer, or the error returned
 *         by i2c_master_bus_add_device()
 */
esp_err_t bme690_init_with_config(bme690_t *dev, i2c_master_bus_handle_t bus, const bme690_config_t *config);

/**
 * @brief Detach the sensor from the I2C bus
 *
 * Does not deinitialize the bus itself, since the bus is owned by the caller.
 *
 * @param[in,out] dev Handle previously initialized with bme690_init()
 *
 * @return ESP_OK on success, or the error returned by i2c_master_bus_rm_device()
 */
esp_err_t bme690_deinit(bme690_t *dev);

/**
 * @brief Read a single register
 *
 * @param[in,out] dev Handle
 * @param[in] reg_addr Register address
 *
 * @return Data at that register, 0 if the read failed
 */
uint8_t bme690_read_reg(bme690_t *dev, uint8_t reg_addr);

/**
 * @brief Read multiple consecutive registers
 *
 * @param[in,out] dev Handle
 * @param[in] reg_addr Start register address
 * @param[out] reg_data Buffer the data is read into
 * @param[in] length Number of registers to read
 *
 * @return BME69X_OK on success, a Bosch API error code otherwise
 */
int8_t bme690_read_regs(bme690_t *dev, uint8_t reg_addr, uint8_t *reg_data, uint32_t length);

/**
 * @brief Write data to a single register
 *
 * @param[in,out] dev Handle
 * @param[in] reg_addr Register address
 * @param[in] reg_data Data for that register
 *
 * @return BME69X_OK on success, a Bosch API error code otherwise
 */
int8_t bme690_write_reg(bme690_t *dev, uint8_t reg_addr, uint8_t reg_data);

/**
 * @brief Write multiple registers
 *
 * @param[in,out] dev Handle
 * @param[in] reg_addr Register addresses
 * @param[in] reg_data Data for those registers
 * @param[in] length Number of registers to write
 *
 * @return BME69X_OK on success, a Bosch API error code otherwise
 */
int8_t bme690_write_regs(bme690_t *dev, uint8_t *reg_addr, const uint8_t *reg_data, uint32_t length);

/**
 * @brief Trigger a soft reset of the sensor
 *
 * All settings are lost, so the sensor has to be reconfigured afterwards.
 *
 * @param[in,out] dev Handle
 *
 * @return BME69X_OK on success, a Bosch API error code otherwise
 */
int8_t bme690_soft_reset(bme690_t *dev);

/**
 * @brief Set the ambient temperature used for a better heater configuration
 *
 * Only kept locally, it is applied the next time a heater profile is set.
 *
 * @param[in,out] dev Handle
 * @param[in] temp Temperature in degrees Celsius, 25 by default
 */
void bme690_set_ambient_temp(bme690_t *dev, int8_t temp);

/**
 * @brief Get the measurement duration in microseconds
 *
 * @param[in,out] dev Handle
 * @param[in] op_mode Operation mode the duration is asked for. Pass
 *                    BME69X_SLEEP_MODE to use the last mode which was set
 *
 * @return Temperature, pressure and humidity measurement time in microseconds
 */
uint32_t bme690_get_meas_dur(bme690_t *dev, uint8_t op_mode);

/**
 * @brief Set the operation mode
 *
 * In forced mode this triggers a single measurement; in parallel and sequential
 * mode the sensor keeps measuring on its own until it is put back to sleep.
 *
 * @param[in,out] dev Handle
 * @param[in] op_mode BME69X_SLEEP_MODE, BME69X_FORCED_MODE, BME69X_PARALLEL_MODE
 *                    or BME69X_SEQUENTIAL_MODE
 *
 * @return BME69X_OK on success, a Bosch API error code otherwise
 */
int8_t bme690_set_op_mode(bme690_t *dev, uint8_t op_mode);

/**
 * @brief Get the operation mode the sensor is currently in
 *
 * @param[in,out] dev Handle
 *
 * @return BME69X_SLEEP_MODE, BME69X_FORCED_MODE, BME69X_PARALLEL_MODE or
 *         BME69X_SEQUENTIAL_MODE
 */
uint8_t bme690_get_op_mode(bme690_t *dev);

/**
 * @brief Get the temperature, pressure and humidity over-sampling
 *
 * @param[in,out] dev Handle
 * @param[out] os_hum Humidity over-sampling, BME69X_OS_NONE to BME69X_OS_16X
 * @param[out] os_temp Temperature over-sampling, BME69X_OS_NONE to BME69X_OS_16X
 * @param[out] os_pres Pressure over-sampling, BME69X_OS_NONE to BME69X_OS_16X
 *
 * @return BME69X_OK on success, a Bosch API error code otherwise
 */
int8_t bme690_get_tph(bme690_t *dev, uint8_t *os_hum, uint8_t *os_temp, uint8_t *os_pres);

/**
 * @brief Set the temperature, pressure and humidity over-sampling
 *
 * @param[in,out] dev Handle
 * @param[in] os_temp Temperature over-sampling, BME69X_OS_NONE to BME69X_OS_16X
 * @param[in] os_pres Pressure over-sampling, BME69X_OS_NONE to BME69X_OS_16X
 * @param[in] os_hum Humidity over-sampling, BME69X_OS_NONE to BME69X_OS_16X
 *
 * @return BME69X_OK on success, a Bosch API error code otherwise
 */
int8_t bme690_set_tph(bme690_t *dev, uint8_t os_temp, uint8_t os_pres, uint8_t os_hum);

/**
 * @brief Set the default temperature, pressure and humidity over-sampling
 *
 * Shorthand for bme690_set_tph() with ::BME690_DEFAULT_OS_TEMP,
 * ::BME690_DEFAULT_OS_PRES and ::BME690_DEFAULT_OS_HUM.
 *
 * @param[in,out] dev Handle
 *
 * @return BME69X_OK on success, a Bosch API error code otherwise
 */
int8_t bme690_set_tph_default(bme690_t *dev);

/**
 * @brief Get the IIR filter configuration
 *
 * @param[in,out] dev Handle
 *
 * @return BME69X_FILTER_OFF to BME69X_FILTER_SIZE_127
 */
uint8_t bme690_get_filter(bme690_t *dev);

/**
 * @brief Set the IIR filter configuration
 *
 * @param[in,out] dev Handle
 * @param[in] filter BME69X_FILTER_OFF to BME69X_FILTER_SIZE_127
 *
 * @return BME69X_OK on success, a Bosch API error code otherwise
 */
int8_t bme690_set_filter(bme690_t *dev, uint8_t filter);

/**
 * @brief Get the sleep duration used in sequential mode
 *
 * @param[in,out] dev Handle
 *
 * @return BME69X_ODR_0_59_MS to BME69X_ODR_NONE
 */
uint8_t bme690_get_seq_sleep(bme690_t *dev);

/**
 * @brief Set the sleep duration used in sequential mode
 *
 * @param[in,out] dev Handle
 * @param[in] odr BME69X_ODR_0_59_MS to BME69X_ODR_NONE
 *
 * @return BME69X_OK on success, a Bosch API error code otherwise
 */
int8_t bme690_set_seq_sleep(bme690_t *dev, uint8_t odr);

/**
 * @brief Set the heater profile for forced mode
 *
 * @param[in,out] dev Handle
 * @param[in] temp Heater temperature in degrees Celsius
 * @param[in] dur Heating duration in milliseconds
 *
 * @return BME69X_OK on success, a Bosch API error code otherwise
 */
int8_t bme690_set_heater_prof(bme690_t *dev, uint16_t temp, uint16_t dur);

/**
 * @brief Set the heater profile for sequential mode
 *
 * The two profile arrays have to stay valid for as long as the sensor runs,
 * the Bosch API only keeps pointers to them.
 *
 * @param[in,out] dev Handle
 * @param[in] temp Heater temperature profile in degrees Celsius
 * @param[in] dur Heating duration profile in milliseconds
 * @param[in] profile_len Length of the profile, up to 10
 *
 * @return BME69X_OK on success, a Bosch API error code otherwise
 */
int8_t bme690_set_heater_prof_seq(bme690_t *dev, uint16_t *temp, uint16_t *dur, uint8_t profile_len);

/**
 * @brief Set the heater profile for parallel mode
 *
 * The two profile arrays have to stay valid for as long as the sensor runs,
 * the Bosch API only keeps pointers to them.
 *
 * @param[in,out] dev Handle
 * @param[in] temp Heater temperature profile in degrees Celsius
 * @param[in] mul Profile of the number of repetitions of the shared duration
 * @param[in] shared_heatr_dur Shared heating duration in milliseconds
 * @param[in] profile_len Length of the profile, up to 10
 *
 * @return BME69X_OK on success, a Bosch API error code otherwise
 */
int8_t bme690_set_heater_prof_par(bme690_t *dev, uint16_t *temp, uint16_t *mul, uint16_t shared_heatr_dur,
                                  uint8_t profile_len);

/**
 * @brief Fetch the data from the sensor into the local buffer
 *
 * @param[in,out] dev Handle
 *
 * @return Number of new data fields, zero if there is nothing new
 */
uint8_t bme690_fetch_data(bme690_t *dev);

/**
 * @brief Get a single data field out of the local buffer
 *
 * Call bme690_fetch_data() first. In parallel and sequential mode this walks
 * through the buffered fields one call at a time.
 *
 * @param[in,out] dev Handle
 * @param[out] data Structure the field is copied into
 *
 * @return Number of fields left to read out, zero when this was the last one
 */
uint8_t bme690_get_data(bme690_t *dev, bme690_data_t *data);

/**
 * @brief Get the whole local data buffer
 *
 * @param[in,out] dev Handle
 *
 * @return Pointer to the ::BME690_NUM_FIELDS buffered data fields
 */
bme690_data_t *bme690_get_all_data(bme690_t *dev);

/**
 * @brief Get the currently set heater configuration
 *
 * @param[in] dev Handle
 *
 * @return Pointer to the heater configuration held in the handle
 */
const struct bme69x_heatr_conf *bme690_get_heater_conf(const bme690_t *dev);

/**
 * @brief Retrieve the unique ID of the sensor
 *
 * @param[in,out] dev Handle
 *
 * @return Unique ID, 0 if the read failed
 */
uint32_t bme690_get_unique_id(bme690_t *dev);

/**
 * @brief Run the built-in self test of the sensor
 *
 * Takes a few seconds. The sensor is left in sleep mode and has to be
 * reconfigured afterwards.
 *
 * @param[in,out] dev Handle
 *
 * @return BME69X_OK if the self test passed, an error code otherwise
 */
int8_t bme690_self_test(bme690_t *dev);

/**
 * @brief Get the error code of the interface functions
 *
 * @param[in] dev Handle
 *
 * @return Interface return code, an esp_err_t cast to BME69X_INTF_RET_TYPE
 */
BME69X_INTF_RET_TYPE bme690_intf_error(const bme690_t *dev);

/**
 * @brief Check whether an error or a warning has occurred
 *
 * @param[in] dev Handle
 *
 * @return ::BME690_ERROR if the last call failed, ::BME690_WARNING if it raised
 *         a warning, BME69X_OK otherwise
 */
int8_t bme690_check_status(const bme690_t *dev);

/**
 * @brief Get a brief text description of the last status code
 *
 * @param[in] dev Handle
 *
 * @return String describing the status code, an empty string when it is
 *         BME69X_OK. The string is static and does not have to be freed
 */
const char *bme690_status_string(const bme690_t *dev);

#ifdef __cplusplus
}
#endif
