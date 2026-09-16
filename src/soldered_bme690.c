/**
 * @file soldered_bme690.c
 * @brief Implementation for the soldered-bme690 component
 *
 * Wraps the Bosch BME69x Sensor API, which lives unmodified in bme69x/, in an
 * ESP-IDF flavoured I2C driver.
 *
 * @author Soldered Electronics
 */

#include <string.h>
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soldered_bme690.h"

/* Typical room temperature in degrees Celsius, the starting point for the
 * heater resistance calculation until the application knows better. */
#define BME690_DEFAULT_AMBIENT_TEMP 25

/* Largest transaction the driver issues: three data fields of 17 bytes each. */
#define BME690_MAX_READ_LENGTH 51

// *****************************************************************************
// Section: Bosch Sensor API callbacks
//
// The sensor API reaches the outside world through these three callbacks. The
// interface descriptor it hands back to them is the handle itself, so that they
// can both talk over I2C and record the ESP-IDF error code.

/**
 * @brief Delay callback handed to the Bosch API
 *
 * Waits the requested number of microseconds. Anything longer than a tick is
 * slept away in the scheduler so that other tasks keep running while the sensor
 * measures; the remainder is busy waited, since the tick is the finest the
 * scheduler can do.
 *
 * @param[in] period_us Duration of the delay in microseconds
 * @param[in] intf_ptr Pointer to the handle, unused here
 */
static void bme690_delay_us(uint32_t period_us, void *intf_ptr)
{
    const uint32_t tick_us = portTICK_PERIOD_MS * 1000;

    (void)intf_ptr;

    if (period_us >= tick_us) {
        vTaskDelay(period_us / tick_us);
        period_us %= tick_us;
    }

    if (period_us) {
        esp_rom_delay_us(period_us);
    }
}

/**
 * @brief I2C write callback handed to the Bosch API
 *
 * @param[in] reg_addr Register address of the sensor
 * @param[in] reg_data Data to be written to the sensor
 * @param[in] length Length of the transfer
 * @param[in] intf_ptr Pointer to the handle
 *
 * @return BME69X_OK if successful, a Bosch API error code otherwise
 */
static BME69X_INTF_RET_TYPE bme690_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length,
                                             void *intf_ptr)
{
    bme690_t *dev = (bme690_t *)intf_ptr;
    uint8_t buf[BME690_MAX_READ_LENGTH + 1];
    esp_err_t err;

    if ((dev == NULL) || (dev->i2c_dev == NULL)) {
        return BME69X_E_NULL_PTR;
    }

    if (length + 1 > sizeof(buf)) {
        return BME69X_E_INVALID_LENGTH;
    }

    /* The register address and the data go out as one transaction, so build
     * them into a single buffer first. */
    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, length);

    err = i2c_master_transmit(dev->i2c_dev, buf, length + 1, BME690_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return BME69X_E_COM_FAIL;
    }

    return BME69X_OK;
}

/**
 * @brief I2C read callback handed to the Bosch API
 *
 * @param[in] reg_addr Register address of the sensor
 * @param[out] reg_data Buffer the data is read into
 * @param[in] length Length of the transfer
 * @param[in] intf_ptr Pointer to the handle
 *
 * @return BME69X_OK if successful, a Bosch API error code otherwise
 */
static BME69X_INTF_RET_TYPE bme690_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    bme690_t *dev = (bme690_t *)intf_ptr;
    esp_err_t err;

    if ((dev == NULL) || (dev->i2c_dev == NULL)) {
        return BME69X_E_NULL_PTR;
    }

    err = i2c_master_transmit_receive(dev->i2c_dev, &reg_addr, 1, reg_data, length, BME690_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return BME69X_E_COM_FAIL;
    }

    return BME69X_OK;
}

// *****************************************************************************
// Section: Initialization

esp_err_t bme690_init(bme690_t *dev, i2c_master_bus_handle_t bus, uint8_t i2c_addr)
{
    bme690_config_t config = {
        .i2c_addr = i2c_addr,
        .scl_speed_hz = BME690_DEFAULT_SCL_SPEED_HZ,
        .idle_task = NULL,
    };

    return bme690_init_with_config(dev, bus, &config);
}

esp_err_t bme690_init_with_config(bme690_t *dev, i2c_master_bus_handle_t bus, const bme690_config_t *config)
{
    esp_err_t err;

    if ((dev == NULL) || (bus == NULL) || (config == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->i2c_addr = config->i2c_addr;
    dev->status = BME69X_OK;
    dev->last_op_mode = BME69X_SLEEP_MODE;

    i2c_device_config_t i2c_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->i2c_addr,
        .scl_speed_hz = config->scl_speed_hz ? config->scl_speed_hz : BME690_DEFAULT_SCL_SPEED_HZ,
    };

    err = i2c_master_bus_add_device(bus, &i2c_conf, &dev->i2c_dev);
    if (err != ESP_OK) {
        return err;
    }

    dev->bme6.intf = BME69X_I2C_INTF;
    dev->bme6.read = bme690_i2c_read;
    dev->bme6.write = bme690_i2c_write;
    dev->bme6.delay_us = config->idle_task ? config->idle_task : bme690_delay_us;
    dev->bme6.intf_ptr = dev;
    dev->bme6.amb_temp = BME690_DEFAULT_AMBIENT_TEMP;

    dev->status = bme69x_init(&dev->bme6);
    if (dev->status != BME69X_OK) {
        i2c_master_bus_rm_device(dev->i2c_dev);
        dev->i2c_dev = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

esp_err_t bme690_deinit(bme690_t *dev)
{
    esp_err_t err;

    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->i2c_dev == NULL) {
        return ESP_OK;
    }

    err = i2c_master_bus_rm_device(dev->i2c_dev);
    dev->i2c_dev = NULL;

    return err;
}

// *****************************************************************************
// Section: Register access

uint8_t bme690_read_reg(bme690_t *dev, uint8_t reg_addr)
{
    uint8_t reg_data = 0;

    bme690_read_regs(dev, reg_addr, &reg_data, 1);

    return reg_data;
}

int8_t bme690_read_regs(bme690_t *dev, uint8_t reg_addr, uint8_t *reg_data, uint32_t length)
{
    dev->status = bme69x_get_regs(reg_addr, reg_data, length, &dev->bme6);

    return dev->status;
}

int8_t bme690_write_reg(bme690_t *dev, uint8_t reg_addr, uint8_t reg_data)
{
    dev->status = bme69x_set_regs(&reg_addr, &reg_data, 1, &dev->bme6);

    return dev->status;
}

int8_t bme690_write_regs(bme690_t *dev, uint8_t *reg_addr, const uint8_t *reg_data, uint32_t length)
{
    dev->status = bme69x_set_regs(reg_addr, reg_data, length, &dev->bme6);

    return dev->status;
}

int8_t bme690_soft_reset(bme690_t *dev)
{
    dev->status = bme69x_soft_reset(&dev->bme6);

    return dev->status;
}

// *****************************************************************************
// Section: Configuration

void bme690_set_ambient_temp(bme690_t *dev, int8_t temp)
{
    dev->bme6.amb_temp = temp;
}

uint32_t bme690_get_meas_dur(bme690_t *dev, uint8_t op_mode)
{
    if (op_mode == BME69X_SLEEP_MODE) {
        op_mode = dev->last_op_mode;
    }

    return bme69x_get_meas_dur(op_mode, &dev->conf, &dev->bme6);
}

int8_t bme690_set_op_mode(bme690_t *dev, uint8_t op_mode)
{
    dev->status = bme69x_set_op_mode(op_mode, &dev->bme6);

    if ((dev->status == BME69X_OK) && (op_mode != BME69X_SLEEP_MODE)) {
        dev->last_op_mode = op_mode;
    }

    return dev->status;
}

uint8_t bme690_get_op_mode(bme690_t *dev)
{
    uint8_t op_mode = BME69X_SLEEP_MODE;

    dev->status = bme69x_get_op_mode(&op_mode, &dev->bme6);

    return op_mode;
}

int8_t bme690_get_tph(bme690_t *dev, uint8_t *os_hum, uint8_t *os_temp, uint8_t *os_pres)
{
    dev->status = bme69x_get_conf(&dev->conf, &dev->bme6);

    if (dev->status == BME69X_OK) {
        if (os_hum) {
            *os_hum = dev->conf.os_hum;
        }
        if (os_temp) {
            *os_temp = dev->conf.os_temp;
        }
        if (os_pres) {
            *os_pres = dev->conf.os_pres;
        }
    }

    return dev->status;
}

int8_t bme690_set_tph(bme690_t *dev, uint8_t os_temp, uint8_t os_pres, uint8_t os_hum)
{
    dev->status = bme69x_get_conf(&dev->conf, &dev->bme6);

    if (dev->status == BME69X_OK) {
        dev->conf.os_hum = os_hum;
        dev->conf.os_temp = os_temp;
        dev->conf.os_pres = os_pres;

        dev->status = bme69x_set_conf(&dev->conf, &dev->bme6);
    }

    return dev->status;
}

int8_t bme690_set_tph_default(bme690_t *dev)
{
    return bme690_set_tph(dev, BME690_DEFAULT_OS_TEMP, BME690_DEFAULT_OS_PRES, BME690_DEFAULT_OS_HUM);
}

uint8_t bme690_get_filter(bme690_t *dev)
{
    dev->status = bme69x_get_conf(&dev->conf, &dev->bme6);

    return dev->conf.filter;
}

int8_t bme690_set_filter(bme690_t *dev, uint8_t filter)
{
    dev->status = bme69x_get_conf(&dev->conf, &dev->bme6);

    if (dev->status == BME69X_OK) {
        dev->conf.filter = filter;

        dev->status = bme69x_set_conf(&dev->conf, &dev->bme6);
    }

    return dev->status;
}

uint8_t bme690_get_seq_sleep(bme690_t *dev)
{
    dev->status = bme69x_get_conf(&dev->conf, &dev->bme6);

    return dev->conf.odr;
}

int8_t bme690_set_seq_sleep(bme690_t *dev, uint8_t odr)
{
    dev->status = bme69x_get_conf(&dev->conf, &dev->bme6);

    if (dev->status == BME69X_OK) {
        dev->conf.odr = odr;

        dev->status = bme69x_set_conf(&dev->conf, &dev->bme6);
    }

    return dev->status;
}

// *****************************************************************************
// Section: Heater profiles

int8_t bme690_set_heater_prof(bme690_t *dev, uint16_t temp, uint16_t dur)
{
    dev->heatr_conf.enable = BME69X_ENABLE;
    dev->heatr_conf.heatr_temp = temp;
    dev->heatr_conf.heatr_dur = dur;

    dev->status = bme69x_set_heatr_conf(BME69X_FORCED_MODE, &dev->heatr_conf, &dev->bme6);

    return dev->status;
}

int8_t bme690_set_heater_prof_seq(bme690_t *dev, uint16_t *temp, uint16_t *dur, uint8_t profile_len)
{
    dev->heatr_conf.enable = BME69X_ENABLE;
    dev->heatr_conf.heatr_temp_prof = temp;
    dev->heatr_conf.heatr_dur_prof = dur;
    dev->heatr_conf.profile_len = profile_len;

    dev->status = bme69x_set_heatr_conf(BME69X_SEQUENTIAL_MODE, &dev->heatr_conf, &dev->bme6);

    return dev->status;
}

int8_t bme690_set_heater_prof_par(bme690_t *dev, uint16_t *temp, uint16_t *mul, uint16_t shared_heatr_dur,
                                  uint8_t profile_len)
{
    dev->heatr_conf.enable = BME69X_ENABLE;
    dev->heatr_conf.heatr_temp_prof = temp;
    dev->heatr_conf.heatr_dur_prof = mul;
    dev->heatr_conf.shared_heatr_dur = shared_heatr_dur;
    dev->heatr_conf.profile_len = profile_len;

    dev->status = bme69x_set_heatr_conf(BME69X_PARALLEL_MODE, &dev->heatr_conf, &dev->bme6);

    return dev->status;
}

const struct bme69x_heatr_conf *bme690_get_heater_conf(const bme690_t *dev)
{
    return &dev->heatr_conf;
}

// *****************************************************************************
// Section: Reading out measurements

uint8_t bme690_fetch_data(bme690_t *dev)
{
    dev->n_fields = 0;
    dev->status = bme69x_get_data(dev->last_op_mode, dev->data, &dev->n_fields, &dev->bme6);
    dev->i_fields = 0;

    return dev->n_fields;
}

uint8_t bme690_get_data(bme690_t *dev, bme690_data_t *data)
{
    if (data == NULL) {
        return 0;
    }

    if (dev->last_op_mode == BME69X_FORCED_MODE) {
        *data = dev->data[0];
    } else if (dev->n_fields) {
        /* i_fields spans from 0-2 while n_fields spans from 0-3, where 0 means
         * that there is no new data. */
        *data = dev->data[dev->i_fields];
        dev->i_fields++;

        /* Limit reading continuously to the last field read. */
        if (dev->i_fields >= dev->n_fields) {
            dev->i_fields = dev->n_fields - 1;
            return 0;
        }

        /* Indicate if there is something left to read. */
        return dev->n_fields - dev->i_fields;
    }

    return 0;
}

bme690_data_t *bme690_get_all_data(bme690_t *dev)
{
    return dev->data;
}

// *****************************************************************************
// Section: Sensor identification and status

uint32_t bme690_get_unique_id(bme690_t *dev)
{
    uint8_t id_regs[4];
    uint32_t id1;

    if (bme690_read_regs(dev, BME69X_REG_UNIQUE_ID, id_regs, sizeof(id_regs)) != BME69X_OK) {
        return 0;
    }

    id1 = ((uint32_t)id_regs[3] + ((uint32_t)id_regs[2] << 8)) & 0x7fff;

    return (id1 << 16) + (((uint32_t)id_regs[1]) << 8) + (uint32_t)id_regs[0];
}

int8_t bme690_self_test(bme690_t *dev)
{
    dev->status = bme69x_selftest_check(&dev->bme6);

    return dev->status;
}

BME69X_INTF_RET_TYPE bme690_intf_error(const bme690_t *dev)
{
    return dev->bme6.intf_rslt;
}

int8_t bme690_check_status(const bme690_t *dev)
{
    if (dev->status < BME69X_OK) {
        return BME690_ERROR;
    }

    if (dev->status > BME69X_OK) {
        return BME690_WARNING;
    }

    return BME69X_OK;
}

const char *bme690_status_string(const bme690_t *dev)
{
    switch (dev->status) {
    case BME69X_OK:
        /* Don't return a text for OK. */
        return "";
    case BME69X_E_NULL_PTR:
        return "Null pointer";
    case BME69X_E_COM_FAIL:
        return "Communication failure";
    case BME69X_E_DEV_NOT_FOUND:
        return "Sensor not found";
    case BME69X_E_INVALID_LENGTH:
        return "Invalid length";
    case BME69X_E_SELF_TEST:
        return "Self test failed";
    case BME69X_W_DEFINE_OP_MODE:
        return "Set the operation mode";
    case BME69X_W_NO_NEW_DATA:
        return "No new data";
    case BME69X_W_DEFINE_SHD_HEATR_DUR:
        return "Set the shared heater duration";
    default:
        return "Undefined error code";
    }
}
