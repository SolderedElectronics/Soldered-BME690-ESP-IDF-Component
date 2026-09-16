/**
 * @file main.c
 * @brief Records BME690 raw data into a .bmerawdata file on an SD card
 *
 * The file follows the Bosch BME AI-Studio Raw Data Format, so it can be
 * imported into BME AI-Studio and used to train a gas classification algorithm.
 *
 * The example runs the sensor in parallel mode with a ten step heater profile,
 * logs every valid gas measurement for LOG_DURATION_MS and then writes the
 * finished file.
 *
 * AI-Studio splits a recording into Specimens wherever the label tag changes,
 * and it measures the duration of a Specimen from its first to its last point.
 * A recording whose label tag never changes therefore imports as a Specimen of
 * zero length. The example starts the recording already tagged, and the button
 * on LABEL_BUTTON_PIN switches the tag while it runs, which marks the moment as
 * the start of a new Specimen. That is the same thing buttons S1 and S2 do on
 * the BME688 Development Kit.
 *
 * Needed hardware:
 * - an ESP32 board (WiFi and NTP are used for the real time clock)
 * - a BME690 breakout on the I2C pins, or connected via Qwiic
 * - an SD card module on the SPI pins, chip select on PIN_NUM_SD_CS
 *
 * The AI-Studio workflow is described here:
 * https://www.bosch-sensortec.com/software/bme/docs/overview/getting-started.html
 *
 * Product used is www.solde.red/333411
 *
 * @author Soldered Electronics
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "cJSON.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdmmc_cmd.h"
#include "soldered_bme690.h"

static const char *TAG = "BME690_AI_LOGGER";

// WiFi credentials, only used to get the current time over NTP
#define WIFI_SSID     "YOUR_SSID_HERE"
#define WIFI_PASSWORD "YOUR_PASSWORD_HERE"

// Change these to match how your breakout is wired
#define PIN_NUM_SDA GPIO_NUM_21
#define PIN_NUM_SCL GPIO_NUM_22

// SPI pins of the SD card module
#define PIN_NUM_SD_MOSI GPIO_NUM_23
#define PIN_NUM_SD_MISO GPIO_NUM_19
#define PIN_NUM_SD_SCK  GPIO_NUM_18
#define PIN_NUM_SD_CS   GPIO_NUM_5

// Where the card is mounted in the virtual file system
#define SD_MOUNT_POINT "/sdcard"

// SPI clock the card is driven at, in kHz. Lower it to SDMMC_FREQ_PROBING
// (400) if the card does not come up on long jumper wires.
#define SD_CARD_FREQ_KHZ 4000

// Button which marks the start of a new Specimen while recording. It is active
// low, so the internal pull up is enough and no wiring is needed if the board
// already has a button on this pin. Set it to -1 to record the whole session as
// a single Specimen.
#define LABEL_BUTTON_PIN 0

// Debounce time of that button
#define LABEL_BUTTON_DEBOUNCE_MS 250

// Total duration of one measurement session
#define LOG_DURATION_MS (10UL * 60000UL)

// Duration of a single heater profile step in milliseconds. The Raw Data Format
// expects the heater profile time base to be 140 ms.
#define MEAS_DUR 140

// Identifier of the board which recorded the data. AI-Studio uses it to tell
// recordings of different boards apart, so give every board its own value.
#define BOARD_ID "E0E2E69BA804"

// Unique id of the sensor element, used to trace a row back to one sensor
#define SENSOR_ID 1903381786UL

// Index of the sensor on the board. A single BME690 breakout only has one.
#define SENSOR_INDEX 0

#define HEATER_LEN 10

// Heater temperature profile in degrees Celsius. It stays in scope for the
// whole run, the sensor API only keeps a pointer to it.
static uint16_t heater_temp[HEATER_LEN] = {320, 100, 100, 100, 200, 200, 200, 320, 320, 320};

// Multipliers of the shared heater duration, one per profile step
static uint16_t heater_mul[HEATER_LEN] = {5, 2, 10, 30, 5, 5, 5, 5, 5, 5};

/**
 * @brief Milliseconds since boot
 */
static uint32_t millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/**
 * @brief Log an error and stop the task if the sensor reported one
 *
 * @param[in] sensor Handle to check
 *
 * @return true if the recording can go on, false if the sensor failed
 */
static bool check_sensor_status(const bme690_t *sensor)
{
    int8_t status = bme690_check_status(sensor);

    if (status == BME690_ERROR) {
        ESP_LOGE(TAG, "BME690 error: %s", bme690_status_string(sensor));
        return false;
    }

    if (status == BME690_WARNING) {
        ESP_LOGW(TAG, "BME690 warning: %s", bme690_status_string(sensor));
    }

    return true;
}

/**
 * @brief Format a unix timestamp as an ISO 8601 UTC string, as the format requires
 *
 * @param[in] t Timestamp to format
 * @param[out] buffer Buffer the string is written into
 * @param[in] size Size of that buffer
 */
static void iso8601(time_t t, char *buffer, size_t size)
{
    struct tm time_info;

    gmtime_r(&t, &time_info);
    strftime(buffer, size, "%Y-%m-%dT%H:%M:%S+00:00", &time_info);
}

/**
 * @brief Format a unix timestamp as the yyyy_mm_dd_hh_mm date used in the file name
 *
 * @param[in] t Timestamp to format
 * @param[out] buffer Buffer the string is written into
 * @param[in] size Size of that buffer
 */
static void file_name_date(time_t t, char *buffer, size_t size)
{
    struct tm time_info;

    gmtime_r(&t, &time_info);
    strftime(buffer, size, "%Y_%m_%d_%H_%M", &time_info);
}

/**
 * @brief Build the random seed which labels all files of one measurement session
 *
 * The format expects sixteen lowercase alphanumeric characters.
 *
 * @param[out] seed Buffer of at least 17 bytes the seed is written into
 */
static void make_seed(char *seed)
{
    const char alphabet[] = "abcdefghijklmnopqrstuvwxyz0123456789";

    for (uint8_t i = 0; i < 16; i++) {
        seed[i] = alphabet[esp_random() % (sizeof(alphabet) - 1)];
    }

    seed[16] = '\0';
}

/**
 * @brief Connect to the WiFi network the credentials at the top point at
 *
 * Only used to reach an NTP server, the format stores both an absolute creation
 * date and a unix timestamp for every measurement.
 *
 * @return ESP_OK once the station has an IP address
 */
static esp_err_t wifi_connect(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    // esp_wifi_connect() only starts the attempt, so poll until the station is
    // actually associated rather than setting up an event handler for it.
    ESP_LOGI(TAG, "Connecting to WiFi");
    for (int i = 0; i < 60; i++) {
        wifi_ap_record_t ap_info;

        esp_wifi_connect();
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            ESP_LOGI(TAG, "WiFi connected");
            return ESP_OK;
        }
    }

    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Mount an SD card wired to the SPI pins at ::SD_MOUNT_POINT
 *
 * @param[out] card Card structure filled in by the driver
 *
 * @return ESP_OK on success, an error code otherwise
 */
static esp_err_t sd_card_mount(sdmmc_card_t **card)
{
    esp_err_t err;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    // SPI3 (VSPI) drives GPIO 18/19/23 straight through the IOMUX on the ESP32.
    // SPI2 works too, but routes through the GPIO matrix. Use SPI2_HOST on chips
    // which have no SPI3.
    host.slot = SPI3_HOST;

    // SDSPI_HOST_DEFAULT() clocks the card at 20 MHz, which most modules do not
    // survive on jumper wires. 4 MHz is plenty for this recording. Drop it
    // further, down to SDMMC_FREQ_PROBING, if the card still does not come up.
    host.max_freq_khz = SD_CARD_FREQ_KHZ;

    // ESP-IDF turns on CRC verification with CMD59 while initializing a card in
    // SPI mode. Plenty of cards, and most of the cheap breakout modules, never
    // answer that command, and initialization then fails with ESP_ERR_TIMEOUT in
    // sdmmc_init_spi_crc(). Skipping the command is what makes those cards work
    // here. The SPI transfers themselves are still checksummed by the command
    // CRC7, only the data block CRC16 is given up, so drop this flag if your
    // card does answer CMD59.
    host.flags |= SDMMC_HOST_FLAG_SPI_IGNORE_DATA_CRC;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_SD_MOSI,
        .miso_io_num = PIN_NUM_SD_MISO,
        .sclk_io_num = PIN_NUM_SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    err = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        return err;
    }

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = PIN_NUM_SD_CS;
    slot_cfg.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 3,
        .allocation_unit_size = 16 * 1024,
    };

    return esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_cfg, &mount_cfg, card);
}

/**
 * @brief Build the header of the .bmerawdata file
 *
 * Everything except the measurements themselves is small enough to build in
 * memory, so the board configuration and the column description are assembled
 * with cJSON first. A ten minute recording holds a few thousand rows, which is
 * far more than fits in RAM as a JSON document, so the part built here is
 * written out with its two closing braces removed, and the rows are appended to
 * the file one by one as they are measured.
 *
 * @param[in] iso_now Creation date of the recording as an ISO 8601 string
 * @param[in] now Creation date of the recording as a unix timestamp
 * @param[in] seed_power_on_off Session seed, repeated in the file name
 *
 * @return Serialized header with its closing braces stripped, to be freed by
 *         the caller, or NULL if it could not be built
 */
static char *build_header(const char *iso_now, time_t now, const char *seed_power_on_off)
{
    char date_created[16];
    char *header;
    size_t length;

    cJSON *doc = cJSON_CreateObject();
    if (doc == NULL) {
        return NULL;
    }

    // The board configuration, the same two objects are stored in a .bmeconfig
    // file by AI-Studio.
    cJSON *config_header = cJSON_AddObjectToObject(doc, "configHeader");
    cJSON_AddStringToObject(config_header, "dateCreated_ISO", iso_now);
    cJSON_AddStringToObject(config_header, "appVersion", "2.2.0");
    cJSON_AddStringToObject(config_header, "boardType", "soldered_bme690");
    cJSON_AddStringToObject(config_header, "boardMode", "burn_in");
    cJSON_AddStringToObject(config_header, "boardLayout", "grouped");

    cJSON *config_body = cJSON_AddObjectToObject(doc, "configBody");

    cJSON *heater_profiles = cJSON_AddArrayToObject(config_body, "heaterProfiles");
    cJSON *heater_profile = cJSON_CreateObject();
    cJSON_AddItemToArray(heater_profiles, heater_profile);
    cJSON_AddStringToObject(heater_profile, "id", "heater_354");
    cJSON_AddNumberToObject(heater_profile, "timeBase", MEAS_DUR);

    cJSON *temperature_time_vectors = cJSON_AddArrayToObject(heater_profile, "temperatureTimeVectors");
    for (uint8_t i = 0; i < HEATER_LEN; i++) {
        cJSON *pair = cJSON_CreateArray();
        cJSON_AddItemToArray(pair, cJSON_CreateNumber(heater_temp[i]));
        cJSON_AddItemToArray(pair, cJSON_CreateNumber(heater_mul[i]));
        cJSON_AddItemToArray(temperature_time_vectors, pair);
    }

    // The sensor scans continuously, so there are no sleeping cycles.
    cJSON *duty_cycle_profiles = cJSON_AddArrayToObject(config_body, "dutyCycleProfiles");
    cJSON *duty_cycle_profile = cJSON_CreateObject();
    cJSON_AddItemToArray(duty_cycle_profiles, duty_cycle_profile);
    cJSON_AddStringToObject(duty_cycle_profile, "id", "duty_1");
    cJSON_AddNumberToObject(duty_cycle_profile, "numberScanningCycles", 1);
    cJSON_AddNumberToObject(duty_cycle_profile, "numberSleepingCycles", 0);

    cJSON *sensor_configurations = cJSON_AddArrayToObject(config_body, "sensorConfigurations");
    cJSON *sensor_configuration = cJSON_CreateObject();
    cJSON_AddItemToArray(sensor_configurations, sensor_configuration);
    cJSON_AddNumberToObject(sensor_configuration, "sensorIndex", SENSOR_INDEX);
    cJSON_AddStringToObject(sensor_configuration, "heaterProfile", "heater_354");
    cJSON_AddStringToObject(sensor_configuration, "dutyCycleProfile", "duty_1");

    // The header of the recording itself, the counters and the seed repeat the
    // matching parts of the file name.
    snprintf(date_created, sizeof(date_created), "%lu", (unsigned long)now);

    cJSON *raw_data_header = cJSON_AddObjectToObject(doc, "rawDataHeader");
    cJSON_AddNumberToObject(raw_data_header, "counterPowerOnOff", 1);
    cJSON_AddStringToObject(raw_data_header, "seedPowerOnOff", seed_power_on_off);
    cJSON_AddNumberToObject(raw_data_header, "counterFileLimit", 0);
    cJSON_AddStringToObject(raw_data_header, "dateCreated", date_created);
    cJSON_AddStringToObject(raw_data_header, "dateCreated_ISO", iso_now);
    cJSON_AddStringToObject(raw_data_header, "firmwareVersion", "1.0.0");
    cJSON_AddStringToObject(raw_data_header, "boardId", BOARD_ID);

    // The column description of the data block which follows it. The order of
    // the columns here is the order of the values in every recorded row.
    cJSON *raw_data_body = cJSON_AddObjectToObject(doc, "rawDataBody");
    cJSON *data_columns = cJSON_AddArrayToObject(raw_data_body, "dataColumns");

    const char *columns[][4] = {
        {"Sensor Index", "", "integer", "sensor_index"},
        {"Sensor ID", "", "integer", "sensor_id"},
        {"Time Since PowerOn", "Milliseconds", "integer", "timestamp_since_poweron"},
        {
            "Real time clock", "Unix Timestamp: seconds since Jan 01 1970. (UTC); 0 = missing", "integer",
            "real_time_clock"
        },
        {"Temperature", "DegreesCelcius", "float", "temperature"},
        {"Pressure", "Hectopascals", "float", "pressure"},
        {"Relative Humidity", "Percent", "float", "relative_humidity"},
        {"Resistance Gassensor", "Ohms", "float", "resistance_gassensor"},
        {"Heater Profile Step Index", "", "integer", "heater_profile_step_index"},
        {"Scanning Mode Enabled", "", "boolean", "scanning_enabled"},
        {"Scanning Cycle Index", "", "integer", "scanning_cycle_index"},
        {"Label Tag", "", "integer", "label_tag"},
        {"Error Code", "", "integer", "error_code"},
    };

    for (uint8_t i = 0; i < sizeof(columns) / sizeof(columns[0]); i++) {
        cJSON *column = cJSON_CreateObject();
        cJSON_AddItemToArray(data_columns, column);
        cJSON_AddStringToObject(column, "name", columns[i][0]);
        cJSON_AddStringToObject(column, "unit", columns[i][1]);
        cJSON_AddStringToObject(column, "format", columns[i][2]);
        cJSON_AddStringToObject(column, "key", columns[i][3]);
        cJSON_AddNumberToObject(column, "colId", i + 1);
    }

    header = cJSON_PrintUnformatted(doc);
    cJSON_Delete(doc);

    if (header == NULL) {
        return NULL;
    }

    // Strip the two closing braces of rawDataBody and of the root object, so
    // that the data block can be appended behind them.
    length = strlen(header);
    if (length < 2) {
        cJSON_free(header);
        return NULL;
    }
    header[length - 2] = '\0';

    return header;
}

void app_main(void)
{
    bme690_t sensor;
    sdmmc_card_t *card;
    char iso_now[40];
    char date[24];
    char seed_power_on_off[17];
    char file_name[128];

    ESP_LOGI(TAG, "BME690 AI-Studio raw data logger");

    // WiFi needs NVS for its calibration data
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    if (wifi_connect() != ESP_OK) {
        ESP_LOGE(TAG, "Could not connect to WiFi!");
        return;
    }

    esp_sntp_config_t sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    ESP_ERROR_CHECK(esp_netif_sntp_init(&sntp_cfg));

    ESP_LOGI(TAG, "Waiting for NTP");
    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(30000)) != ESP_OK) {
        ESP_LOGE(TAG, "Could not get the time over NTP!");
        return;
    }
    ESP_LOGI(TAG, "Time synced");

    time_t now = time(NULL);
    iso8601(now, iso_now, sizeof(iso_now));

    // Every session gets its own seed instead of repeating the same one after
    // every reset, esp_random() is seeded by the hardware RNG.
    make_seed(seed_power_on_off);

    if (LABEL_BUTTON_PIN >= 0) {
        gpio_config_t button_cfg = {
            .pin_bit_mask = 1ULL << LABEL_BUTTON_PIN,
                                 .mode = GPIO_MODE_INPUT,
                                 .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&button_cfg));
    }

    if (sd_card_mount(&card) != ESP_OK) {
        ESP_LOGE(TAG, "SD card initialization failed!");
        return;
    }
    ESP_LOGI(TAG, "SD card ready");

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

    if (bme690_init(&sensor, bus, BME69X_I2C_ADDR_LOW) != ESP_OK) {
        ESP_LOGE(TAG, "BME690 initialization failed: %s", bme690_status_string(&sensor));
        return;
    }

    // Default temperature, pressure and humidity oversampling
    bme690_set_tph_default(&sensor);
    if (!check_sensor_status(&sensor)) {
        return;
    }

    // The shared heating duration is the total measurement duration minus the
    // time needed for the temperature, pressure and humidity measurement.
    uint16_t shared_heatr_dur = MEAS_DUR - (bme690_get_meas_dur(&sensor, BME69X_PARALLEL_MODE) / 1000);

    bme690_set_heater_prof_par(&sensor, heater_temp, heater_mul, shared_heatr_dur, HEATER_LEN);
    if (!check_sensor_status(&sensor)) {
        return;
    }

    bme690_set_op_mode(&sensor, BME69X_PARALLEL_MODE);
    if (!check_sensor_status(&sensor)) {
        return;
    }

    // The file name carries the date, the board, the power on counter, the
    // session seed and the file counter, each part separated by an underscore.
    file_name_date(now, date, sizeof(date));
    snprintf(file_name, sizeof(file_name), "%s/%s_Board_%s_PowerOnOff_1_%s_File_0.bmerawdata", SD_MOUNT_POINT, date,
             BOARD_ID, seed_power_on_off);

    FILE *file = fopen(file_name, "w");
    if (file == NULL) {
        ESP_LOGE(TAG, "Could not create the file on the SD card!");
        return;
    }

    char *header = build_header(iso_now, now, seed_power_on_off);
    if (header == NULL) {
        ESP_LOGE(TAG, "Could not build the file header!");
        fclose(file);
        return;
    }

    fputs(header, file);
    fputs(",\"dataBlock\":[", file);
    cJSON_free(header);

    ESP_LOGI(TAG, "Recording, this takes %lu minutes.", (unsigned long)(LOG_DURATION_MS / 60000UL));

    uint32_t start = millis();

    // The Raw Data Format counts the Scanning Cycles from one
    uint32_t scanning_cycle_index = 1;

    uint32_t rows = 0;
    uint32_t dropped_cycles = 0;

    // Recording starts already tagged, so that the whole session forms a
    // Specimen even when the button is never pressed. Pressing the button
    // switches the tag, which starts the next Specimen.
    uint8_t label_tag = 1;
    uint32_t last_button_press = 0;

    // No step has been seen yet, the first one is expected to be step zero
    int16_t last_step_index = -1;

    uint8_t n_fields_left = 0;
    bme690_data_t data;
    char row[192];

    while (millis() - start < LOG_DURATION_MS) {
        // The shortest heater step of the profile lasts two time bases, so
        // polling twice per time base is fast enough not to miss one.
        vTaskDelay(pdMS_TO_TICKS(MEAS_DUR / 2));

        // Mark the start of a new Specimen when the button is pressed. The tag
        // alternates between one and two, the same way the two buttons of the
        // BME688 Development Kit are used.
        if (LABEL_BUTTON_PIN >= 0 && gpio_get_level(LABEL_BUTTON_PIN) == 0 &&
                millis() - last_button_press > LABEL_BUTTON_DEBOUNCE_MS) {
            last_button_press = millis();
            label_tag = (label_tag == 1) ? 2 : 1;
            ESP_LOGI(TAG, "New specimen, label tag is now %u", label_tag);
        }

        if (!bme690_fetch_data(&sensor)) {
            continue;
        }

        do {
            n_fields_left = bme690_get_data(&sensor, &data);

            // Skip the fields which hold no valid gas measurement
            if (data.status != BME690_VALID_DATA) {
                continue;
            }

            // One scanning cycle is one full sweep through the heater profile,
            // so the counter moves on whenever the step index wraps around.
            if (data.gas_index <= last_step_index) {
                scanning_cycle_index++;
            }

            // A step which is not the one after the previous step means the
            // measurement in between was lost. The format reports that as error
            // code 2, and AI-Studio drops the whole cycle on import.
            uint8_t error_code = 0;
            if (data.gas_index != (last_step_index + 1) % HEATER_LEN) {
                error_code = 2;
                dropped_cycles++;
            }

            last_step_index = data.gas_index;

            // The format wants the pressure in hectopascals, the sensor reports
            // it in pascals.
            snprintf(row, sizeof(row), "%s[%d,%lu,%lu,%lu,%f,%f,%f,%f,%d,%d,%lu,%d,%d]", rows == 0 ? "" : ",",
                     SENSOR_INDEX, (unsigned long)SENSOR_ID, (unsigned long)(millis() - start),
                     (unsigned long)time(NULL), data.temperature, data.pressure / 100.0f, data.humidity,
                     data.gas_resistance, data.gas_index, 1, (unsigned long)scanning_cycle_index, label_tag,
                     error_code);
            fputs(row, file);
            rows++;

            ESP_LOGI(TAG, "T: %.2f C | P: %.2f hPa | H: %.2f %% | R: %.2f Ohm | step: %u", data.temperature,
                     data.pressure / 100.0f, data.humidity, data.gas_resistance, data.gas_index);
        } while (n_fields_left);
    }

    // Close the data block, the rawDataBody object and the root object
    fputs("]}}", file);
    fclose(file);

    ESP_LOGI(TAG, "Recording finished, %lu rows written, %lu cycles marked as lost.", (unsigned long)rows,
             (unsigned long)dropped_cycles);
    ESP_LOGI(TAG, "File saved as: %s", file_name);
    ESP_LOGI(TAG, "Import it into BME AI-Studio to label the data and train an algorithm.");

    bme690_set_op_mode(&sensor, BME69X_SLEEP_MODE);
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
    esp_wifi_disconnect();
    esp_wifi_stop();
}
