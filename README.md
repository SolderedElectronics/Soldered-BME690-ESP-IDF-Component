# Soldered BME690 Environmental Sensor Component

| ![BME690 Environmental sensor breakout](https://upload.wikimedia.org/wikipedia/commons/8/8f/Example_image.svg) |
| :---------------------------------------------------------------------------------------------------------------------------------------------------------------: |
|                                       [BME690 Environmental sensor breakout](https://www.solde.red/333411)                                       |

ESP-IDF component for the Soldered BME690 breakout board. The BME690 is a Bosch environmental sensor which measures temperature, pressure, relative humidity and gas resistance, the last one usable for air quality and, with BME AI-Studio, for gas classification. The board connects over I2C and is part of the [Qwiic ecosystem](https://soldered.com/collections/qwiic-ecosystem), so no soldering is needed to hook it up.


### Installation

Add it to your project with the component manager:

```bash
idf.py add-dependency "solderedelectronics/soldered-bme690"
```

Or clone this repository into your project's `components/` folder.

### Usage

The I2C bus belongs to your application, not to the driver, so that other Qwiic devices can share it. Create the bus first, then hand it over:

```c
#include "driver/i2c_master.h"
#include "soldered_bme690.h"

i2c_master_bus_config_t bus_cfg = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = GPIO_NUM_21,
    .scl_io_num = GPIO_NUM_22,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};
i2c_master_bus_handle_t bus;
ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

bme690_t sensor;
ESP_ERROR_CHECK(bme690_init(&sensor, bus, BME69X_I2C_ADDR_LOW));

bme690_set_tph_default(&sensor);
bme690_set_filter(&sensor, BME69X_FILTER_SIZE_3);
bme690_set_heater_prof(&sensor, 300, 100);

bme690_set_op_mode(&sensor, BME69X_FORCED_MODE);
vTaskDelay(pdMS_TO_TICKS(200));

bme690_data_t data;
if (bme690_fetch_data(&sensor)) {
    bme690_get_data(&sensor, &data);
    printf("%.2f C, %.2f Pa, %.2f %%, %.2f Ohm\n", data.temperature, data.pressure, data.humidity,
           data.gas_resistance);
}
```

Use `BME69X_I2C_ADDR_HIGH` instead if the address jumper on the board is soldered. `bme690_init_with_config()` takes a different I2C clock or a custom idle callback.

Every call leaves the Bosch API result code in the handle, so `bme690_check_status()` and `bme690_status_string()` describe what went wrong after any of them.

### Examples

- **forced_mode** - one measurement at a time, the mode most applications want
- **sequential_mode** - the sensor steps through a heater profile on its own, sleeping in between
- **parallel_mode** - the gas sensor sweeps a heater profile while temperature, pressure and humidity are measured continuously
- **self_test** - runs the built-in self test and prints the unique sensor ID
- **ai_studio_logger** - records a `.bmerawdata` file on an SD card in the Bosch BME AI-Studio Raw Data Format, ready to be imported into AI-Studio to train a gas classification algorithm. Needs WiFi for the real time clock and an SD card module on the SPI pins

Build any of them with:

```bash
cd examples/forced_mode
idf.py set-target esp32
idf.py build flash monitor
```

### Repository Contents

- **/src** - source files (.c), with the unmodified Bosch BME69x Sensor API in `src/bme69x/`
- **/include** - header files (.h), with the Bosch API headers in `include/bme69x/`
- **/examples** - examples for using the library
- **_other_** - idf_component.yml manifest file for ESP Component Registry


### Hardware design

You can find hardware design for this board in _BME690 Environmental sensor breakout_ hardware repository.

### Documentation

Access library documentation [here](https://docs.soldered.com/).

### About Soldered

<img src="https://raw.githubusercontent.com/SolderedElectronics/Soldered-Generic-Arduino-Library/dev/extras/Soldered-logo-color.png" alt="soldered-logo" width="500"/>

At Soldered, we design and manufacture a wide selection of electronic products to help you turn your ideas into acts and bring you one step closer to your final project. Our products are intented for makers and crafted in-house by our experienced team in Osijek, Croatia. We believe that sharing is a crucial element for improvement and innovation, and we work hard to stay connected with all our makers regardless of their skill or experience level. Therefore, all our products are open-source. Finally, we always have your back. If you face any problem concerning either your shopping experience or your electronics project, our team will help you deal with it, offering efficient customer service and cost-free technical support anytime. Some of those might be useful for you:

- [Web Store](https://www.soldered.com/shop)
- [Tutorials & Projects](https://soldered.com/learn)
- [Documentation](https://docs.soldered.com)

### Original source

This component is possible thanks to the original [BME69x Sensor API](https://github.com/boschsensortec/BME69x_SensorAPI) by Bosch Sensortec. Thank you, Bosch. The Bosch API is BSD-3-Clause licensed, its license is kept alongside the sources in `src/bme69x/LICENSE`.

### Open-source license

Soldered invests vast amounts of time into hardware & software for these products, which are all open-source. Please support future development by buying one of our products.

Check license details in the LICENSE file. Long story short, use these open-source files for any purpose you want to, as long as you apply the same open-source licence to it and disclose the original source. No warranty - all designs in this repository are distributed in the hope that they will be useful, but without any warranty. They are provided "AS IS", therefore without warranty of any kind, either expressed or implied. The entire quality and performance of what you do with the contents of this repository are your responsibility. In no event, Soldered (TAVU) will be liable for your damages, losses, including any general, special, incidental or consequential damage arising out of the use or inability to use the contents of this repository.

## Have fun!

And thank you from your fellow makers at Soldered Electronics.
