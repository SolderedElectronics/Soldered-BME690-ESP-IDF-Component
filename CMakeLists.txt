idf_component_register(SRCS "src/soldered_bme690.c"
                            "src/bme69x/bme69x.c"
                    INCLUDE_DIRS "include" "include/bme69x"
                    REQUIRES esp_driver_i2c)
