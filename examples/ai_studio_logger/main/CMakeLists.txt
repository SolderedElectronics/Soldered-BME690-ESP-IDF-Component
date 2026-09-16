idf_component_register(SRCS "main.c"
                    REQUIRES esp_wifi esp_netif esp_event esp_timer nvs_flash
                             esp_driver_gpio esp_driver_spi esp_driver_sdspi fatfs sdmmc)
