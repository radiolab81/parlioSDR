idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_eth esp_netif esp_event nvs_flash esp_mm esp_driver_parlio esp_driver_gpio lwip driver ip101)