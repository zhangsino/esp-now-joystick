idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES common esp_driver_gpio esp_driver_rmt esp_timer
                                  esp_wifi esp_netif esp_event esp_app_format nvs_flash usb)
