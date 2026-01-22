idf_component_register(SRCS
                        "esp_ot_rcp.c"
                        "ws2812.c"
                        "rcp_led_status.c"
                       PRIV_REQUIRES esp_coex esp_event nvs_flash openthread esp_timer esp_driver_rmt
                       INCLUDE_DIRS ".")
