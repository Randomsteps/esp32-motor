idf_component_register(SRCS "uart_example_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_uart esp_driver_gpio esp_wifi esp_http_server nvs_flash espressif__led_strip)
