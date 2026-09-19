idf_component_register(
    SRCS "cube_client.c" "rgb_led.c" "main.c" "gan_crypto.c" "cube_state.c" "gan_proto.c" "gan_gatt.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash bt driver esp_timer esp_driver_uart esp_driver_gpio esp_driver_rmt mbedtls esp_hw_support)
