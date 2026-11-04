idf_component_register(SRCS "src/as5600.cpp" "src/mt6701.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c esp_driver_spi esp_driver_gpio esp_timer)
