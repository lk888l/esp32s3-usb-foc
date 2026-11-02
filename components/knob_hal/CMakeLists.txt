idf_component_register(SRCS "src/board.cpp" INCLUDE_DIRS "include"
    REQUIRES knob_core esp_driver_mcpwm esp_driver_i2c esp_driver_gpio esp_adc esp_timer)
