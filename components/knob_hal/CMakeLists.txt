idf_component_register(SRCS "src/board.cpp" "src/bus_voltage.cpp" INCLUDE_DIRS "include"
    REQUIRES knob_core knob_encoder esp_driver_mcpwm esp_driver_gpio esp_adc esp_timer)
