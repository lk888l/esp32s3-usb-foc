idf_component_register(SRCS "src/module.cpp" "src/protocol.cpp" "src/reference_json.cpp" INCLUDE_DIRS "include"
    REQUIRES app knob_core knob_hal knob_encoder json nvs_flash esp_driver_usb_serial_jtag esp_driver_uart esp_timer esp_app_format)
