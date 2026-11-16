idf_component_register(SRCS "src/usb_device.cpp" INCLUDE_DIRS "include"
    REQUIRES esp_tinyusb esp_timer esp_hw_support app)
