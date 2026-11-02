#include "app_manager.hpp"
#include "knob_app.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
extern "C" void app_main() {
    // Establish the off state before NVS, allocation, or task startup can fail.
    gpio_config_t pins{};
    pins.pin_bit_mask = (1ULL << 15) | (1ULL << 16) | (1ULL << 17) | (1ULL << 18);
    pins.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&pins));
    for (int pin : {15, 16, 17, 18}) gpio_set_level(static_cast<gpio_num_t>(pin), 0);
    app::AppManager manager;
    if (!manager.register_module(knob::create_module()) || !manager.initialize_all()) {
        ESP_LOGE("smartknob", "initialization failed; driver remains disabled");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    while (true) { manager.process_all(); vTaskDelay(pdMS_TO_TICKS(100)); }
}
