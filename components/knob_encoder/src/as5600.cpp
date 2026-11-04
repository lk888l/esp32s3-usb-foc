#include "as5600.hpp"
#include "encoder_config.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

namespace knob {
esp_err_t As5600::initialize() {
    if (bus_ || device_) return ESP_ERR_INVALID_STATE;
    failed_operation_ = "none"; transport_error_ = ESP_OK; retries_ = 0;
    const auto check = [this](esp_err_t error, const char* operation) {
        if (error != ESP_OK) { failed_operation_ = operation; shutdown(); }
        return error;
    };
    i2c_master_bus_config_t bc{};
    bc.i2c_port = I2C_NUM_0;
    bc.sda_io_num = static_cast<gpio_num_t>(encoder_config::as5600_sda_gpio);
    bc.scl_io_num = static_cast<gpio_num_t>(encoder_config::as5600_scl_gpio);
    bc.clk_source = I2C_CLK_SRC_DEFAULT; bc.glitch_ignore_cnt = 7; bc.flags.enable_internal_pullup = true;
    bc.intr_priority = 2;
    esp_err_t error = check(i2c_new_master_bus(&bc, &bus_), "AS5600 I2C bus");
    if (error != ESP_OK) return error;
    i2c_device_config_t dc{};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7; dc.device_address = 0x36;
    dc.scl_speed_hz = encoder_config::as5600_clock_hz;
    error = check(i2c_master_bus_add_device(bus_, &dc, &device_), "AS5600 I2C device");
    if (error != ESP_OK) return error;
    uint8_t reg = 0x07, conf[2]{};
    // A USB reset may interrupt an I2C transfer while the sensor stays powered.
    for (int attempt = 0; attempt < 3; ++attempt) {
        error = i2c_master_transmit_receive(device_, &reg, 1, conf, 2, 10);
        if (error == ESP_OK) break;
        if (attempt < 2) vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (check(error, "AS5600 startup config read") != ESP_OK) return error;
    // Original volatile SF=3, FTH=1, PM=0, HYST=0; never write/burn OTP.
    const uint8_t settings[] = {0x07, static_cast<uint8_t>((conf[0] & 0xe0) | 0x07), static_cast<uint8_t>(conf[1] & 0xf0)};
    return check(i2c_master_transmit(device_, settings, sizeof(settings), 10), "AS5600 volatile config write");
}
esp_err_t As5600::read(EncoderSample& sample) {
    sample = {};
    if (!device_) {
        sample.fault = EncoderFault::transport;
        return transport_error_ = ESP_ERR_INVALID_STATE;
    }
    const uint8_t reg = 0x0b; uint8_t data[3]{};
    const int64_t started = esp_timer_get_time();
    esp_err_t error = i2c_master_transmit_receive(device_, &reg, 1, data, sizeof(data), 2);
    if (error != ESP_OK && esp_timer_get_time() - started < 500) {
        ++retries_;
        error = i2c_master_transmit_receive(device_, &reg, 1, data, sizeof(data), 2);
    }
    if (error == ESP_OK && esp_timer_get_time() - started > 1000) error = ESP_ERR_TIMEOUT;
    transport_error_ = error;
    if (error != ESP_OK) { sample.fault = EncoderFault::transport; return error; }
    sample.magnet = data[0];
    sample.angle = ((data[1] & 0x0f) * 256 + data[2]) * (6.28318530717958647692f / 4096.0f);
#if CONFIG_KNOB_ALLOW_WEAK_MAGNET
    constexpr uint8_t rejected_flags = 0x08;
#else
    constexpr uint8_t rejected_flags = 0x18;
#endif
    if (!(sample.magnet & 0x20)) sample.fault = EncoderFault::missing_magnet;
    else if (sample.magnet & rejected_flags) sample.fault = EncoderFault::magnet_strength;
    return sample.fault == EncoderFault::none ? ESP_OK : ESP_ERR_INVALID_STATE;
}
void As5600::shutdown() {
    if (device_) { i2c_master_bus_rm_device(device_); device_ = nullptr; }
    if (bus_) { i2c_del_master_bus(bus_); bus_ = nullptr; }
}
} // namespace knob
