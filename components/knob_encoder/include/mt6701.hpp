#pragma once
#include "encoder_types.hpp"
#include "driver/spi_master.h"

namespace knob {
class Mt6701 {
public:
    Mt6701() = default;
    Mt6701(const Mt6701&) = delete;
    Mt6701& operator=(const Mt6701&) = delete;
    esp_err_t initialize();
    void shutdown(); // Call only after the encoder worker has stopped.
    esp_err_t read(EncoderSample& sample);
    const char* failed_operation() const { return failed_operation_; }
    int transport_error() const { return transport_error_; }
    uint32_t retries() const { return 0; }
private:
    static constexpr spi_host_device_t host = SPI2_HOST;
    spi_device_handle_t device_ = nullptr;
    uint8_t* rx_ = nullptr;
    spi_transaction_t transaction_{};
    bool bus_initialized_ = false, pending_ = false;
    const char* failed_operation_ = "none";
    int transport_error_ = ESP_OK;
    esp_err_t finish(TickType_t wait);
};
} // namespace knob
