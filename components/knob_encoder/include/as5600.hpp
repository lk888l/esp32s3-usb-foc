#pragma once
#include "encoder_types.hpp"
#include "driver/i2c_master.h"

namespace knob {
class As5600 {
public:
    As5600() = default;
    As5600(const As5600&) = delete;
    As5600& operator=(const As5600&) = delete;
    esp_err_t initialize();
    void shutdown();
    esp_err_t read(EncoderSample& sample);
    const char* failed_operation() const { return failed_operation_; }
    int transport_error() const { return transport_error_; }
    uint32_t retries() const { return retries_; }
private:
    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t device_ = nullptr;
    const char* failed_operation_ = "none";
    int transport_error_ = ESP_OK;
    uint32_t retries_ = 0;
};
} // namespace knob
