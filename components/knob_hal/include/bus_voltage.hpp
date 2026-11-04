#pragma once
#include "sdkconfig.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include <cstdint>

namespace knob {
struct BusVoltageSample {
    static constexpr int64_t max_age_us = 100000;
    float volts = 0;
    int raw = 0;
    int millivolts = 0;
    int64_t timestamp_us = 0;
    esp_err_t error = ESP_ERR_NOT_FINISHED;
    bool valid = false;
    constexpr bool usable(int64_t now_us) const {
        return valid && error == ESP_OK && timestamp_us > 0 && now_us >= timestamp_us &&
            now_us - timestamp_us <= max_age_us;
    }
};

// Own this adapter from one core-0 task. ADC2 uses its RTC oneshot controller;
// ADC1's PWM-synchronous digital controller and DMA pattern remain untouched.
// read() polls the ADC, so it must never run in the motor task or an ISR.
class BusVoltageMonitor {
public:
#if CONFIG_KNOB_BUS_VOLTAGE_SENSE
    static constexpr bool enabled = true;
    static constexpr float divider_ratio = CONFIG_KNOB_BUS_DIVIDER_MILLI * 0.001f;
#else
    static constexpr bool enabled = false;
    static constexpr float divider_ratio = 11.0f;
#endif
    static constexpr int gpio = 11; // ADC2_CH0; H1-10 divided bus, never VM directly.
    static constexpr uint32_t sample_period_us = 10000;

    BusVoltageMonitor() = default;
    ~BusVoltageMonitor() { shutdown(); }
    BusVoltageMonitor(const BusVoltageMonitor&) = delete;
    BusVoltageMonitor& operator=(const BusVoltageMonitor&) = delete;

    esp_err_t initialize();
    BusVoltageSample read();
    void shutdown();

private:
    adc_oneshot_unit_handle_t adc_ = nullptr;
    adc_cali_handle_t calibration_ = nullptr;
    esp_err_t initialization_error_ = ESP_ERR_NOT_FINISHED;
};
}
