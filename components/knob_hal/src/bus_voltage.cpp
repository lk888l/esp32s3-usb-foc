#include "bus_voltage.hpp"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include <cmath>

#if CONFIG_KNOB_BUS_VOLTAGE_SENSE && !CONFIG_IDF_TARGET_ESP32S3
#error "Revalidate the independent ADC2 bus monitor before changing chip"
#endif

namespace knob {
esp_err_t BusVoltageMonitor::initialize() {
#if CONFIG_KNOB_BUS_VOLTAGE_SENSE
    shutdown();
    adc_oneshot_unit_init_cfg_t unit{};
    unit.unit_id = ADC_UNIT_2;
    unit.ulp_mode = ADC_ULP_MODE_DISABLE;
    esp_err_t error = adc_oneshot_new_unit(&unit, &adc_);
    if (error == ESP_OK) {
        adc_oneshot_chan_cfg_t channel{};
        channel.atten = ADC_ATTEN_DB_12;
        channel.bitwidth = ADC_BITWIDTH_12;
        error = adc_oneshot_config_channel(adc_, ADC_CHANNEL_0, &channel);
    }
    if (error == ESP_OK) {
        adc_cali_curve_fitting_config_t calibration{};
        calibration.unit_id = ADC_UNIT_2;
        calibration.chan = ADC_CHANNEL_0;
        calibration.atten = ADC_ATTEN_DB_12;
        calibration.bitwidth = ADC_BITWIDTH_12;
        error = adc_cali_create_scheme_curve_fitting(&calibration, &calibration_);
    }
    if (error != ESP_OK) shutdown();
    initialization_error_ = error;
    return error;
#else
    initialization_error_ = ESP_ERR_NOT_SUPPORTED;
    return ESP_OK;
#endif
}

BusVoltageSample BusVoltageMonitor::read() {
    BusVoltageSample sample;
    sample.error = initialization_error_;
#if CONFIG_KNOB_BUS_VOLTAGE_SENSE
    if (!adc_ || !calibration_) return sample;
    const int64_t started = esp_timer_get_time();
    sample.error = adc_oneshot_read(adc_, ADC_CHANNEL_0, &sample.raw);
    const int64_t finished = esp_timer_get_time();
    sample.timestamp_us = started + (finished - started) / 2;
    if (sample.error != ESP_OK) return sample;
    sample.error = adc_cali_raw_to_voltage(calibration_, sample.raw, &sample.millivolts);
    if (sample.error != ESP_OK) return sample;
    sample.volts = sample.millivolts * 0.001f * divider_ratio;
    // A clipped or out-of-range reading must never become a manual fallback.
    // A floating pin can look plausible; wiring must be confirmed before enabling.
    sample.valid = sample.raw > 0 && sample.raw < 4095 && sample.millivolts > 0 &&
        std::isfinite(sample.volts) && sample.volts >= 4.5f && sample.volts <= 30.0f;
    if (!sample.valid) sample.error = ESP_ERR_INVALID_RESPONSE;
#endif
    return sample;
}

void BusVoltageMonitor::shutdown() {
    if (calibration_) {
        adc_cali_delete_scheme_curve_fitting(calibration_);
        calibration_ = nullptr;
    }
    if (adc_) {
        adc_oneshot_del_unit(adc_);
        adc_ = nullptr;
    }
    initialization_error_ = ESP_ERR_NOT_FINISHED;
}
}
