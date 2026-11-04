#include "bus_voltage.hpp"
#include "esp_adc/adc_cali_scheme.h"
#include <cassert>
#include <cmath>
#include <iostream>

namespace {
int units = 0, calibrations = 0, reads = 0;
int fail_stage = 0;
int next_raw = 2100, next_mv = 1764;
int64_t now_us = 1000;
int unit_token = 0, calibration_token = 0;
}

esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t* config, adc_oneshot_unit_handle_t* handle) {
    assert(config->unit_id == ADC_UNIT_2);
    if (fail_stage == 1) return ESP_ERR_NO_MEM;
    assert(units == 0); ++units; *handle = &unit_token; return ESP_OK;
}
esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t handle, adc_channel_t channel,
                                    const adc_oneshot_chan_cfg_t* config) {
    assert(handle == &unit_token && channel == ADC_CHANNEL_0);
    assert(config->atten == ADC_ATTEN_DB_12 && config->bitwidth == ADC_BITWIDTH_12);
    return fail_stage == 2 ? ESP_ERR_NOT_SUPPORTED : ESP_OK;
}
esp_err_t adc_oneshot_del_unit(adc_oneshot_unit_handle_t handle) {
    assert(handle == &unit_token && units == 1); --units; return ESP_OK;
}
esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t handle, adc_channel_t channel, int* raw) {
    assert(handle == &unit_token && channel == ADC_CHANNEL_0);
    ++reads; now_us += 20; *raw = next_raw;
    return fail_stage == 4 ? ESP_ERR_TIMEOUT : ESP_OK;
}
esp_err_t adc_cali_create_scheme_curve_fitting(const adc_cali_curve_fitting_config_t* config,
                                              adc_cali_handle_t* handle) {
    assert(config->unit_id == ADC_UNIT_2 && config->chan == ADC_CHANNEL_0);
    assert(config->atten == ADC_ATTEN_DB_12 && config->bitwidth == ADC_BITWIDTH_12);
    if (fail_stage == 3) return ESP_ERR_NOT_SUPPORTED;
    assert(calibrations == 0); ++calibrations; *handle = &calibration_token; return ESP_OK;
}
esp_err_t adc_cali_delete_scheme_curve_fitting(adc_cali_handle_t handle) {
    assert(handle == &calibration_token && calibrations == 1); --calibrations; return ESP_OK;
}
esp_err_t adc_cali_raw_to_voltage(adc_cali_handle_t handle, int raw, int* mv) {
    assert(handle == &calibration_token && raw == next_raw);
    *mv = next_mv; return fail_stage == 5 ? ESP_ERR_NOT_SUPPORTED : ESP_OK;
}
int64_t esp_timer_get_time() { return now_us; }

int main() {
    using knob::BusVoltageMonitor;
    {
        BusVoltageMonitor monitor;
        assert(!monitor.read().valid && reads == 0);
#if CONFIG_KNOB_BUS_VOLTAGE_SENSE
        static_assert(BusVoltageMonitor::enabled && BusVoltageMonitor::gpio == 11);
        // Every partial initialization failure releases the resources it owns.
        for (fail_stage = 1; fail_stage <= 3; ++fail_stage) {
            assert(monitor.initialize() != ESP_OK);
            const auto sample = monitor.read();
            assert(!sample.valid && sample.error != ESP_OK && reads == 0);
            assert(units == 0 && calibrations == 0);
        }
        fail_stage = 0;
        assert(monitor.initialize() == ESP_OK);
        assert(monitor.initialize() == ESP_OK); // Re-initialization must not leak.
        const auto good = monitor.read();
        assert(good.valid && good.error == ESP_OK);
        assert(std::abs(good.volts - 19.404f) < .001f);
        assert(good.timestamp_us == 1010 && good.raw == next_raw && good.millivolts == 1764);
        assert(good.usable(good.timestamp_us));
        assert(good.usable(good.timestamp_us + knob::BusVoltageSample::max_age_us));
        assert(!good.usable(good.timestamp_us + knob::BusVoltageSample::max_age_us + 1));
        assert(!good.usable(good.timestamp_us - 1));
        auto invalid = good;
        invalid.valid = false;
        assert(!invalid.usable(good.timestamp_us));
        invalid = good; invalid.error = ESP_ERR_TIMEOUT;
        assert(!invalid.usable(good.timestamp_us));
        invalid = good; invalid.timestamp_us = 0;
        assert(!invalid.usable(0));
        for (const int mv : {0, 409, 2728, 3100}) {
            next_mv = mv;
            const auto sample = monitor.read();
            assert(!sample.valid && sample.error == ESP_ERR_INVALID_RESPONSE);
        }
        for (const int mv : {410, 2727}) {
            next_mv = mv; assert(monitor.read().valid);
        }
        next_mv = 1764;
        for (const int raw : {0, 4095}) {
            next_raw = raw; assert(!monitor.read().valid);
        }
        next_raw = 2100;
        for (fail_stage = 4; fail_stage <= 5; ++fail_stage) {
            const auto sample = monitor.read();
            assert(!sample.valid && sample.error != ESP_OK);
        }
        fail_stage = 0;
        assert(monitor.read().valid); // A failed read does not reuse a stale valid reading.
        monitor.shutdown();
        assert(!monitor.read().valid && units == 0 && calibrations == 0);
        assert(monitor.initialize() == ESP_OK);
#else
        static_assert(!BusVoltageMonitor::enabled);
        assert(monitor.initialize() == ESP_OK);
        const auto sample = monitor.read();
        assert(!sample.valid && sample.error == ESP_ERR_NOT_SUPPORTED);
        assert(units == 0 && calibrations == 0 && reads == 0);
#endif
    }
    assert(units == 0 && calibrations == 0); // Destructor releases an initialized adapter.
    std::cout << "Bus voltage monitor checks passed\n";
}
