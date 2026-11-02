#pragma once
#include "control.hpp"
#include "driver/i2c_master.h"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>
namespace knob {
struct RawSample {
    int raw[2]{};
    uint8_t phase[2]{};
    bool valid = false;
    int64_t timestamp_us = 0;
    uint32_t duration_ticks = 0, rejected = 0;
    int start_tick = 0, end_tick = 0;
};
class Board {
public:
    static constexpr int pwm_gpio[3] = {15, 16, 17};
    static constexpr int enable_gpio = 18;
    // Fast ADC pair measured at 18-19 us; 20 kHz has a 30 us common low vector.
    static constexpr uint32_t resolution = 80'000'000, half_period = 2000;
    esp_err_t initialize();
    void shutdown();
    esp_err_t start_sampling(TaskHandle_t task);
    esp_err_t read_angle(float& angle, uint8_t& status);
    esp_err_t write_duty(const std::array<float, 3>& duty);
    void enable(bool enabled);
    RawSample sample();
    RawSample trip_sample();
    float volts(unsigned phase, int raw) const;
    void set_trip(const std::array<float, 3>& zero, float gain, float amperes);
    void reset_trip();
    int trip() const { return trip_.load(); }
    const char* failed_operation() const { return failed_operation_; }
    int initialization_error() const { return initialization_error_; }
    int encoder_transport_error() const { return encoder_transport_error_; }
    uint32_t encoder_retries() const { return encoder_retries_; }
    void kick() { watchdog_us_.store(static_cast<uint32_t>(esp_time())); }
    static int64_t esp_time();
private:
    static bool on_empty(mcpwm_timer_handle_t, const mcpwm_timer_event_data_t*, void* context);
    static bool on_sample(mcpwm_cmpr_handle_t, const mcpwm_compare_event_data_t*, void* context);
    static bool read_current_fast(unsigned channel, int& raw);
    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t encoder_ = nullptr;
    adc_oneshot_unit_handle_t adc_ = nullptr;
    adc_cali_handle_t cali_[3]{};
    std::array<std::array<uint16_t, 4096>, 3> millivolts_{};
    mcpwm_timer_handle_t timer_ = nullptr;
    mcpwm_oper_handle_t operators_[3]{};
    mcpwm_cmpr_handle_t comparators_[3]{};
    mcpwm_cmpr_handle_t sample_comparator_ = nullptr;
    mcpwm_gen_handle_t generators_[3]{};
    TaskHandle_t task_ = nullptr;
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    RawSample sample_{};
    RawSample trip_sample_{};
    std::array<int32_t, 3> offset_uv_{};
    int32_t trip_uv_ = 0, absolute_trip_uv_ = 0;
    CurrentTripFilter current_trip_filter_;
    std::array<uint32_t, 3> pending_{1999,1999,1999};
    std::atomic<bool> enabled_{false}, trip_ready_{false};
    std::atomic<int> trip_{0};
    std::atomic<uint32_t> watchdog_us_{0};
    uint32_t divider_ = 0, pair_ = 0, rejected_ = 0;
    bool timer_enabled_ = false, timer_running_ = false;
    bool duty_pending_ = true;
    const char* failed_operation_ = "none";
    int initialization_error_ = 0;
    int encoder_transport_error_ = 0;
    uint32_t encoder_retries_ = 0;
};
}
