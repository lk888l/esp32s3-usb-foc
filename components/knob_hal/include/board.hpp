#pragma once
#include "sdkconfig.h"
#include "control.hpp"
#include "pwm_timing.hpp"
#include "encoder.hpp"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>
namespace knob {
struct RawSample {
    int raw[2]{};
    uint32_t dma_words[2]{};
    uint32_t adc_slot_words[3]{};
    int16_t adc_trigger_ticks[3]{}, adc_callback_ticks[3]{};
    uint16_t adc_callback_elapsed_us[3]{};
    uint16_t adc_trigger_elapsed_us[3]{};
    uint16_t adc_pwm_compare_before[3][3]{}, adc_pwm_compare_after[3][3]{};
    uint16_t phase_skew_us = 0;
    uint8_t adc_completed_mask = 0, adc_window_after_mask = 0;
    uint8_t adc_trigger_sar[3]{}, adc_callback_sar[3]{};
    uint8_t adc_slot_mask = 0, adc_trigger_down_mask = 0, adc_callback_down_mask = 0;
    // Software mirror of the TEZ-latched compares, before/after the burst TEZ.
    // Reading the comparator register itself returns its shadow value.
    uint16_t pwm_compare_before[3]{}, pwm_compare_after[3]{};
    uint32_t pwm_latch_sequence = 0;
    bool pwm_compare_valid = false;
    uint32_t sequence = 0;
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
    static constexpr uint32_t resolution = 80'000'000, half_period = resolution / (2 * pwm_profile_hz);
    static constexpr uint32_t pwm_hz = resolution / (2 * half_period), encoder_hz = 2500;
    static constexpr bool staggered_sampling = pwm_hz == 20000;
    static constexpr uint32_t sampling_phase_skew_us = staggered_sampling ? 50 : 0;
    static constexpr uint32_t control_hz = CONFIG_KNOB_CONTROL_HZ, control_period_us = 1000000 / control_hz;
    static_assert(control_hz == 2000 || control_hz == 5000);
    static_assert(pwm_hz % control_hz == 0 && pwm_hz % encoder_hz == 0);
    esp_err_t initialize();
    esp_err_t initialize_encoder();
    void shutdown();
    esp_err_t start_sampling(TaskHandle_t task);
    esp_err_t read_encoder(EncoderSample& sample);
    void set_encoder_task(TaskHandle_t task) { encoder_task_.store(task); }
    IRAM_ATTR esp_err_t write_duty(const std::array<float, 3>& duty);
    IRAM_ATTR esp_err_t write_voltage(Dq voltage, float sample_electrical_angle,
        float electrical_velocity, int64_t sample_timestamp_us, float bus_voltage,
        std::array<float, 3>& duty, float& apply_delay_us);
    IRAM_ATTR void enable(bool enabled);
    IRAM_ATTR RawSample sample();
    IRAM_ATTR RawSample trip_sample();
    IRAM_ATTR float volts(unsigned phase, int raw) const;
    void set_trip(const std::array<float, 3>& zero, float gain, float amperes);
    void reset_trip();
    int trip() const { return trip_.load(); }
    bool sampling_restart_required() const { return adc_desynchronized_ || pwm_tracking_failed_; }
    const char* failed_operation() const { return failed_operation_; }
    int initialization_error() const { return initialization_error_; }
    int encoder_transport_error() const { return encoder_.transport_error(); }
    uint32_t encoder_retries() const { return encoder_.retries(); }
    void kick() { watchdog_us_.store(static_cast<uint32_t>(esp_time())); }
    IRAM_ATTR static int64_t esp_time();
private:
    static bool on_empty(mcpwm_timer_handle_t, const mcpwm_timer_event_data_t*, void* context);
    static bool on_full(mcpwm_timer_handle_t, const mcpwm_timer_event_data_t*, void* context);
    static bool on_sample(mcpwm_cmpr_handle_t, const mcpwm_compare_event_data_t*, void* context);
    static bool on_adc_done(adc_continuous_handle_t, const adc_continuous_evt_data_t*, void* context);
    void start_requested_conversion();
    bool publish_sample(RawSample sample);
    Encoder encoder_;
    adc_continuous_handle_t adc_ = nullptr;
    bool adc_started_ = false, adc_pending_ = false;
    bool adc_desynchronized_ = false;
    RawSample pending_sample_{};
    adc_cali_handle_t cali_[3]{};
    std::array<std::array<uint16_t, 4096>, 3> millivolts_{};
    mcpwm_timer_handle_t timer_ = nullptr;
    mcpwm_oper_handle_t operators_[3]{};
    mcpwm_cmpr_handle_t comparators_[3]{};
    mcpwm_cmpr_handle_t sample_comparator_ = nullptr;
    mcpwm_gen_handle_t generators_[3]{};
    TaskHandle_t task_ = nullptr;
    std::atomic<TaskHandle_t> encoder_task_{nullptr};
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    RawSample sample_{};
    RawSample trip_sample_{};
    std::array<int32_t, 3> offset_uv_{};
    int32_t trip_uv_ = 0, peak_trip_uv_ = 0, absolute_trip_uv_ = 0;
    CurrentTripFilter current_trip_filter_;
    std::array<uint32_t, 3> pending_{half_period - 1, half_period - 1, half_period - 1};
    std::array<uint32_t, 3> shadow_compare_ = pending_, active_compare_ = pending_;
    uint32_t pwm_latch_sequence_ = 0;
    int64_t last_tez_us_ = 0;
    int64_t last_tep_us_ = 0;
    bool pwm_compare_valid_ = false, pwm_tracking_failed_ = false;
    std::atomic<bool> enabled_{false}, trip_ready_{false};
    std::atomic<int> trip_{0};
    std::atomic<uint32_t> watchdog_us_{0};
    uint32_t divider_ = 0, rejected_ = 0;
    uint32_t encoder_divider_ = 0, sequence_ = 0;
    unsigned adc_stage_ = 0, adc_received_ = 0;
    unsigned adc_requested_stage_ = 0;
    int64_t adc_last_start_us_ = 0;
    PwmForceLowState force_low_{};
    bool timer_enabled_ = false, timer_running_ = false;
    bool duty_pending_ = true;
    const char* failed_operation_ = "none";
    int initialization_error_ = 0;
};
}
