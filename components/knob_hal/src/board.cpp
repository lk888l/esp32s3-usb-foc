#include "board.hpp"
#include "pwm_timing.hpp"
#include "driver/gpio.h"
#include "esp_adc/adc_cali_scheme.h"
#include "hal/adc_ll.h"
#include "soc/sens_struct.h"
#include "esp_idf_version.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "hal/mcpwm_ll.h"
#include "hal/gpio_ll.h"
#include "soc/mcpwm_struct.h"
#include "soc/gpio_struct.h"
#include <algorithm>
#include <cmath>
#if !CONFIG_IDF_TARGET_ESP32S3 || ESP_IDF_VERSION_MAJOR != 5 || ESP_IDF_VERSION_MINOR != 5
#error "Revalidate the PWM-synchronous ADC1 DMA adapter before changing chip or IDF series"
#endif
namespace knob {
#define TRY(call) do { const esp_err_t e = (call); if (e != ESP_OK) { failed_operation_ = #call; initialization_error_ = e; return e; } } while (false)
static IRAM_ATTR PwmCounter pwm_counter() {
    return decode_pwm_counter(MCPWM0.timer[0].timer_status.val, Board::half_period);
}
static IRAM_ATTR uint8_t sar_status() {
    return HAL_FORCE_READ_U32_REG_FIELD(SENS.sar_slave_addr1, meas_status);
}
static IRAM_ATTR bool apply_force_low_mask(uint8_t active, uint8_t next,
        const std::array<uint32_t, 3>& compare) {
    const uint8_t changed = active ^ next;
    if (!changed) return true;
    // Board owns MCPWM0 operators0/1/2 and each first generator(A). The
    // public force API is flash-resident, so retain the bounded LL writes.
    // These controls act immediately: a late TEZ ISR must not release force
    // after a phase's compare and thereby create a truncated high pulse.
    constexpr uint32_t margin_ticks = 5 * (Board::resolution / 1000000);
    if (!pwm_force_update_fits(pwm_counter(), changed, compare.data(), margin_ticks)) {
        gpio_ll_set_level(&GPIO, Board::enable_gpio, 0);
        return false;
    }
    for (int p = 0; p < 3; ++p) {
        if (changed & (1U << p)) {
            if (next & (1U << p)) mcpwm_ll_gen_set_continue_force_level(&MCPWM0, p, 0, 0);
            else mcpwm_ll_gen_disable_continue_force_action(&MCPWM0, p, 0);
        }
    }
    // Detect an unexpected interruption/stall despite the reserved margin.
    // Do not publish a trustworthy active mask when the edge was crossed.
    if (!pwm_force_update_fits(pwm_counter(), changed, compare.data(), 0)) {
        gpio_ll_set_level(&GPIO, Board::enable_gpio, 0);
        return false;
    }
    return true;
}
int64_t Board::esp_time() { return esp_timer_get_time(); }
esp_err_t Board::initialize() {
    failed_operation_ = "none"; initialization_error_ = 0;
    gpio_config_t gpio{};
    gpio.pin_bit_mask = 1ULL << enable_gpio; gpio.mode = GPIO_MODE_OUTPUT;
    TRY(gpio_config(&gpio)); enable(false);
    // setup_board can retry with the same Board after shutdown. Hardware is
    // about to be forced low, so all software mirrors must start in that state.
    pending_ = {half_period, half_period, half_period};
    shadow_compare_ = active_compare_ = pending_;
    force_low_ = {};
    duty_pending_ = true; pwm_compare_valid_ = pwm_tracking_failed_ = false;
    pwm_latch_sequence_ = 0; last_tez_us_ = last_tep_us_ = 0;
    adc_pending_ = adc_desynchronized_ = false; adc_stage_ = adc_received_ = adc_requested_stage_ = 0;
    adc_last_start_us_ = 0; pending_sample_ = {};
    adc_continuous_handle_cfg_t ac{};
    ac.max_store_buf_size = 256; ac.conv_frame_size = SOC_ADC_DIGI_RESULT_BYTES;
    // Consume the DMA frame directly in on_conv_done. The optional task-read
    // ring buffer may stay full; callbacks still receive every completed frame.
    ac.flags.flush_pool = false;
    TRY(adc_continuous_new_handle(&ac, &adc_));
    adc_digi_pattern_config_t pattern[9]{};
    // Each burst takes two useful phases and a trailing pipeline flush. Keep
    // the entire sequence programmed: the digital controller preloads the
    // analog mux, so changing a live pattern can mislabel the next conversion.
    constexpr uint8_t scan[9] = {3, 4, 5, 4, 5, 3, 5, 3, 4};
    for (int p = 0; p < 9; ++p) {
        pattern[p].atten = ADC_ATTEN_DB_12; pattern[p].channel = scan[p];
        pattern[p].unit = ADC_UNIT_1; pattern[p].bit_width = ADC_BITWIDTH_12;
    }
    for (int p = 0; p < 3; ++p) {
        adc_cali_curve_fitting_config_t calibration{};
        calibration.unit_id = ADC_UNIT_1; calibration.chan = static_cast<adc_channel_t>(3 + p);
        calibration.atten = ADC_ATTEN_DB_12; calibration.bitwidth = ADC_BITWIDTH_12;
        TRY(adc_cali_create_scheme_curve_fitting(&calibration, &cali_[p]));
        for (int raw = 0; raw < 4096; ++raw) {
            int mv = 0; TRY(adc_cali_raw_to_voltage(cali_[p], raw, &mv));
            millivolts_[p][raw] = std::clamp(mv, 0, 5000);
        }
    }
    adc_continuous_config_t config{};
    config.sample_freq_hz = 83333; config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2; config.pattern_num = 9; config.adc_pattern = pattern;
    TRY(adc_continuous_config(adc_, &config));
    adc_continuous_evt_cbs_t callbacks{}; callbacks.on_conv_done = on_adc_done;
    TRY(adc_continuous_register_event_callbacks(adc_, &callbacks, this));
    mcpwm_timer_config_t tc{};
    tc.group_id = 0; tc.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT; tc.resolution_hz = resolution;
    // IDF 5.5 defines period_ticks as the full up/down period, not the peak.
    tc.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN; tc.period_ticks = 2 * half_period; tc.intr_priority = 3;
    // Board exclusively owns MCPWM0; its first allocated timer is hardware timer 0.
    TRY(mcpwm_new_timer(&tc, &timer_));
    for (int p = 0; p < 3; ++p) {
        mcpwm_operator_config_t oc{}; oc.group_id = 0;
        TRY(mcpwm_new_operator(&oc, &operators_[p]));
        TRY(mcpwm_operator_connect_timer(operators_[p], timer_));
        mcpwm_comparator_config_t cc{}; cc.flags.update_cmp_on_tez = true;
        TRY(mcpwm_new_comparator(operators_[p], &cc, &comparators_[p]));
        TRY(mcpwm_comparator_set_compare_value(comparators_[p], half_period));
        mcpwm_generator_config_t gc{}; gc.gen_gpio_num = pwm_gpio[p];
        TRY(mcpwm_new_generator(operators_[p], &gc, &generators_[p]));
        TRY(mcpwm_generator_set_action_on_timer_event(generators_[p],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
        TRY(mcpwm_generator_set_action_on_compare_event(generators_[p],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators_[p], MCPWM_GEN_ACTION_HIGH)));
        TRY(mcpwm_generator_set_action_on_compare_event(generators_[p],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, comparators_[p], MCPWM_GEN_ACTION_LOW)));
        TRY(mcpwm_generator_set_force_level(generators_[p], 0, true));
    }
    mcpwm_comparator_config_t trigger{};
    trigger.intr_priority = 3;
    TRY(mcpwm_new_comparator(operators_[0], &trigger, &sample_comparator_));
    // Trigger 15 us before TEZ. The ISR explicitly verifies that ADC starts
    // at least 6 us after the last falling PWM edge (propagation + CSA settling).
    TRY(mcpwm_comparator_set_compare_value(sample_comparator_, staggered_sampling ? 560 : 1200));
    return ESP_OK;
}
esp_err_t Board::initialize_encoder() {
    const esp_err_t error = encoder_.initialize();
    if (error != ESP_OK) {
        failed_operation_ = encoder_.failed_operation(); initialization_error_ = error;
    }
    return error;
}
esp_err_t Board::start_sampling(TaskHandle_t task) {
    task_ = task;
    divider_ = encoder_divider_ = 0;
    TRY(adc_continuous_start(adc_)); adc_started_ = true;
    adc_ll_digi_trigger_disable();
    APB_SARADC.ctrl.start_force = 1;
    APB_SARADC.ctrl.start = 0;
    adc_ll_digi_convert_limit_enable(false);
    // The manual-START adapter has a primed DMA pipeline: the first START
    // after stopping the digital timer need not produce a frame. Prime a full
    // fixed pattern with PWM stopped, then drain all its callbacks. No frame
    // from this interval can own a subsequent PWM conversion.
    for (int i = 0; i < 9; ++i) {
        APB_SARADC.ctrl.start = 0;
        esp_rom_delay_us(2);
        APB_SARADC.ctrl.start = 1;
        esp_rom_delay_us(20);
    }
    APB_SARADC.ctrl.start = 0;
    vTaskDelay(pdMS_TO_TICKS(2));
    mcpwm_timer_event_callbacks_t cb{}; cb.on_empty = on_empty;
    if constexpr (staggered_sampling) cb.on_full = on_full;
    TRY(mcpwm_timer_register_event_callbacks(timer_, &cb, this));
    mcpwm_comparator_event_callbacks_t sample_cb{}; sample_cb.on_reach = on_sample;
    TRY(mcpwm_comparator_register_event_callbacks(sample_comparator_, &sample_cb, this));
    if constexpr (staggered_sampling)
        mcpwm_ll_intr_enable(&MCPWM0, MCPWM_LL_EVENT_CMP_EQUAL(0, 1), false);
    TRY(mcpwm_timer_enable(timer_)); timer_enabled_ = true;
    TRY(mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_NO_STOP)); timer_running_ = true;
    return ESP_OK;
}
bool IRAM_ATTR Board::on_full(mcpwm_timer_handle_t, const mcpwm_timer_event_data_t*, void* context) {
    auto& b = *static_cast<Board*>(context);
    const auto counter = pwm_counter();
    const bool on_time = counter.down && counter.count > 400;
    if (!on_time) b.pwm_tracking_failed_ = true;
    // The unused UP compare shares one interrupt bit with DOWN. Keep it
    // masked through the current conversion, then discard its status at TEP.
    mcpwm_ll_intr_clear_status(&MCPWM0, MCPWM_LL_EVENT_CMP_EQUAL(0, 1));
    mcpwm_ll_intr_enable(&MCPWM0, MCPWM_LL_EVENT_CMP_EQUAL(0, 1), true);
    const int64_t now = esp_timer_get_time();
    if (b.last_tep_us_ && now - b.last_tep_us_ > 1000000 / pwm_hz * 3 / 2)
        b.pwm_tracking_failed_ = true;
    b.last_tep_us_ = now;
    // Same-core motor task and MCPWM ISRs own these values. The zero-command
    // fallback can update its shadow here, away from the ADC completion.
    if (b.duty_pending_ && on_time) {
        for (int p = 0; p < 3; ++p) mcpwm_comparator_set_compare_value(b.comparators_[p], b.pending_[p]);
        b.shadow_compare_ = b.pending_; b.force_low_.commit_shadow(); b.duty_pending_ = false;
    }
    BaseType_t wake = pdFALSE;
    if (++b.encoder_divider_ >= pwm_hz / encoder_hz) {
        b.encoder_divider_ = 0;
        const auto encoder_task = b.encoder_task_.load(std::memory_order_relaxed);
        if (encoder_task) vTaskNotifyGiveFromISR(encoder_task, &wake);
    }
    return wake == pdTRUE;
}
void IRAM_ATTR Board::start_requested_conversion() {
    if constexpr (staggered_sampling) return;
    const unsigned previous = adc_stage_;
    if (!adc_pending_ || !adc_next_stage_ready(previous, adc_received_, adc_requested_stage_, pending_sample_.adc_completed_mask)) return;
    // The standard S3 digital conversion cadence is at most83.3kHz. Keep
    // >=12us between real STARTs, with1us for timestamp/register-write error.
    // Do not hold the interrupt lock while waiting; a higher-priority PWM
    // callback may request (or launch) this same stage in the meantime.
    while (esp_timer_get_time() - adc_last_start_us_ < 13) {
        if (!adc_pending_ || adc_stage_ != previous) return;
    }
    portENTER_CRITICAL_ISR(&mux_);
    if (!adc_pending_ || adc_stage_ != previous ||
        !adc_next_stage_ready(previous, adc_received_, adc_requested_stage_, pending_sample_.adc_completed_mask)) {
        portEXIT_CRITICAL_ISR(&mux_); return;
    }
    const auto sar = sar_status();
    adc_last_start_us_ = esp_timer_get_time();
    const auto trigger = pwm_counter();
    adc_stage_ = previous + 1;
    APB_SARADC.ctrl.start = 0; APB_SARADC.ctrl.start = 1;
    auto& s = pending_sample_;
    s.adc_trigger_ticks[previous] = trigger.signed_tick();
    s.adc_trigger_sar[previous] = sar;
    s.adc_trigger_elapsed_us[previous] = std::clamp<int64_t>(adc_last_start_us_ - s.timestamp_us, 0, 65535);
    if (trigger.down) s.adc_trigger_down_mask |= 1U << previous;
    if (sar != 0 || trigger.down) s.valid = false;
    s.end_tick = trigger.signed_tick();
    portEXIT_CRITICAL_ISR(&mux_);
}
bool IRAM_ATTR Board::on_empty(mcpwm_timer_handle_t, const mcpwm_timer_event_data_t*, void* context) {
    auto& b = *static_cast<Board*>(context);
    if constexpr (staggered_sampling) {
        // This path lies inside a 15 us common-low window. Leave wall-clock
        // diagnostics, shadow writes and encoder notification to TEP.
        const auto counter = pwm_counter();
        if (counter.down || counter.count >= half_period) b.pwm_tracking_failed_ = true;
        ++b.pwm_latch_sequence_;
        b.active_compare_ = b.shadow_compare_;
        if (apply_force_low_mask(b.force_low_.active, b.force_low_.shadow, b.active_compare_)) b.force_low_.latch();
        else b.pwm_tracking_failed_ = true;
        b.pwm_compare_valid_ = !b.pwm_tracking_failed_;
        if (b.adc_pending_ && b.adc_stage_ >= 1 && b.adc_stage_ <= 3) {
            const unsigned slot = b.adc_stage_ - 1;
            auto& sample = b.pending_sample_;
            if (!(sample.adc_window_after_mask & (1U << slot))) {
                for (int p = 0; p < 3; ++p) sample.adc_pwm_compare_after[slot][p] = b.active_compare_[p];
                sample.adc_window_after_mask |= 1U << slot;
                if (!b.pwm_compare_valid_ || sample.pwm_latch_sequence + slot + 1 != b.pwm_latch_sequence_)
                    sample.pwm_compare_valid = false;
            }
        }
        return false;
    }
    const bool trigger_second = !staggered_sampling && b.adc_pending_ && b.adc_stage_ == 1;
    if (trigger_second) {
        b.adc_requested_stage_ = 2;
        b.start_requested_conversion();
    }
    const auto counter = pwm_counter();
    const int64_t now = esp_timer_get_time();
    constexpr uint32_t period_us = 1000000 / pwm_hz;
    const uint32_t phase_ticks = counter.phase_ticks(half_period);
    // TEZ already moved the old shadow into hardware before this callback.
    // Never substitute the new shadow we are about to write for that value.
    portENTER_CRITICAL_ISR(&b.mux_);
    if (phase_ticks >= half_period || (b.last_tez_us_ && now - b.last_tez_us_ > period_us * 3 / 2))
        b.pwm_tracking_failed_ = true;
    b.last_tez_us_ = now - phase_ticks / (resolution / 1000000);
    ++b.pwm_latch_sequence_;
    b.active_compare_ = b.shadow_compare_;
    if (apply_force_low_mask(b.force_low_.active, b.force_low_.shadow, b.active_compare_)) b.force_low_.latch();
    else b.pwm_tracking_failed_ = true;
    b.pwm_compare_valid_ = !b.pwm_tracking_failed_;
    if constexpr (staggered_sampling) {
        if (b.adc_pending_ && b.adc_stage_ >= 1 && b.adc_stage_ <= 3) {
            const unsigned slot = b.adc_stage_ - 1;
            auto& sample = b.pending_sample_;
            if (!(sample.adc_window_after_mask & (1U << slot))) {
                for (int p = 0; p < 3; ++p) sample.adc_pwm_compare_after[slot][p] = b.active_compare_[p];
                sample.adc_window_after_mask |= 1U << slot;
                sample.pwm_compare_valid &= b.pwm_compare_valid_ &&
                    sample.pwm_latch_sequence + slot + 1 == b.pwm_latch_sequence_;
            }
        }
    }
    if (trigger_second) {
        auto& sample = b.pending_sample_;
        for (int p = 0; p < 3; ++p) sample.pwm_compare_after[p] = b.active_compare_[p];
        sample.pwm_compare_valid &= b.pwm_compare_valid_ && sample.pwm_latch_sequence + 1 == b.pwm_latch_sequence_;
        sample.adc_window_after_mask = 7;
    }
    // Update the shadow registers just after TEZ; all three latch at the next TEZ.
    if (b.duty_pending_) {
        for (int p = 0; p < 3; ++p) mcpwm_comparator_set_compare_value(b.comparators_[p], b.pending_[p]);
        b.shadow_compare_ = b.pending_;
        b.force_low_.commit_shadow();
        b.duty_pending_ = false;
    }
    portEXIT_CRITICAL_ISR(&b.mux_);
    return false;
}
bool IRAM_ATTR Board::on_sample(mcpwm_cmpr_handle_t, const mcpwm_compare_event_data_t* event, void* context) {
    auto& b = *static_cast<Board*>(context);
    if (event->direction != MCPWM_TIMER_DIRECTION_DOWN) {
        if constexpr (staggered_sampling) return false;
        if (!staggered_sampling && b.adc_pending_ && b.adc_stage_ < 3) {
            b.adc_requested_stage_ = 3;
            b.start_requested_conversion();
        }
        // Encoder cadence is derived directly from PWM, independently of the
        // selected ADC/control rate. +15 us stays close to the former DMA
        // publication phase without halving encoder bandwidth at 2 kHz.
        BaseType_t wake = pdFALSE;
        if (++b.encoder_divider_ >= pwm_hz / encoder_hz) {
            b.encoder_divider_ = 0;
            const auto encoder_task = b.encoder_task_.load(std::memory_order_relaxed);
            if (encoder_task) vTaskNotifyGiveFromISR(encoder_task, &wake);
        }
        return wake == pdTRUE;
    }
    if constexpr (staggered_sampling) {
        mcpwm_ll_intr_enable(&MCPWM0, MCPWM_LL_EVENT_CMP_EQUAL(0, 1), false);
        // Three conversions at one common carrier phase in successive cycles;
        // the remaining cycle(s) preserve the configured control cadence.
        const unsigned slot = b.divider_;
        b.divider_ = (b.divider_ + 1) % (pwm_hz / control_hz);
        if (slot >= 3) return false;
        if (b.adc_desynchronized_) {
            if (slot == 0) {
                auto missing = b.pending_sample_; missing.valid = false;
                missing.timestamp_us = esp_timer_get_time();
                return b.publish_sample(missing);
            }
            return false;
        }
        if (slot == 0) {
            if (b.adc_pending_) {
                APB_SARADC.ctrl.start = 0;
                b.adc_pending_ = false;
                b.adc_desynchronized_ = true;
                auto missing = b.pending_sample_; missing.valid = false;
                return b.publish_sample(missing);
            }
            b.pending_sample_ = {};
            b.pending_sample_.valid = true;
            b.pending_sample_.pwm_compare_valid = b.pwm_compare_valid_;
            b.pending_sample_.pwm_latch_sequence = b.pwm_latch_sequence_;
            b.adc_received_ = 0; b.adc_pending_ = true;
        }
        auto& s = b.pending_sample_;
        if (!b.adc_pending_ || b.adc_received_ != slot) {
            s.valid = false;
            return false;
        }
        for (int p = 0; p < 3; ++p) s.adc_pwm_compare_before[slot][p] = b.active_compare_[p];
        const uint32_t falling = std::min({b.active_compare_[0], b.active_compare_[1], b.active_compare_[2]});
        s.pwm_compare_valid &= b.pwm_compare_valid_ &&
            !(mcpwm_ll_intr_get_status(&MCPWM0) & MCPWM_LL_EVENT_TIMER_EMPTY(0));
        // The compare fires early enough to prepare bookkeeping. Wait at most
        // seven microseconds, and start near -1.4 us with the required settling.
        // No diagnostic copies or wall-clock calls are made after this point
        // until START is asserted.
        const int64_t before_wait_us = esp_timer_get_time();
        auto trigger = pwm_counter();
        const int before_wait_tick = trigger.signed_tick();
        const int earliest = std::max(-112, -static_cast<int>(falling) + 480);
        while (trigger.down && trigger.signed_tick() < earliest) trigger = pwm_counter();
        const auto sar = sar_status();
        b.adc_stage_ = slot + 1;
        s.valid &= trigger.down && trigger.signed_tick() < 0 && sar == 0;
        APB_SARADC.ctrl.start = 0; APB_SARADC.ctrl.start = 1;
        const int64_t now = before_wait_us + (trigger.signed_tick() - before_wait_tick) / 80;
        if (slot == 0) {
            s.timestamp_us = now;
            s.start_tick = trigger.signed_tick();
            for (int p = 0; p < 3; ++p) s.pwm_compare_before[p] = s.adc_pwm_compare_before[0][p];
        }
        s.adc_trigger_elapsed_us[slot] = std::clamp<int64_t>(now - s.timestamp_us, 0, 65535);
        s.adc_trigger_ticks[slot] = trigger.signed_tick();
        s.adc_trigger_sar[slot] = sar;
        if (trigger.down) s.adc_trigger_down_mask |= 1U << slot;
        s.end_tick = trigger.signed_tick();
        return false;
    }
    if (++b.divider_ < pwm_hz / control_hz) return false;
    b.divider_ = 0;
    if (b.adc_desynchronized_) {
        auto missing = b.pending_sample_; missing.valid = false;
        missing.timestamp_us = esp_timer_get_time();
        return b.publish_sample(missing);
    }
    if (b.adc_pending_) {
        adc_ll_digi_trigger_disable();
        APB_SARADC.ctrl.start = 0;
        b.adc_pending_ = false;
        b.adc_desynchronized_ = true;
        auto missing = b.pending_sample_; missing.valid = false;
        return b.publish_sample(missing);
    }
    // Clear/copy the diagnostic envelope before taking the trigger timestamp.
    // This keeps initialization work out of the measured trigger-to-TEZ gap.
    b.pending_sample_ = {};
    auto& s = b.pending_sample_;
    for (int p = 0; p < 3; ++p) s.pwm_compare_before[p] = b.active_compare_[p];
    s.pwm_latch_sequence = b.pwm_latch_sequence_;
    s.pwm_compare_valid = b.pwm_compare_valid_ &&
        !(mcpwm_ll_intr_get_status(&MCPWM0) & MCPWM_LL_EVENT_TIMER_EMPTY(0));
    s.timestamp_us = esp_timer_get_time();
    const auto trigger = pwm_counter();
    s.start_tick = trigger.signed_tick();
    s.adc_trigger_ticks[0] = s.start_tick;
    s.adc_trigger_sar[0] = sar_status();
    if (trigger.down) s.adc_trigger_down_mask |= 1;
    constexpr int low_ticks = half_period - static_cast<int>(half_period * duty_ceiling);
    if (!trigger.down || s.start_tick < -low_ticks + 480 || s.start_tick > 0) return b.publish_sample(s);
    s.valid = s.adc_trigger_sar[0] == 0; b.adc_stage_ = b.adc_requested_stage_ = 1; b.adc_received_ = 0; b.adc_pending_ = true;
    b.adc_last_start_us_ = esp_timer_get_time();
    APB_SARADC.ctrl.start = 0; APB_SARADC.ctrl.start = 1;
    return false;
}
bool IRAM_ATTR Board::on_adc_done(adc_continuous_handle_t, const adc_continuous_evt_data_t* event, void* context) {
    auto& b = *static_cast<Board*>(context);
    // DMA has completed before entering this callback. Capture its conservative
    // completion bound before lock/ownership bookkeeping adds latency. Reject
    // the observation if a PWM ISR changes ownership before we acquire mux_.
    const unsigned captured_stage = b.adc_stage_;
    const unsigned captured_received = b.adc_received_;
    const auto callback_sar = sar_status();
    const auto callback_counter = pwm_counter();
    // Atomically associate this frame with its START before clearing START.
    // A delayed callback may never cancel a newer PWM-triggered conversion.
    portENTER_CRITICAL_ISR(&b.mux_);
    if (!b.adc_pending_ || b.adc_stage_ != captured_stage || b.adc_received_ != captured_received ||
        !adc_stage_matches(b.adc_stage_, b.adc_received_)) {
        if (!b.timer_running_) APB_SARADC.ctrl.start = 0;
        if (b.adc_pending_) b.pending_sample_.valid = false;
        portEXIT_CRITICAL_ISR(&b.mux_);
        return false;
    }
    APB_SARADC.ctrl.start = 0;
    const int64_t callback_us = esp_timer_get_time();
    // Keep START low between conversions. A back-to-back APB 0/1 write is
    // shorter than a 5 MHz SAR controller cycle and can lose the rising edge.
    const unsigned index = b.adc_received_++;
    const bool good = event->size == SOC_ADC_DIGI_RESULT_BYTES;
    adc_digi_output_data_t data{};
    if (good) {
        static_assert(sizeof(adc_digi_output_data_t) == sizeof(uint32_t));
        data.val = *reinterpret_cast<const volatile uint32_t*>(event->conv_frame_buffer);
    }
    portEXIT_CRITICAL_ISR(&b.mux_);
    auto& pending = b.pending_sample_;
    pending.adc_callback_ticks[index] = callback_counter.signed_tick();
    pending.adc_callback_sar[index] = callback_sar;
    pending.adc_callback_elapsed_us[index] = std::clamp<int64_t>(callback_us - pending.timestamp_us, 0, 65535);
    if (callback_counter.down) pending.adc_callback_down_mask |= 1U << index;
    pending.valid &= good;
    if (good) {
        // IDF returns the descriptor to DMA before this callback. Read the
        // aligned word exactly once: validation, phase and current must all
        // describe the same frame even if the DMA buffer is later recycled.
        pending.adc_slot_words[index] = data.val;
        pending.adc_slot_mask |= 1U << index;
    }
    if constexpr (staggered_sampling) {
        const uint32_t falling = std::min({pending.adc_pwm_compare_before[index][0],
            pending.adc_pwm_compare_before[index][1], pending.adc_pwm_compare_before[index][2]});
        const uint32_t rising = std::min({pending.adc_pwm_compare_after[index][0],
            pending.adc_pwm_compare_after[index][1], pending.adc_pwm_compare_after[index][2]});
        const int elapsed = pending.adc_callback_elapsed_us[index] - pending.adc_trigger_elapsed_us[index];
        pending.valid &= (pending.adc_window_after_mask & (1U << index)) &&
            !callback_counter.down && callback_sar == 0 && elapsed >= 0 && elapsed < 15 &&
            pwm_adc_window_valid(pending.adc_trigger_ticks[index], callback_counter.signed_tick(), falling, rising);
    }
    // Preserve all three words to test the existing one-word pipeline model.
    // Continue feeding the original two slots to PI during the window A/B test.
    if (index == 0) {
        pending.adc_completed_mask |= 1U << index;
        b.start_requested_conversion();
        return false;
    }
    const unsigned phase_index = index - 1;
    if (good) {
        pending.dma_words[phase_index] = data.val;
        pending.valid &= data.type2.unit == 0 && data.type2.channel >= 3 && data.type2.channel <= 5 && data.type2.data <= 4095;
        pending.phase[phase_index] = data.type2.channel - 3;
        pending.raw[phase_index] = data.type2.data;
    }
    pending.adc_completed_mask |= 1U << index;
    if (index != 2) { b.start_requested_conversion(); return false; }
    APB_SARADC.ctrl.start = 0;
    portENTER_CRITICAL_ISR(&b.mux_);
    RawSample s = pending;
    b.adc_pending_ = false;
    portEXIT_CRITICAL_ISR(&b.mux_);
    const uint32_t duration = callback_us - s.timestamp_us;
    s.duration_ticks = duration * 80;
    s.phase_skew_us = s.adc_trigger_elapsed_us[1];
    s.valid &= b.adc_stage_ == 3 && s.phase[0] != s.phase[1] && s.adc_completed_mask == 7 && s.pwm_compare_valid;
    if constexpr (staggered_sampling) {
        // Existing one-word DMA pipeline: the two useful words represent the
        // first two STARTs. This is an experimental non-simultaneous estimate;
        // retain the actual phase separation and timestamp its midpoint.
        s.valid &= adc_staggered_timing_valid(s.phase_skew_us, s.adc_trigger_elapsed_us[2], duration);
        // Both useful conversions must see the same applied command. Changing
        // voltage between these samples would invalidate reconstruction even
        // when both individual low-side windows are valid.
        s.valid &= pwm_adc_pair_coherent(s.adc_pwm_compare_before, s.adc_pwm_compare_after);
        s.timestamp_us += s.phase_skew_us / 2;
        for (int p = 0; p < 3; ++p) s.pwm_compare_after[p] = s.adc_pwm_compare_after[2][p];
    } else {
        // Expand repeated diagnostic metadata only after all conversions have
        // finished, outside the narrow TEZ-to-next-START interval.
        for (int slot = 0; slot < 3; ++slot)
            for (int p = 0; p < 3; ++p) {
                s.adc_pwm_compare_before[slot][p] = s.pwm_compare_before[p];
                s.adc_pwm_compare_after[slot][p] = s.pwm_compare_after[p];
            }
        const uint32_t falling = std::min({s.pwm_compare_before[0], s.pwm_compare_before[1], s.pwm_compare_before[2]});
        const uint32_t rising = std::min({s.pwm_compare_after[0], s.pwm_compare_after[1], s.pwm_compare_after[2]});
        s.valid &= !callback_counter.down && callback_sar == 0 && duration < 45 &&
            pwm_adc_window_valid(s.start_tick, callback_counter.signed_tick(), falling, rising);
    }
    return b.publish_sample(s);
}
bool IRAM_ATTR Board::publish_sample(RawSample s) {
    auto& b = *this;
    if (!s.valid) ++b.rejected_;
    s.rejected = b.rejected_;
    s.sequence = ++b.sequence_;
    const bool good = s.valid;
    if (b.enabled_.load(std::memory_order_relaxed)) {
        int cause = 0;
        portENTER_CRITICAL_ISR(&b.mux_);
        if (good && b.trip_ready_.load(std::memory_order_acquire)) {
            // Integer microvolts, using the same calibrated LUT as the task.
            // Include the reconstructed phase so rotation of the ADC pair
            // cannot conceal a sustained excess in the unmeasured phase.
            const int32_t a = b.millivolts_[s.phase[0]][s.raw[0]] * 1000 - b.offset_uv_[s.phase[0]];
            const int32_t c = b.millivolts_[s.phase[1]][s.raw[1]] * 1000 - b.offset_uv_[s.phase[1]];
            const int32_t peak = std::max({std::abs(a), std::abs(c), std::abs(a + c)});
            const auto level = peak >= b.absolute_trip_uv_ ? CurrentTripLevel::emergency :
                               peak >= b.peak_trip_uv_ ? CurrentTripLevel::peak :
                               peak >= b.trip_uv_ ? CurrentTripLevel::overload : CurrentTripLevel::normal;
            if (b.current_trip_filter_.step(level, control_period_us)) cause = 1;
        } else {
            b.current_trip_filter_.step(CurrentTripLevel::invalid, control_period_us);
        }
        portEXIT_CRITICAL_ISR(&b.mux_);
        if (!cause && static_cast<uint32_t>(esp_timer_get_time()) - b.watchdog_us_.load() > 10000) cause = 2;
        if (cause && !b.trip_.load()) {
            portENTER_CRITICAL_ISR(&b.mux_);
            b.trip_sample_ = s;
            b.trip_.store(cause);
            portEXIT_CRITICAL_ISR(&b.mux_);
        }
        if (b.trip_.load()) gpio_ll_set_level(&GPIO, enable_gpio, 0);
    }
    portENTER_CRITICAL_ISR(&b.mux_);
    b.sample_ = s;
    portEXIT_CRITICAL_ISR(&b.mux_);
    BaseType_t wake = pdFALSE;
    if (b.task_) vTaskNotifyGiveFromISR(b.task_, &wake);
    return wake == pdTRUE;
}
RawSample Board::sample() { portENTER_CRITICAL(&mux_); const auto s = sample_; portEXIT_CRITICAL(&mux_); return s; }
RawSample Board::trip_sample() { portENTER_CRITICAL(&mux_); const auto s = trip_sample_; portEXIT_CRITICAL(&mux_); return s; }
float Board::volts(unsigned p, int raw) const { return millivolts_[p][std::clamp(raw, 0, 4095)] * 0.001f; }
void Board::set_trip(const std::array<float, 3>& zero, float gain, float amperes) {
    std::array<int32_t, 3> offsets{};
    for (int p = 0; p < 3; ++p) {
        offsets[p] = std::lround(zero[p] * 1000000);
    }
    const int32_t threshold = std::ceil(gain * amperes * 1000000);
    const int32_t peak = std::ceil(gain * peak_current_trip * 1000000);
    const int32_t absolute = std::ceil(gain * absolute_current_trip * 1000000);
    // Publish all three phases atomically relative to the sampling ISR.
    portENTER_CRITICAL(&mux_);
    // Live operating-limit changes retain accumulated evidence, especially
    // the fixed peak band. Only a changed current measurement scale/zero starts
    // a new exposure here; disable explicitly clears it for the next session.
    if (offset_uv_ != offsets || peak_trip_uv_ != peak || absolute_trip_uv_ != absolute) current_trip_filter_.reset();
    offset_uv_ = offsets; trip_uv_ = threshold; peak_trip_uv_ = peak; absolute_trip_uv_ = absolute;
    trip_ready_.store(true, std::memory_order_release);
    portEXIT_CRITICAL(&mux_);
}
void Board::reset_trip() {
    portENTER_CRITICAL(&mux_);
    trip_.store(0); current_trip_filter_.reset();
    portEXIT_CRITICAL(&mux_);
}
esp_err_t Board::read_encoder(EncoderSample& sample) {
    return encoder_.read(sample);
}
esp_err_t Board::write_duty(const std::array<float, 3>& duty) {
    std::array<uint32_t, 3> next{};
    const uint8_t force_mask = zero_duty_mask(duty[0], duty[1], duty[2]);
    for (int p = 0; p < 3; ++p) {
        if (!std::isfinite(duty[p]) || duty[p] < 0 || duty[p] > duty_ceiling ||
            (duty[p] > 0 && duty[p] < duty_floor)) { enable(false); return ESP_ERR_INVALID_ARG; }
        next[p] = std::clamp<uint32_t>(static_cast<uint32_t>(half_period * (1 - duty[p])), 1, half_period);
    }
    if (force_mask == 7) {
        // Stop/offset paths require zero to be accepted even at the TEZ guard.
        // Queue it for the ordinary TEZ pipeline (at most two carrier cycles).
        portENTER_CRITICAL(&mux_); pending_ = next; force_low_.queue(force_mask); duty_pending_ = true; portEXIT_CRITICAL(&mux_);
        return ESP_OK;
    }
    if constexpr (staggered_sampling) {
        for (int attempt = 0; attempt < 3; ++attempt) {
            portENTER_CRITICAL(&mux_);
            const uint32_t phase = pwm_counter().phase_ticks(half_period);
            const bool tez_pending = mcpwm_ll_intr_get_status(&MCPWM0) & MCPWM_LL_EVENT_TIMER_EMPTY(0);
            if (tez_pending || phase + 400 >= 2 * half_period) { portEXIT_CRITICAL(&mux_); continue; }
            for (int p = 0; p < 3; ++p) mcpwm_comparator_set_compare_value(comparators_[p], next[p]);
            shadow_compare_ = pending_ = next; force_low_.queue(force_mask); force_low_.commit_shadow(); duty_pending_ = false;
            const bool crossed = pwm_counter().phase_ticks(half_period) < phase ||
                (mcpwm_ll_intr_get_status(&MCPWM0) & MCPWM_LL_EVENT_TIMER_EMPTY(0));
            portEXIT_CRITICAL(&mux_);
            if (crossed) { enable(false); return ESP_ERR_TIMEOUT; }
            return ESP_OK;
        }
        enable(false); return ESP_ERR_TIMEOUT;
    }
    portENTER_CRITICAL(&mux_); pending_ = next; force_low_.queue(force_mask); duty_pending_ = true; portEXIT_CRITICAL(&mux_);
    return ESP_OK;
}
esp_err_t Board::write_voltage(Dq voltage, float sample_electrical_angle,
        float electrical_velocity, int64_t sample_timestamp_us, float bus_voltage,
        std::array<float, 3>& duty, float& apply_delay_us) {
    if (!timer_running_ || !std::isfinite(voltage.d) || !std::isfinite(voltage.q) ||
        !std::isfinite(sample_electrical_angle) || !std::isfinite(electrical_velocity) ||
        !std::isfinite(bus_voltage) || bus_voltage < 4.5f || sample_timestamp_us <= 0) {
        enable(false); return ESP_ERR_INVALID_ARG;
    }
    constexpr uint32_t period_ticks = 2 * half_period;
    constexpr uint32_t period_us = 1000000 / pwm_hz;
    constexpr uint32_t ticks_per_us = resolution / 1000000;
    constexpr uint32_t commit_guard_ticks = (staggered_sampling ? 5 : 2) * ticks_per_us;
    // This API and MCPWM ISRs belong to the same motor-task core. No task can
    // run between IDF clearing TEZ status and our on_empty callback completing.
    // Keep trigonometry outside the critical section and retry if a TEZ was
    // serviced (or became pending) while calculating the command.
    for (int attempt = 0; attempt < 3; ++attempt) {
        portENTER_CRITICAL(&mux_);
        const uint32_t sequence = pwm_latch_sequence_;
        const bool pending = mcpwm_ll_intr_get_status(&MCPWM0) & MCPWM_LL_EVENT_TIMER_EMPTY(0);
        const int64_t captured_us = esp_timer_get_time();
        const uint32_t phase = pwm_counter().phase_ticks(half_period);
        const bool pending_after = mcpwm_ll_intr_get_status(&MCPWM0) & MCPWM_LL_EVENT_TIMER_EMPTY(0);
        portEXIT_CRITICAL(&mux_);
        if (pending != pending_after || phase + commit_guard_ticks >= period_ticks) continue;
        if constexpr (staggered_sampling) { if (pending) continue; }
        if (captured_us < sample_timestamp_us) { enable(false); return ESP_ERR_INVALID_ARG; }
        const float delay_us = static_cast<float>(staggered_sampling ? pwm_direct_latch_delay_ticks(phase, half_period) :
            pwm_latch_delay_ticks(phase, pending, half_period)) / ticks_per_us;
        const float horizon_s = (static_cast<float>(captured_us - sample_timestamp_us) + delay_us) * 1e-6f;
        auto next_duty = svpwm(voltage, wrap(sample_electrical_angle + electrical_velocity * horizon_s), bus_voltage);
        const bool quiet = voltage.d == 0 && voltage.q == 0;
        if (quiet) next_duty = {0, 0, 0};
        const uint8_t force_mask = zero_duty_mask(next_duty[0], next_duty[1], next_duty[2]);
        std::array<uint32_t, 3> next{};
        for (int p = 0; p < 3; ++p) {
            if (!std::isfinite(next_duty[p]) || next_duty[p] < 0 || next_duty[p] > duty_ceiling ||
                (next_duty[p] > 0 && next_duty[p] < duty_floor)) {
                enable(false); return ESP_ERR_INVALID_ARG;
            }
            next[p] = std::clamp<uint32_t>(static_cast<uint32_t>(half_period * (1 - next_duty[p])), 1, half_period);
        }
        portENTER_CRITICAL(&mux_);
        const int64_t commit_us = esp_timer_get_time();
        const uint32_t current_phase = pwm_counter().phase_ticks(half_period);
        const bool current_pending = mcpwm_ll_intr_get_status(&MCPWM0) & MCPWM_LL_EVENT_TIMER_EMPTY(0);
        const bool same_schedule = pwm_schedule_still_valid(sequence, pwm_latch_sequence_,
            pending, current_pending, phase, current_phase, period_ticks, commit_guard_ticks,
            static_cast<uint32_t>(commit_us - captured_us), period_us);
        if (same_schedule) {
            pending_ = next;
            force_low_.queue(force_mask);
            if constexpr (staggered_sampling) {
                for (int p = 0; p < 3; ++p) mcpwm_comparator_set_compare_value(comparators_[p], next[p]);
                shadow_compare_ = next; force_low_.commit_shadow(); duty_pending_ = false;
            } else duty_pending_ = true;
        }
        const bool crossed = staggered_sampling && same_schedule &&
            (pwm_counter().phase_ticks(half_period) < current_phase ||
             (mcpwm_ll_intr_get_status(&MCPWM0) & MCPWM_LL_EVENT_TIMER_EMPTY(0)));
        portEXIT_CRITICAL(&mux_);
        if (crossed) { enable(false); return ESP_ERR_TIMEOUT; }
        if (same_schedule) {
            duty = next_duty;
            apply_delay_us = delay_us - static_cast<float>(commit_us - captured_us);
            return ESP_OK;
        }
    }
    enable(false);
    return ESP_ERR_TIMEOUT;
}
void Board::enable(bool enabled) {
    if (!enabled) {
        gpio_set_level(static_cast<gpio_num_t>(enable_gpio), 0);
        portENTER_CRITICAL(&mux_);
        enabled_.store(false); trip_ready_.store(false); current_trip_filter_.reset();
        portEXIT_CRITICAL(&mux_);
        return;
    }
    kick(); enabled_.store(true);
    gpio_set_level(static_cast<gpio_num_t>(enable_gpio), 1);
}
void Board::shutdown() {
    enable(false);
    if (timer_running_) { mcpwm_timer_start_stop(timer_, MCPWM_TIMER_STOP_EMPTY); vTaskDelay(pdMS_TO_TICKS(2)); timer_running_ = false; }
    if (timer_enabled_) { mcpwm_timer_disable(timer_); timer_enabled_ = false; }
    if (adc_started_) { adc_continuous_stop(adc_); adc_started_ = false; }
    if (adc_) { adc_continuous_deinit(adc_); adc_ = nullptr; }
    task_ = nullptr; adc_pending_ = false;
    if (sample_comparator_) { mcpwm_del_comparator(sample_comparator_); sample_comparator_ = nullptr; }
    for (int p = 2; p >= 0; --p) {
        if (generators_[p]) { mcpwm_del_generator(generators_[p]); generators_[p] = nullptr; }
        if (comparators_[p]) { mcpwm_del_comparator(comparators_[p]); comparators_[p] = nullptr; }
        if (operators_[p]) { mcpwm_del_operator(operators_[p]); operators_[p] = nullptr; }
        if (cali_[p]) { adc_cali_delete_scheme_curve_fitting(cali_[p]); cali_[p] = nullptr; }
    }
    if (timer_) { mcpwm_del_timer(timer_); timer_ = nullptr; }
    encoder_.shutdown();
}
}
