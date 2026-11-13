#include "knob_app.hpp"
#include "app_task.hpp"
#include "board.hpp"
#include "bus_voltage.hpp"
#include "protocol.hpp"
#include "encoder_mailbox.hpp"
#include "encoder_config.hpp"
#include "reference_modes.hpp"
#include "live_config.hpp"
#include "sampling_recovery.hpp"
#include "control_timing.hpp"
#include "current_offsets.hpp"
#include "driver/usb_serial_jtag.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace knob {
namespace {
// Compile-time transport adapter: exactly one tuning transport is active.
int console_read(uint8_t* bytes, size_t capacity) {
#if CONFIG_KNOB_CONSOLE_UART
    return uart_read_bytes(UART_NUM_0, bytes, capacity, pdMS_TO_TICKS(20));
#else
    return usb_serial_jtag_read_bytes(bytes, capacity, pdMS_TO_TICKS(20));
#endif
}
bool console_write(const char* bytes, size_t size) {
#if CONFIG_KNOB_CONSOLE_UART
    return uart_write_bytes(UART_NUM_0, bytes, size) == static_cast<int>(size);
#else
    // The driver may enqueue only part of the frame before a timeout.
    const int64_t deadline = Board::esp_time() + 100000;
    size_t sent = 0;
    while (sent < size && Board::esp_time() < deadline) {
        // A ring-buffer item must fit in the driver's TX ring. Expanded
        // telemetry can exceed its maximum item size even when the ring is
        // empty; submit bounded chunks and keep the original frame deadline.
        const int count = usb_serial_jtag_write_bytes(bytes + sent, std::min<size_t>(size - sent, 512), pdMS_TO_TICKS(10));
        if (count < 0) return false;
        sent += count;
    }
    return sent == size;
#endif
}
// Encoder transport waits on core 0; core 1 only copies a completed snapshot.
class EncoderWorker final : public AppTask {
public:
    using Frame = EncoderFrame;
    explicit EncoderWorker(Board& board) : AppTask("knob_encoder", 4096, 18, 0), board_(board) {}
    bool initialize() {
        initialized_.store(ESP_ERR_NOT_FINISHED);
        if (!start()) return false;
        const int64_t deadline = Board::esp_time() + 250000;
        while (initialized_.load() == ESP_ERR_NOT_FINISHED && Board::esp_time() < deadline)
            vTaskDelay(pdMS_TO_TICKS(1));
        return initialized_.load() == ESP_OK;
    }
    IRAM_ATTR Frame consume_frame() {
        portENTER_CRITICAL(&mux_); const auto result = mailbox_.consume(); portEXIT_CRITICAL(&mux_); return result;
    }
    void acknowledge_errors() {
        portENTER_CRITICAL(&mux_); mailbox_.acknowledge_errors(); portEXIT_CRITICAL(&mux_);
    }
private:
    Board& board_;
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    EncoderMailbox mailbox_;
    std::atomic<int> initialized_{ESP_ERR_NOT_FINISHED};
    void main() override {
        portENTER_CRITICAL(&mux_); mailbox_.reset(); portEXIT_CRITICAL(&mux_);
        const esp_err_t init = board_.initialize_encoder();
        initialized_.store(init);
        if (init != ESP_OK) {
            while (!should_exit()) vTaskDelay(pdMS_TO_TICKS(5));
            return;
        }
        board_.set_encoder_task(xTaskGetCurrentTaskHandle());
        uint32_t failures = 0;
        while (!should_exit()) {
            if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5))) continue;
            Frame next;
            const int64_t started = Board::esp_time();
            next.error = board_.read_encoder(next);
            next.duration_us = Board::esp_time() - started;
            next.timestamp_us = started + next.duration_us / 2;
            next.transport_error = board_.encoder_transport_error();
            next.retries = board_.encoder_retries();
            if (next.error) ++failures;
            next.failures = failures;
            portENTER_CRITICAL(&mux_); mailbox_.publish(next); portEXIT_CRITICAL(&mux_);
        }
        board_.set_encoder_task(nullptr);
    }
};
class BusVoltageWorker final : public AppTask {
public:
    BusVoltageWorker() : AppTask("knob_vbus", 3072, 6, 0) {}
    bool initialize() {
        if constexpr (!BusVoltageMonitor::enabled) return true;
        portENTER_CRITICAL(&mux_); sample_ = {}; portEXIT_CRITICAL(&mux_);
        return is_running() || start();
    }
    void request_reinitialize() {
        if constexpr (!BusVoltageMonitor::enabled) return;
        portENTER_CRITICAL(&mux_); sample_ = {}; reinitialize_.store(true); portEXIT_CRITICAL(&mux_);
    }
    IRAM_ATTR BusVoltageSample sample() {
        portENTER_CRITICAL(&mux_);
        const auto result = sample_;
        portEXIT_CRITICAL(&mux_);
        return result;
    }
private:
    BusVoltageMonitor monitor_;
    BusVoltageSample sample_{};
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    std::atomic<bool> reinitialize_{false};
    void main() override {
        monitor_.initialize();
        TickType_t wake = xTaskGetTickCount();
        while (!should_exit()) {
            if (reinitialize_.exchange(false)) monitor_.initialize();
            const auto next = monitor_.read();
            portENTER_CRITICAL(&mux_);
            if (!reinitialize_.load()) sample_ = next;
            portEXIT_CRITICAL(&mux_);
            vTaskDelayUntil(&wake, pdMS_TO_TICKS(BusVoltageMonitor::sample_period_us / 1000));
        }
        monitor_.shutdown();
    }
};
class Motor final : public AppTask {
public:
    Motor() : AppTask("knob_control", 9216, 20, 1) {}
    QueueHandle_t requests = nullptr, replies = nullptr;
#if CONFIG_KNOB_DIAGNOSTICS
    QueueHandle_t test_plans = nullptr, test_requests = nullptr, test_replies = nullptr;
#endif
    Snapshot state{};
private:
    Board board_;
    EncoderWorker encoder_{board_};
    BusVoltageWorker bus_;
    AngleTracker angle_;
    Haptics haptic_;
    ReferenceHaptics reference_haptic_;
    Frictionless frictionless_;
    HapticSpeedLimiter haptic_speed_;
    HapticTransition haptic_transition_;
    CurrentLoop current_loop_;
    QuietOutput quiet_output_;
    CurrentLimiter current_limiter_;
    CurrentTripFilter current_trip_filter_;
    bool ready_ = false, calibrating_ = false, wake_ = false;
    int64_t heartbeat_ = 0, stage_start_ = 0, last_sample_ = 0, pulse_start_ = 0;
    int64_t trip_update_at_ = 0, last_encoder_ = 0, metrics_at_ = 0, samples_after_ = 0;
    uint32_t last_sequence_ = 0, metrics_cycles_ = 0;
    float sample_angle_ = 0, output_angle_ = 0;
    CurrentOffsetCollector offsets_;
    CalibrationSequence calibration_sequence_;
    CalibrationPlan calibration_plan_{};
    int64_t calibration_started_ = 0;
    float offset_start_ = 0;
    SamplingRecovery sampling_recovery_;
    ControlSampleClock control_clock_;
    EncoderFreshness encoder_freshness_;
#if CONFIG_KNOB_DIAGNOSTICS
    DiagnosticPlan test_plan_{};
    Pi test_speed_pi_;
    DiagnosticHold torque_hold_{};
    std::array<DiagnosticSample, diagnostic_capacity> test_samples_{};
    int64_t test_started_ = 0, test_sampled_ = 0;
    float test_origin_ = 0, test_electrical_ = 0;
    int64_t capture_started_ = 0;
    int capture_next_ = 0;
#endif
    bool energized() const { return state.state != State::idle && state.state != State::fault; }
    const ReferenceModeConfig& reference_config(Mode mode) const {
        const int preset = static_cast<int>(mode) - reference_mode_first;
        return preset >= 0 && preset < reference_config_count ? state.reference_modes[preset] : default_reference_config(0);
    }
    IRAM_ATTR void reset_frictionless() {
        frictionless_.reset(); state.frictionless_phase = 0;
        state.frictionless_reference = state.frictionless_correction = state.frictionless_feedforward = 0;
    }
    IRAM_ATTR void update_haptic_position() {
        if (reference_mode(state.config.mode) && state.config.mode != Mode::frictionless) {
            state.position = reference_haptic_.position(); state.sub_position = reference_haptic_.sub_position();
        } else {
            state.position = haptic_.position(); state.sub_position = haptic_.sub_position(state.config, angle_.angle());
        }
    }
    IRAM_ATTR void capture_fault_sample(const RawSample& raw, int interval_us, int64_t now) {
        state.fault_sample_interval_us = interval_us;
        state.fault_sample_age_us = now - raw.timestamp_us;
        state.fault_sample_start_tick = raw.start_tick;
        state.fault_sample_end_tick = raw.end_tick;
        state.fault_sample_us = raw.duration_ticks / 80.0f;
        state.fault_sample_valid = raw.valid;
        state.fault_adc_dma_words = {raw.dma_words[0], raw.dma_words[1]};
        state.fault_adc_raw = {raw.raw[0], raw.raw[1]};
        state.fault_adc_phase = {raw.phase[0], raw.phase[1]};
        state.fault_adc_captured = true;
    }
    IRAM_ATTR void capture_overcurrent() {
        const auto raw = board_.trip_sample();
        capture_fault_sample(raw, Board::control_period_us, Board::esp_time());
        if (!raw.valid) return;
        float sum = 0;
        for (int k = 0; k < 2; ++k) {
            const int p = raw.phase[k];
            state.fault_currents[p] = phase_current(board_.volts(p, raw.raw[k]), state.offsets[p], state.config.sense_gain);
            sum += state.fault_currents[p];
        }
        state.fault_currents[3 - raw.phase[0] - raw.phase[1]] = -sum;
        state.fault_current_valid = true;
        state.fault_current_trip_limit = state.current_trip_limit;
        state.fault_angle = state.angle;
        state.fault_velocity = state.velocity;
        state.fault_velocity_fast = state.velocity_fast;
    }
    IRAM_ATTR void halt(const char* fault = nullptr) {
        if (calibrating_) {
            calibration_sequence_.abort();
            state.calibration_report.passed = false;
            state.calibration_report.cancelled = fault == nullptr;
            state.calibration_report.stage = fault ? "failed" : "cancelled";
            state.calibration_report.error = fault ? fault : "calibration_cancelled";
            calibrating_ = false;
        }
#if CONFIG_KNOB_DIAGNOSTICS
        torque_hold_.reset();
        if (state.diagnostic.busy && state.diagnostic.stage < 4) {
            state.diagnostic.stage = 5; state.diagnostic.cancelled = true;
        }
        if (state.diagnostic.stage == 6) state.diagnostic.stage = 7;
#endif
        board_.enable(false); board_.write_duty({0, 0, 0});
        state.state = fault ? State::fault : State::idle; state.fault = fault ? fault : "none";
        state.target = state.target_d = 0; state.duty = {}; state.current_valid = false;
        state.voltage_d = 0; state.voltage_q = 0;
        state.pwm_apply_delay_us = 0;
        quiet_output_.reset(); state.quiet_output = false;
        state.current_limited = false; state.current_target_limit = 0;
        haptic_speed_.reset(); state.haptic_scale = 0;
        haptic_transition_.reset();
        reset_frictionless();
        state.speed_limited = false; state.haptic_paused = false;
        current_limiter_.reset(state.config.current_limit); trip_update_at_ = 0;
        current_trip_filter_.reset();
        if (!fault) state.current_trip_limit = overcurrent_threshold(state.config.current_limit);
        current_loop_.reset(); control_clock_.reset(); pulse_start_ = 0;
        sampling_recovery_.reset();
        encoder_freshness_.reset();
    }
    IRAM_ATTR void fail(const char* fault) {
        if (state.state == State::fault) return; // Preserve the first cause until clear.
        halt(fault);
    }
    IRAM_ATTR void zero_for_feedback_recovery() {
        reset_frictionless();
        if (calibrating_ && state.state != State::offsets)
            calibration_sequence_.invalidate(Board::esp_time(), true);
        // Keep the driver enabled while waiting for trustworthy feedback.
        // Reset PI once per episode so resumption cannot replay old voltage.
        if (board_.write_duty({0, 0, 0}) != ESP_OK) { fail("pwm_failure"); return; }
        current_loop_.reset();
        control_clock_.reset();
        state.duty = {}; state.voltage_d = state.voltage_q = 0;
        state.pwm_apply_delay_us = 0;
        ++state.sampling_recoveries;
    }
#if CONFIG_KNOB_DIAGNOSTICS
    IRAM_ATTR void update_diagnostic_deadline(int64_t now) {
        auto& diagnostic = state.diagnostic;
        if (!diagnostic.busy || !test_started_ ||
            state.state != State::active || diagnostic.stage < 2 || diagnostic.stage > 3) return;
        if (diagnostic.axis == DiagnosticAxis::q) {
            if (std::abs(angle_.velocity()) > 75) { fail("diagnostic_speed_guard"); return; }
            const auto elapsed = now - test_started_;
            if (elapsed >= int64_t(test_plan_.drive_us) + test_plan_.observe_us) {
                diagnostic.stage = 4; halt(); return;
            }
            diagnostic.stage = elapsed < test_plan_.drive_us ? 2 : 3;
            return;
        }
        const auto action = diagnostic_d_action(test_plan_, now - test_started_,
            angle_.angle() - test_origin_, angle_.velocity());
        if (action == DiagnosticDAction::motion_fault) { fail("diagnostic_motion_guard"); return; }
        if (action == DiagnosticDAction::complete) {
            diagnostic.stage = 4; halt(); return;
        }
        if (action == DiagnosticDAction::release && diagnostic.stage == 2) {
            // End injection even if the next current frame is absent/invalid.
            // Observation remains at zero voltage; this is not an ADC recovery.
            diagnostic.stage = 3;
            state.target = state.target_d = 0;
            current_loop_.reset();
            control_clock_.reset();
            if (board_.write_duty({0, 0, 0}) != ESP_OK) { fail("pwm_failure"); return; }
            state.duty = {}; state.voltage_d = state.voltage_q = 0;
            state.pwm_apply_delay_us = 0;
        }
    }
#endif
    bool setup_board() {
        control_clock_.reset();
        if (!bus_.initialize() || board_.initialize() != ESP_OK || !encoder_.initialize() ||
            board_.start_sampling(xTaskGetCurrentTaskHandle()) != ESP_OK) {
            state.hardware_detail = board_.failed_operation();
            state.hardware_error = board_.initialization_error();
            encoder_.stop(); bus_.stop(); board_.shutdown(); return false;
        }
        state.hardware_detail = "none"; state.hardware_error = 0;
        return true;
    }
    const char* restart_sampling() {
        if (energized()) return "disable_before_save";
        // Flash writes can stall a PWM callback even with the bridge disabled.
        // Restart the sampling owners, preserving the first fault and settings.
        halt(state.state == State::fault ? state.fault : nullptr);
        ready_ = false;
        const bool encoder_stopped = encoder_.stop(pdMS_TO_TICKS(100));
        const bool bus_stopped = bus_.stop(pdMS_TO_TICKS(100));
        if (!encoder_stopped || !bus_stopped) {
            state.hardware_detail = "stop_sampling_workers";
            state.hardware_error = ESP_ERR_TIMEOUT;
            fail("hardware_restart_failed");
            return "hardware_restart_failed";
        }
        board_.shutdown();
        // Board::sample() may still contain its last published pre-restart
        // value. Only callbacks newer than this boundary can refresh telemetry.
        samples_after_ = Board::esp_time();
        last_sample_ = last_encoder_ = metrics_at_ = 0;
        last_sequence_ = metrics_cycles_ = 0;
        sampling_recovery_.reset();
        angle_.reset(); sample_angle_ = output_angle_ = 0;
        state.angle = state.velocity = state.velocity_fast = 0;
        state.id = state.iq = 0; state.currents = {}; state.volts = {};
        state.adc_sample_valid = false; state.sampling_restart_required = true;
        state.adc_completed_mask = state.adc_window_after_mask = state.adc_slot_mask = 0;
        state.adc_done_mask = state.adc_done_down_mask = 0;
        state.adc_done_raw_mask = 0;
        state.adc_done_ticks = {}; state.adc_done_elapsed_us = {};
        state.adc_done_words = {};
        state.adc_armed_words = {};
        state.adc_current_words = {};
        state.pwm_compare_valid = false;
        state.control_hz = state.control_interval_us = state.control_max_us = 0;
        state.control_cycles = 0;
        state.encoder_age_us = state.encoder_age_max_us = 0;
        state.bus_sample = {}; state.bus_voltage_valid = false;
        state.bus_voltage_used = state.effective_voltage_limit = 0;
        ready_ = setup_board();
        update_bus_voltage(Board::esp_time());
        if (!ready_) {
            fail("hardware_restart_failed");
            return "hardware_restart_failed";
        }
        // Do not call reset_trip/clear here: a save must not acknowledge faults.
        return nullptr;
    }
    IRAM_ATTR void update_bus_voltage(int64_t now) {
        if constexpr (BusVoltageMonitor::enabled) {
            state.bus_sample = bus_.sample();
            now = Board::esp_time(); // The core-0 producer may have published after the caller's timestamp.
            const auto& reading = state.bus_sample;
            state.bus_voltage_age_us = reading.timestamp_us > 0 ?
                static_cast<int>(std::clamp<int64_t>(now - reading.timestamp_us, 0, 1000000000)) : 1000000000;
            state.bus_voltage_valid = reading.usable(now);
            state.bus_voltage_used = state.bus_voltage_valid ? reading.volts : 0;
            if (!state.bus_voltage_valid && energized())
                fail(state.bus_voltage_age_us > 100000 ? "bus_voltage_stale" : "bus_voltage_invalid");
        } else {
            state.bus_voltage_used = state.config.bus_voltage;
            state.bus_voltage_valid = max_output_voltage(state.config.bus_voltage) > 0;
        }
        state.effective_voltage_limit = max_output_voltage(state.bus_voltage_used);
    }
    IRAM_ATTR const char* command(const Request& r, int64_t now) {
#if CONFIG_KNOB_DIAGNOSTICS
        if (torque_hold_.active() && r.op != Op::status && r.op != Op::disable)
            return "diagnostic_busy";
        if (state.diagnostic.busy && !r.test_command && r.op != Op::status && r.op != Op::disable)
            return "diagnostic_busy";
        if (r.test_command && state.diagnostic.cancelled &&
            r.op != Op::status && r.op != Op::disable && r.op != Op::configure && r.op != Op::test_end)
            return "diagnostic_cancelled";
#endif
        switch (r.op) {
#if CONFIG_KNOB_DIAGNOSTICS
        case Op::diagnostic_restore_calibration:
            // Restore an explicitly supplied pre-reflash RAM backup. Never
            // energize the bridge or write NVS as a side effect of restoration.
            if (state.state != State::idle) return "idle_required";
            if (!valid_calibration(r.diagnostic_calibration)) return "invalid_calibration";
            state.calibration = r.diagnostic_calibration;
            state.calibration_quality = {};
            return nullptr;
        case Op::diagnostic_hold:
            if (state.state != State::active || !state.current_valid || state.diagnostic.busy ||
                r.diagnostic.current * state.target <= 0 ||
                !torque_hold_.start(r.diagnostic.current, angle_.velocity(), angle_.angle(),
                                    state.config.width, state.config.current_limit, now))
                return "diagnostic_hold_not_ready";
            pulse_start_ = 0; return nullptr;
        case Op::diagnose:
            if (state.state != State::idle) return "idle_required";
            if (!valid_calibration(state.calibration)) return "calibrate_first";
            if (!valid_diagnostic_plan(r.diagnostic, state.config.current_limit)) return "diagnostic_range_or_current_limit";
            if (xQueueSend(test_plans, &r.diagnostic, 0) != pdTRUE) return "diagnostic_busy";
            state.diagnostic = {}; state.diagnostic.busy = true; state.diagnostic.stage = 1;
            state.diagnostic.axis = r.diagnostic.axis;
            test_started_ = 0; return nullptr;
        case Op::diagnostic_read:
            if (state.diagnostic.stage == 6) return "disable_before_trace_read";
            return r.diagnostic_offset <= state.diagnostic.count ? nullptr : "diagnostic_offset";
        case Op::test_begin:
            if (!r.test_command || !state.diagnostic.busy || state.state != State::active) return "diagnostic_not_ready";
            if (!valid_diagnostic_plan(r.diagnostic, state.config.current_limit)) return "diagnostic_range_or_current_limit";
            test_plan_ = r.diagnostic; test_started_ = now; test_sampled_ = 0;
            test_speed_pi_.reset();
            test_origin_ = angle_.angle(); pulse_start_ = 0;
            test_electrical_ = wrap(state.calibration.direction * state.calibration.pole_pairs *
                angle_.wrapped_angle() - state.calibration.electrical_zero);
            state.diagnostic.stage = 2; return nullptr;
        case Op::test_end:
            if (!r.test_command || energized()) return "disable_first";
            state.diagnostic.busy = false; return nullptr;
#endif
        case Op::status: return nullptr;
        case Op::disable:
            halt(state.state == State::fault ? state.fault : nullptr); return nullptr;
        case Op::save: return energized() ? "disable_before_save" : nullptr;
        case Op::restart_sampling: return restart_sampling();
        case Op::clear:
            if (energized()) return "disable_first";
            if (!ready_ || board_.sampling_restart_required())
                if (const char* error = restart_sampling()) return error;
            if (!state.bus_voltage_valid) bus_.request_reinitialize();
            if (!ready_) return "hardware_unavailable";
            board_.reset_trip(); sampling_recovery_.reset(); angle_.reset(); last_encoder_ = 0;
            encoder_.acknowledge_errors();
            state.encoder_error = 0; state.fault_magnet = 0;
            state.fault_currents = {}; state.fault_current_valid = false;
            state.fault_current_trip_limit = 0;
            state.fault_angle = 0; state.fault_velocity = 0; state.fault_velocity_fast = 0;
            state.fault_sample_interval_us = 0; state.fault_sample_age_us = 0;
            state.fault_sample_start_tick = 0; state.fault_sample_end_tick = 0;
            state.fault_sample_us = 0; state.fault_sample_valid = false;
            state.fault_adc_dma_words = {}; state.fault_adc_raw = {}; state.fault_adc_phase = {};
            state.fault_adc_captured = false;
            halt(); update_bus_voltage(now); return nullptr;
        case Op::configure: {
            const auto& c = r.config;
            LiveConfigChange change;
            const auto config_state = state.state == State::idle ? ConfigState::idle :
                state.state == State::active ? ConfigState::active : ConfigState::unavailable;
            if (const char* e = plan_live_config(config_state, state.config, c,
                    state.reference_modes, r.reference_modes, r.reference_update_mask,
                    state.frictionless_config, r.frictionless_update ? &r.frictionless_config : nullptr,
                    change)) return e;
            // Retain overload/peak exposure across live tuning. Configuration
            // changes cannot stand in for valid below-threshold recovery.
            if (c.sense_gain != state.config.sense_gain || c.bus_voltage != state.config.bus_voltage) {
                state.calibration.valid = false; state.calibration_quality = {};
            }
            haptic_.reconfigure(state.config, c, angle_.angle());
            const auto previous_reference = reference_config(c.mode);
            for (int i = 0; i < reference_config_count; ++i)
                if (r.reference_update_mask & (1u << i)) state.reference_modes[i] = r.reference_modes[i];
            if (change.mode) {
                reference_haptic_.reset(angle_.angle(), static_cast<int>(c.mode) - 4, reference_config(c.mode));
                reset_frictionless();
            } else if (change.reference_feel) {
                reference_haptic_.reconfigure(angle_.angle(), static_cast<int>(c.mode) - 4,
                                              previous_reference, reference_config(c.mode));
            }
            if (change.frictionless) state.frictionless_config = r.frictionless_config;
            if (state.state == State::active && change.output) haptic_transition_.begin();
            if (change.output) { quiet_output_.reset(); pulse_start_ = 0; }
            state.config = c;
            update_bus_voltage(now);
            current_limiter_.configure(c.current_limit);
            state.target = std::clamp(state.target, -c.current_limit, c.current_limit);
            state.current_target_limit = std::min(state.current_target_limit, c.current_limit);
            const float trip = overcurrent_threshold(c.current_limit);
            if (state.state == State::active && trip < state.current_trip_limit) {
                // The new torque limit is immediate. Allow winding current to
                // decay before tightening the ordinary overload band. Repeated
                // edits must not postpone an already pending threshold update.
                if (change.current && !trip_update_at_) trip_update_at_ = now + 20000;
            } else {
                state.current_trip_limit = trip; trip_update_at_ = 0;
                if (state.state == State::active) board_.set_trip(state.offsets, c.sense_gain, trip);
            }
            update_haptic_position();
            if (change.electrical) { current_loop_.reset(); control_clock_.reset(); }
            return nullptr;
        }
        case Op::zero:
            if (state.state != State::idle && state.state != State::active) return "busy";
            haptic_transition_.reset(); pulse_start_ = 0;
            haptic_.reset(angle_.angle(), std::clamp(0, state.config.min_position, state.config.max_position));
            reference_haptic_.reset(angle_.angle(), static_cast<int>(state.config.mode) - 4, reference_config(state.config.mode));
            reset_frictionless(); update_haptic_position(); state.sub_position = 0; return nullptr;
        case Op::pulse:
            if (state.state != State::active) return "enable_first";
            pulse_start_ = now; return nullptr;
        case Op::calibrate:
        case Op::enable: {
            if (!ready_) return "hardware_unavailable";
            if (state.state != State::idle) return "idle_required";
            if (!state.bus_voltage_valid) return "bus_voltage_not_ready";
            if (board_.trip()) return "clear_fault_first";
            if (r.op == Op::enable && !valid_calibration(state.calibration)) return "calibrate_first";
            if (r.op == Op::calibrate)
                if (const char* error = validate_calibration_plan(r.calibration_plan)) return error;
            if (r.op == Op::calibrate && r.calibration_plan.max_current > state.config.current_limit)
                return "calibration_exceeds_current_limit";
            const auto startup_sample = board_.sample();
            if (startup_sample.timestamp_us <= samples_after_ ||
                !SamplingRecovery::can_start_offsets(!board_.sampling_restart_required(),
                    startup_sample.timestamp_us, Board::esp_time())) return "adc_window_invalid";
            if (!last_encoder_ || now - last_encoder_ > 2000) return "encoder_not_ready";
            state.control_max_us = 0; state.encoder_age_max_us = 0;
#if CONFIG_KNOB_DIAGNOSTICS
            if (!state.diagnostic.busy) { state.diagnostic = {}; capture_next_ = 0; }
#endif
            calibrating_ = r.op == Op::calibrate;
            if (calibrating_) {
                calibration_plan_ = r.calibration_plan;
                calibration_started_ = now;
                state.calibration_report = {};
                state.calibration_report.stage = "offsets";
                state.calibration_report.quality.current = calibration_plan_.current;
                state.calibration_report.quality.voltage_limit = calibration_plan_.voltage_limit;
            }
            stage_start_ = now; wake_ = false; offsets_.reset(state.config.sense_gain);
            sampling_recovery_.reset(now); current_loop_.reset(); offset_start_ = angle_.angle();
            control_clock_.reset();
            encoder_freshness_.reset();
            current_trip_filter_.reset();
            quiet_output_.reset(); state.quiet_output = false;
            current_limiter_.reset(state.config.current_limit);
            board_.write_duty({0,0,0}); state.state = State::offsets; return nullptr;
        }
        }
        return "unknown_op";
    }
    IRAM_ATTR void control(const RawSample& raw, float dt, float frame_dt, int64_t now) {
        const auto& c = state.config;
        state.rejected = raw.rejected; state.sample_us = raw.duration_ticks / 80.0f;
        if (state.state == State::offsets) {
            // Allow zero duty to commit before waking the bridge; wait for CSA startup.
            if (now - stage_start_ > 10000 && !wake_) { board_.enable(true); wake_ = true; }
            if (now - stage_start_ < 30000) return;
            if (std::abs(angle_.angle() - offset_start_) > 0.03f) { fail("keep_rotor_still_for_offsets"); return; }
            for (int k = 0; k < 2; ++k) {
                const int p = raw.phase[k]; const float v = board_.volts(p, raw.raw[k]);
                offsets_.add(p, v); state.volts[p] = v;
            }
            state.offset_noise_a = offsets_.noise_a();
            state.offset_sem_a = offsets_.standard_error_a();
            state.offset_drift_a = offsets_.drift_a();
            if (calibrating_) {
                state.calibration_report.quality.noise_a = state.offset_noise_a;
                state.calibration_report.quality.sem_a = state.offset_sem_a;
                state.calibration_report.quality.drift_a = state.offset_drift_a;
            }
            if (offsets_.error()) { fail(offsets_.error()); return; }
            if (!offsets_.complete()) return;
            state.offsets = offsets_.means();
            state.current_trip_limit = overcurrent_threshold(calibrating_ ? calibration_plan_.max_current : c.current_limit);
            board_.set_trip(state.offsets, c.sense_gain, state.current_trip_limit);
            if (calibrating_) {
                if (!calibration_sequence_.start(calibration_plan_, now)) { fail("calibration_parameters"); return; }
                state.state = State::align; stage_start_ = now;
                update_calibration_report(now);
            }
            else {
                state.state = State::active;
                haptic_.reset(angle_.angle(), std::clamp(0,c.min_position,c.max_position));
                reference_haptic_.reset(angle_.angle(), static_cast<int>(c.mode) - 4, reference_config(c.mode));
                reset_frictionless(); haptic_speed_.reset(); haptic_transition_.reset();
            }
            return;
        }
        if (!energized()) return;
#if CONFIG_KNOB_DIAGNOSTICS
        if (test_started_ && state.diagnostic.busy && state.diagnostic.stage < 4 &&
            state.diagnostic.axis == DiagnosticAxis::q) {
            if (std::abs(angle_.velocity()) > 75) { fail("diagnostic_speed_guard"); return; }
            const int64_t elapsed = now - test_started_;
            if (elapsed >= test_plan_.drive_us + test_plan_.observe_us) {
                state.diagnostic.stage = 4; halt(); return;
            }
            state.diagnostic.stage = elapsed < test_plan_.drive_us ? 2 : 3;
        }
#endif
        float sum = 0;
        for (int k = 0; k < 2; ++k) {
            const int p = raw.phase[k]; const float v = board_.volts(p, raw.raw[k]);
            if (v < 0.25f || v > 3.05f) {
                capture_fault_sample(raw, static_cast<int>(frame_dt * 1000000 + .5f), now);
                fail("adc_saturation"); return;
            }
            state.volts[p] = v;
            // DRV8316C Fig 8-30: positive current out of OUTx raises SOx.
            state.currents[p] = phase_current(v, state.offsets[p], c.sense_gain);
            sum += state.currents[p];
        }
        state.currents[3 - raw.phase[0] - raw.phase[1]] = -sum;
        if (trip_update_at_ && now >= trip_update_at_) {
            state.current_trip_limit = overcurrent_threshold(c.current_limit);
            board_.set_trip(state.offsets, c.sense_gain, state.current_trip_limit);
            trip_update_at_ = 0;
        }
        float phase_peak = 0;
        for (float i : state.currents) {
            if (!std::isfinite(i)) { fail("overcurrent"); return; }
            phase_peak = std::max(phase_peak, std::abs(i));
        }
        const auto current_level = phase_peak >= absolute_current_trip ? CurrentTripLevel::emergency :
                                   phase_peak >= peak_current_trip ? CurrentTripLevel::peak :
                                   phase_peak >= state.current_trip_limit ? CurrentTripLevel::overload : CurrentTripLevel::normal;
        // A rejected frame is unknown current, not evidence of continued
        // overload or recovery. Keep protection on observed-frame exposure.
        if (current_trip_filter_.step(current_level, static_cast<unsigned>(frame_dt * 1000000 + .5f))) {
            capture_fault_sample(raw, static_cast<int>(frame_dt * 1000000 + .5f), now);
            state.fault_currents = state.currents; state.fault_current_valid = true;
            state.fault_current_trip_limit = state.current_trip_limit;
            state.fault_angle = state.angle;
            state.fault_velocity = state.velocity;
            state.fault_velocity_fast = state.velocity_fast;
            fail("overcurrent"); return;
        }
        float electrical = wrap(state.calibration.direction * state.calibration.pole_pairs * sample_angle_ - state.calibration.electrical_zero);
        Dq target{};
        bool zero_torque_intent = false;
        bool d_test = false, d_observing = false;
        bool calibration_voltage_mode = false;
        float calibration_voltage = 0;
#if CONFIG_KNOB_DIAGNOSTICS
        d_test = state.diagnostic.busy && state.diagnostic.axis == DiagnosticAxis::d;
        d_observing = d_test && state.diagnostic.stage != 2;
#endif
        if (state.state == State::active) {
            if (d_test) {
#if CONFIG_KNOB_DIAGNOSTICS
                // A stationary stator vector can settle the unloaded rotor;
                // following the rotor would turn calibration error into a
                // persistent torque. Diagnostic Id/Iq use this fixed frame.
                electrical = test_electrical_;
                target.d = d_observing ? 0 : diagnostic_d_target(test_plan_, now - test_started_);
                state.target = 0; state.haptic_scale = 0;
                state.speed_limited = state.haptic_paused = false;
                zero_torque_intent = d_observing;
#endif
            } else {
                bool drive_override = false;
#if CONFIG_KNOB_DIAGNOSTICS
                drive_override = state.diagnostic.busy && state.diagnostic.stage == 2;
#endif
                if (c.mode == Mode::frictionless) {
                    // Keep the familiar USB angle/count telemetry in free rotation.
                    Config tracking = c; tracking.mode = Mode::free;
                    haptic_.step(tracking, output_angle_, angle_.haptic_velocity());
                    state.target = frictionless_.step(state.frictionless_config, angle_.haptic_velocity(), dt,
                        std::min(c.current_limit, current_limiter_.capacity()), drive_override);
                    state.frictionless_phase = frictionless_.phase();
                    state.frictionless_reference = frictionless_.reference();
                    state.frictionless_correction = frictionless_.correction();
                    state.frictionless_feedforward = frictionless_.feedforward();
                } else if (reference_mode(c.mode)) {
                    state.target = reference_haptic_.step(static_cast<int>(c.mode) - 4,
                        reference_config(c.mode), output_angle_, angle_.haptic_velocity(), dt, c.current_limit);
                } else state.target = haptic_.step(c, output_angle_, angle_.haptic_velocity());
#if CONFIG_KNOB_DIAGNOSTICS
                if (torque_hold_.active()) state.target = torque_hold_.current();
#endif
                if (pulse_start_ && c.mode != Mode::frictionless) {
                    const int64_t elapsed = now - pulse_start_;
                    if (elapsed < 6000) state.target += elapsed < 3000 ? 0.08f : -0.08f;
                    else pulse_start_ = 0;
                }
                const float mode_limit = reference_mode(c.mode) && c.mode != Mode::frictionless ?
                    std::min(c.current_limit, reference_current_limit) : c.current_limit;
                state.target = haptic_transition_.step(state.target, mode_limit, dt);
                zero_torque_intent = state.target == 0;
                state.target = haptic_speed_.step(state.target, angle_.velocity(), mode_limit, dt);
                state.haptic_scale = haptic_speed_.scale();
                state.speed_limited = haptic_speed_.limited(); state.haptic_paused = haptic_speed_.paused();
                if (state.haptic_paused && c.mode == Mode::frictionless) reset_frictionless();
#if CONFIG_KNOB_DIAGNOSTICS
                if (state.diagnostic.busy && state.diagnostic.stage == 2) {
                    zero_torque_intent = false;
                    // Bounded external-motion stimulus. Haptics still track angle
                    // and speed, but the test current emulates a hand-driven spin.
                    const float ramp = std::min(1.0f, (now - test_started_) / 50000.0f);
                    state.target = test_speed_pi_.step(test_plan_.velocity * ramp - angle_.velocity(),
                        dt, .015f, .04f, std::min(test_plan_.current, c.current_limit));
                }
#endif
                // Haptic torque uses sensor coordinates; electromagnetic torque uses electrical coordinates.
                target.q = state.target * state.calibration.direction;
            }
        } else {
            CalibrationSequenceInput input{};
            input.now_us = now; input.rotor_angle = angle_.angle();
            input.currents = state.currents;
            input.measured_phases = {raw.phase[0], raw.phase[1]};
            input.output_voltage = std::sqrt(state.voltage_d * state.voltage_d + state.voltage_q * state.voltage_q);
            input.available_voltage_limit = state.effective_voltage_limit;
            const auto command = calibration_sequence_.tick(input);
            update_calibration_report(now);
            if (command.error) { fail(command.error); return; }
            if (command.done) {
                const auto candidate = calibration_sequence_.candidate();
                if (!valid_calibration(candidate) || !valid_calibration_quality(state.calibration_report.quality)) {
                    fail("calibration_quality_invalid"); return;
                }
                state.calibration = candidate;
                state.calibration_quality = state.calibration_report.quality;
                calibrating_ = false; halt(); return;
            }
            const auto stage = calibration_sequence_.report().stage;
            state.state = stage == CalibrationStage::forward || stage == CalibrationStage::reverse ? State::sweep :
                stage == CalibrationStage::verify || stage == CalibrationStage::evaluate ? State::settle : State::align;
            electrical = wrap(command.angle); target.d = command.current;
            calibration_voltage_mode = command.voltage_mode;
            calibration_voltage = command.voltage_d;
            if (command.reset_controller) current_loop_.reset();
        }
        const Dq measured = park(state.currents, electrical);
        if (state.state == State::active) {
            // Foldback and budget recovery also require observed current;
            // elapsed blind time must not replenish the operating budget.
            if (d_test) {
#if CONFIG_KNOB_DIAGNOSTICS
                target.d = limit_diagnostic_d_current(current_limiter_, target.d, measured,
                    phase_peak, c.current_limit, frame_dt);
#endif
            } else {
                target.q = current_limiter_.step(target.q, measured, phase_peak, c.current_limit, frame_dt);
                state.target = target.q * state.calibration.direction;
            }
            state.current_target_limit = current_limiter_.capacity();
            state.current_limited = current_limiter_.limited();
        }
        state.target_d = target.d;
        state.id = measured.d; state.iq = measured.q;
        state.current_valid = true;
        const float limit = state.state == State::active ? state.effective_voltage_limit :
            std::min(state.effective_voltage_limit, calibration_plan_.voltage_limit);
        // Supply winding voltage on the same tick as the torque request. The
        // integral then corrects model error/back-EMF instead of remembering a
        // several-millisecond detent pulse after the requested torque is zero.
        constexpr float resistance = CONFIG_KNOB_PHASE_RESISTANCE_MILLIOHM * .001f;
        // FR-L tracks zero Iq even at rest; a zero-voltage short adds drag.
        state.quiet_output = d_observing || quiet_output_.step(state.state == State::active &&
                                                              c.mode != Mode::frictionless && zero_torque_intent,
                                                              angle_.velocity(), dt);
        Dq voltage{};
        if (state.quiet_output) current_loop_.reset();
        else if (calibration_voltage_mode) {
            current_loop_.reset(); voltage.d = std::clamp(calibration_voltage, -limit, limit);
        }
        else voltage = current_loop_.step({target.d - measured.d, target.q - measured.q}, dt,
            c.current_kp, c.current_ki, limit, {resistance * target.d, resistance * target.q});
        // Park uses the acquisition angle. The HAL predicts the actual shadow
        // latch from PWM phase and queues the resulting duty in one operation.
        const float electrical_velocity = state.state == State::active && !d_test ?
            state.calibration.direction * state.calibration.pole_pairs * angle_.haptic_velocity() : 0;
        if (state.quiet_output) {
            state.duty = {}; state.pwm_apply_delay_us = 0;
            if (board_.write_duty({0, 0, 0}) != ESP_OK) { fail("pwm_failure"); return; }
        } else {
            const esp_err_t written = board_.write_voltage(voltage, electrical, electrical_velocity, raw.timestamp_us,
                state.bus_voltage_used, state.duty, state.pwm_apply_delay_us);
            // A busy shadow-commit interval keeps the previous coherent command.
            // The next fresh control update retries without dropping the bridge.
            if (written == ESP_ERR_TIMEOUT) { ++state.pwm_deferred; return; }
            if (written != ESP_OK) { fail("pwm_failure"); return; }
        }
        state.voltage_d = voltage.d; state.voltage_q = voltage.q;
#if CONFIG_KNOB_DIAGNOSTICS
        auto& diagnostic = state.diagnostic;
        if (diagnostic.busy && diagnostic.stage >= 2 && diagnostic.stage <= 3) {
            diagnostic.peak_speed = std::max(diagnostic.peak_speed, std::abs(angle_.velocity()));
            diagnostic.peak_current = std::max(diagnostic.peak_current, phase_peak);
            if (diagnostic.stage == 3 && state.haptic_paused && state.haptic_scale <= 1e-6f &&
                std::abs(state.target) <= 1e-5f) ++diagnostic.muted_samples;
            if (diagnostic.stage == 3 && diagnostic.muted_samples && !state.speed_limited)
                diagnostic.recovered = true;
            // Size capture cadence to the complete bounded run, including its
            // release tail; long FR-L tests must not fill the buffer early.
            const int capture_period_us = diagnostic_capture_period_us(test_plan_);
            if ((!test_sampled_ || now - test_sampled_ >= capture_period_us) && diagnostic.count < diagnostic_capacity) {
                test_sampled_ = now;
                test_samples_[diagnostic.count++] = {
                    static_cast<uint32_t>(now - test_started_), angle_.angle(), angle_.velocity(), state.target,
                    state.iq, state.id, state.haptic_scale, state.voltage_q, phase_peak,
                    static_cast<uint32_t>(diagnostic.stage | (state.haptic_paused ? 256 : 0) |
                                          (state.speed_limited ? 512 : 0) |
                                          ((raw.phase[0] | (raw.phase[1] << 2)) << 16)),
                    angle_.haptic_velocity(), state.voltage_d,
                    static_cast<uint16_t>(raw.raw[0]), static_cast<uint16_t>(raw.raw[1]),
                    static_cast<int16_t>(raw.start_tick), static_cast<int16_t>(raw.end_tick), state.target_d,
                    static_cast<float>(state.frictionless_phase), state.frictionless_reference,
                    state.frictionless_correction, state.frictionless_feedforward};
            }
        }
        // Diagnostic builds keep the preceding ~410 ms at the full control
        // rate during a manual session. Freeze on disable/fault; reuse the
        // test buffer without sampling interrupts or slow USB status polling.
        if (!diagnostic.busy && state.state == State::active) {
            if (diagnostic.stage != 6) {
                diagnostic = {}; diagnostic.stage = 6;
                capture_started_ = now; capture_next_ = 0;
            }
            diagnostic.peak_speed = std::max(diagnostic.peak_speed, std::abs(angle_.velocity()));
            diagnostic.peak_current = std::max(diagnostic.peak_current, phase_peak);
            test_samples_[capture_next_] = {
                static_cast<uint32_t>(now - capture_started_), state.angle, state.velocity, state.target,
                state.iq, state.id, state.haptic_scale, state.voltage_q, phase_peak,
                static_cast<uint32_t>(6 | (state.quiet_output ? 1024 : 0) | (torque_hold_.active() ? 2048 : 0) |
                    ((raw.phase[0] | (raw.phase[1] << 2)) << 16)),
                angle_.haptic_velocity(), state.voltage_d,
                static_cast<uint16_t>(raw.raw[0]), static_cast<uint16_t>(raw.raw[1]),
                static_cast<int16_t>(raw.start_tick), static_cast<int16_t>(raw.end_tick), state.target_d,
                static_cast<float>(state.frictionless_phase), state.frictionless_reference,
                state.frictionless_correction, state.frictionless_feedforward};
            capture_next_ = (capture_next_ + 1) % diagnostic_capacity;
            diagnostic.count = std::min(diagnostic_capacity, diagnostic.count + 1);
        }
#endif
    }
    void update_calibration_report(int64_t now) {
        const auto& report = calibration_sequence_.report();
        auto& result = state.calibration_report;
        result.stage = calibration_stage_name(report.stage);
        result.error = report.error ? report.error : "none";
        result.passed = report.passed; result.cancelled = report.cancelled;
        result.attempts = report.attempts; result.point = report.point_index;
        result.elapsed_s = (now - calibration_started_) * 1e-6f;
        result.phase_response = report.phase_currents;
        result.observed_d = report.observed_d; result.observed_q = report.observed_q;
        result.tracking_error = report.tracking_error; result.saturation_fraction = report.saturation_fraction;
        auto& q = result.quality;
        q.method = report.passed ? 1 : 0;
        q.current = report.current; q.voltage_limit = report.voltage_limit;
        q.forward_points = report.forward_points; q.reverse_points = report.reverse_points;
        q.verification_points = report.verify_points;
        q.phase_check_passed = report.phase_check_passed;
        const auto& fit = report.fit_result;
        result.zero_forward = fit.forward_zero; result.zero_reverse = fit.reverse_zero;
        q.measured_pole_pairs = fit.pole_pairs;
        q.forward_pole_pairs = fit.forward_pole_pairs; q.reverse_pole_pairs = fit.reverse_pole_pairs;
        q.direction_difference = fit.direction_difference; q.hysteresis_max = fit.hysteresis_max;
        q.residual_rms = fit.residual_rms; q.residual_max = fit.residual_max;
        q.verification_max = report.verification_max;
    }
    void IRAM_ATTR main() override {
        state.current_trip_limit = overcurrent_threshold(state.config.current_limit);
        heartbeat_ = Board::esp_time();
        ready_ = setup_board();
        if (!ready_) fail("hardware_init_failed");
        while (!should_exit()) {
            if (ready_) ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5)); else vTaskDelay(pdMS_TO_TICKS(5));
            int64_t now = Board::esp_time();
            update_bus_voltage(now);
            if (energized() && board_.trip()) {
                if (board_.trip() == 1) capture_overcurrent();
                fail(board_.trip() == 1 ? "overcurrent_isr" : "control_watchdog");
            }
            if (state.state == State::offsets && now - stage_start_ > 2000000) fail("current_offsets_timeout");
            if (calibrating_) {
                state.calibration_report.elapsed_s = (now - calibration_started_) * 1e-6f;
                if (now - calibration_started_ > 121000000) fail("calibration_timeout");
                else if (state.state != State::offsets) {
                    const auto command = calibration_sequence_.check_timeout(now);
                    if (command.error) { update_calibration_report(now); fail(command.error); }
                    else if (command.reset_controller) {
                        current_loop_.reset();
                        if (command.current == 0 && command.voltage_d == 0) {
                            if (board_.write_duty({0,0,0}) != ESP_OK) fail("pwm_failure");
                            state.duty = {}; state.voltage_d = state.voltage_q = 0;
                        }
                    }
                }
            }
#if CONFIG_KNOB_DIAGNOSTICS
            if (state.state == State::active && torque_hold_.must_stop(now, angle_.angle(), angle_.velocity())) halt();
            update_diagnostic_deadline(now);
#endif
            Request request{};
            bool received = xQueueReceive(requests, &request, 0) == pdTRUE;
            QueueHandle_t reply_queue = replies;
#if CONFIG_KNOB_DIAGNOSTICS
            if (!received && xQueueReceive(test_requests, &request, 0) == pdTRUE) {
                received = true; request.test_command = true; reply_queue = test_replies;
            }
#endif
            if (received) {
                const char* error = command(request, now);
                // Scanning the unused stack on every status request can take
                // a feedback period. Read its watermark only after stopping;
                // the retained low-water mark still captures prior peak use.
                if (!energized()) state.control_stack_free = uxTaskGetStackHighWaterMark(nullptr);
                // Well-formed but refused commands cannot renew torque either.
                if (!error && request.renew_lease) heartbeat_ = now;
                Reply reply{error, state, request.id};
                reply.reference_defaults = !error && request.op == Op::status && request.reference_defaults;
#if CONFIG_KNOB_DIAGNOSTICS
                if (!error && request.op == Op::diagnostic_read) {
                    reply.diagnostic_offset = request.diagnostic_offset;
                    reply.diagnostic_count = std::min(4, state.diagnostic.count - request.diagnostic_offset);
                    const int first = state.diagnostic.stage == 7 && state.diagnostic.count == diagnostic_capacity
                        ? capture_next_ : 0;
                    for (int i = 0; i < reply.diagnostic_count; ++i)
                        reply.diagnostic_samples[i] = test_samples_[(first + request.diagnostic_offset + i) % diagnostic_capacity];
                }
#endif
                xQueueOverwrite(reply_queue, &reply);
                // A hardware restart can take milliseconds; resume with a new
                // timestamp rather than compare new samples with pre-restart now.
                if (request.op == Op::restart_sampling || request.op == Op::clear) continue;
            }
            if (!ready_) continue;
            // The ISR watchdog measures task liveness, independently of current
            // window availability. Invalid ADC observations are handled below.
            board_.kick();
#if CONFIG_KNOB_HOST_LEASE_MS > 0
            if (energized() && now - heartbeat_ > CONFIG_KNOB_HOST_LEASE_MS * 1000LL) { fail("host_timeout"); continue; }
#endif
            const auto raw = board_.sample();
            now = Board::esp_time();
            state.pwm_peak_duty = board_.peak_duty();
            state.pwm_full_duty_cycles = board_.full_duty_cycles();
            state.adc_sample_valid = raw.valid && raw.timestamp_us > samples_after_;
            state.adc_reject_reasons = raw.rejection_reasons;
            std::copy_n(raw.rejection_counts, state.adc_reject_counts.size(), state.adc_reject_counts.begin());
            state.sampling_restart_required = board_.sampling_restart_required();
            if (state.sampling_restart_required && energized()) {
                fail("adc_transport_failure"); continue;
            }
            state.adc_dma_words = {raw.dma_words[0], raw.dma_words[1]};
            // A notification can arrive before sample() reads its newer data.
            // The next wake then refers to a sample already consumed here.
            // Never integrate the same current measurement twice, but keep
            // checking encoder health and recovery even if ADC frames stop.
            const bool new_sample = raw.timestamp_us > last_sample_ && raw.timestamp_us > samples_after_;
            if (!new_sample) ++state.reused_samples;
            if (new_sample && last_sequence_ && raw.sequence > last_sequence_ + 1)
                state.missed_samples += raw.sequence - last_sequence_ - 1;
            if (new_sample) last_sequence_ = raw.sequence;
            state.current_valid = false;
            state.rejected = raw.rejected; state.sample_us = raw.duration_ticks / 80.0f;
            state.sample_start_tick = raw.start_tick; state.sample_end_tick = raw.end_tick;
            std::copy_n(raw.adc_slot_words, 3, state.adc_slot_words.begin());
            std::copy_n(raw.adc_trigger_ticks, 3, state.adc_trigger_ticks.begin());
            std::copy_n(raw.adc_callback_ticks, 3, state.adc_callback_ticks.begin());
            std::copy_n(raw.adc_trigger_sar, 3, state.adc_trigger_sar.begin());
            std::copy_n(raw.adc_callback_sar, 3, state.adc_callback_sar.begin());
            std::copy_n(raw.adc_callback_elapsed_us, 3, state.adc_callback_elapsed_us.begin());
            std::copy_n(raw.adc_trigger_elapsed_us, 3, state.adc_trigger_elapsed_us.begin());
            for (int slot = 0; slot < 3; ++slot) {
                std::copy_n(raw.adc_pwm_compare_before[slot], 3, state.adc_pwm_compare_before[slot].begin());
                std::copy_n(raw.adc_pwm_compare_after[slot], 3, state.adc_pwm_compare_after[slot].begin());
            }
            state.phase_skew_us = raw.phase_skew_us;
            state.adc_completed_mask = raw.adc_completed_mask;
            state.adc_done_mask = raw.adc_done_mask;
            state.adc_done_raw_mask = raw.adc_done_raw_mask;
            state.adc_done_down_mask = raw.adc_done_down_mask;
            std::copy_n(raw.adc_done_ticks, 3, state.adc_done_ticks.begin());
            std::copy_n(raw.adc_done_words, 3, state.adc_done_words.begin());
            std::copy_n(raw.adc_armed_words, 3, state.adc_armed_words.begin());
            std::copy_n(raw.current_words, 2, state.adc_current_words.begin());
            std::copy_n(raw.adc_done_elapsed_us, 3, state.adc_done_elapsed_us.begin());
            state.adc_window_after_mask = raw.adc_window_after_mask;
            state.adc_slot_mask = raw.adc_slot_mask;
            state.adc_trigger_down_mask = raw.adc_trigger_down_mask;
            state.adc_callback_down_mask = raw.adc_callback_down_mask;
            std::copy_n(raw.pwm_compare_before, 3, state.pwm_compare_before.begin());
            std::copy_n(raw.pwm_compare_after, 3, state.pwm_compare_after.begin());
            state.pwm_latch_sequence = raw.pwm_latch_sequence;
            state.pwm_compare_valid = raw.pwm_compare_valid;
            const float elapsed_dt = last_sample_ && new_sample ?
                (raw.timestamp_us - last_sample_) * 0.000001f : 1.0f / Board::control_hz;
            const float frame_dt = SamplingRecovery::integration_dt(elapsed_dt);
            const bool valid_sample = new_sample && SamplingRecovery::sample_usable(raw.valid,
                raw.timestamp_us, last_sample_, now);
            if (raw.timestamp_us > last_sample_) last_sample_ = raw.timestamp_us;
            const auto encoder = encoder_.consume_frame();
            state.encoder_us = encoder.duration_us;
            state.encoder_retries = encoder.retries; state.encoder_failures = encoder.failures;
            state.encoder_age_us = encoder.timestamp_us ? std::max<int64_t>(0, now - encoder.timestamp_us) : 0;
            state.encoder_age_max_us = std::max(state.encoder_age_max_us, state.encoder_age_us);
            if (encoder.timestamp_us && encoder.timestamp_us != last_encoder_) {
                const float encoder_dt = last_encoder_ ? (encoder.timestamp_us - last_encoder_) * 1e-6f : .0004f;
                last_encoder_ = encoder.timestamp_us;
                state.magnet = encoder.magnet;
                if (encoder.error != ESP_OK) {
                    if (state.state != State::fault) {
                        state.encoder_error = encoder.error; state.fault_magnet = encoder.magnet;
                    }
                    fail(encoder_fault_name(encoder.fault));
                    continue;
                }
                if (!angle_.update(encoder.angle, encoder_dt)) { fail("encoder_jump"); continue; }
                state.angle = angle_.angle(); state.velocity = angle_.velocity(); state.velocity_fast = angle_.haptic_velocity();
            }
#if CONFIG_KNOB_DIAGNOSTICS
            update_diagnostic_deadline(now);
#endif
            const auto encoder_action = encoder_freshness_.observe(state.encoder_age_us);
            if (!last_encoder_ || encoder_action == EncoderFreshnessAction::lost) {
                if (energized()) fail("encoder_stale");
                continue;
            }
            if (encoder_action != EncoderFreshnessAction::ready) {
                // The prediction horizon is only 2 ms. Pause torque through a
                // short sensor delay and resume automatically on a fresh frame.
                ++state.timing_skips;
                ++state.encoder_timing_skips;
                if (calibrating_ && state.state != State::offsets) {
                    const auto command = calibration_sequence_.invalidate(now);
                    if (command.error) { update_calibration_report(now); fail(command.error); continue; }
                }
                if (energized() && encoder_action == EncoderFreshnessAction::zero_voltage)
                    zero_for_feedback_recovery();
                continue;
            }
            // Sensor latency belongs to the selected encoder adapter. Predict
            // from its timestamp independently of the fresh ADC dt used by PI.
            const float sample_horizon = std::clamp((raw.timestamp_us - last_encoder_) * 1e-6f + encoder_config::sensor_latency_s, -.001f, .002f);
            sample_angle_ = wrap(angle_.wrapped_angle() + angle_.haptic_velocity() * sample_horizon);
            output_angle_ = angle_.haptic_angle(
                std::clamp((now - last_encoder_) * 1e-6f + encoder_config::sensor_latency_s, 0.0f, .002f));
            if (energized()) {
                if (board_.trip()) {
                    if (board_.trip() == 1) capture_overcurrent();
                    fail(board_.trip() == 1 ? "overcurrent_isr" : "control_watchdog"); continue;
                }
                // Active haptics fade out without disabling current regulation.
                // Calibration has no haptic envelope and still aborts on motion.
                if (state.state != State::active && std::abs(angle_.velocity()) >= haptic_speed_cutoff) {
                    fail("calibration_motion_too_fast"); continue;
                }
            }
            state.rejected = raw.rejected; state.sample_us = raw.duration_ticks / 80.0f;
            if (!valid_sample) {
                // A request/duplicate notification can revisit a still-fresh
                // frame. It contributes no evidence, but is not a rejected
                // measurement and must not erase the qualification window.
                if (calibrating_ && state.state != State::offsets &&
                    (new_sample || now - raw.timestamp_us > SamplingRecovery::maximum_sample_age_us)) {
                    const auto command = calibration_sequence_.invalidate(now);
                    if (command.error) { update_calibration_report(now); fail(command.error); continue; }
                    if (command.reset_controller) zero_for_feedback_recovery();
                }
                current_trip_filter_.step(CurrentTripLevel::invalid, 0);
                if (new_sample || now - raw.timestamp_us > SamplingRecovery::maximum_sample_age_us) {
                    ++state.timing_skips;
                    ++state.adc_timing_skips;
                }
                if (energized() && sampling_recovery_.observe(false, now)) {
                    // Reopen current windows without a latched fault or driver
                    // disable. PI resumes from zero on the next good sample.
                    zero_for_feedback_recovery();
                }
                continue;
            }
            sampling_recovery_.observe(true, now);
            const auto interval = control_clock_.accept(raw.timestamp_us, Board::control_period_us);
            const int64_t control_started = Board::esp_time();
            control(raw, interval.integration_s, frame_dt, now);
            state.control_us = Board::esp_time() - control_started;
            state.control_max_us = std::max(state.control_max_us, state.control_us);
            state.control_interval_us = interval.elapsed_s * 1000000;
            ++state.control_cycles; ++metrics_cycles_;
            if (!metrics_at_) metrics_at_ = now;
            if (now - metrics_at_ >= 500000) {
                state.control_hz = metrics_cycles_ * 1000000.0f / (now - metrics_at_);
                metrics_cycles_ = 0; metrics_at_ = now;
            }
            update_haptic_position();
            state.loop_us = Board::esp_time() - now;
        }
        board_.enable(false); encoder_.stop(); bus_.stop(); board_.shutdown();
    }
};
class Console final : public AppTask {
public:
    explicit Console(Motor& motor) : AppTask("knob_console", 24576, 5, 0), motor_(motor) {}
private:
    Motor& motor_;
    bool transact(const Request& request, Reply& reply, int64_t deadline) {
        if (xQueueSend(motor_.requests, &request, pdMS_TO_TICKS(20)) != pdTRUE) return false;
        while (Board::esp_time() < deadline) {
            Reply received{};
            if (xQueueReceive(motor_.replies, &received, pdMS_TO_TICKS(10)) == pdTRUE &&
                received.request_id == request.id) {
                reply = received;
                return true;
            }
        }
        return false;
    }
    void save_and_restart(const Request& request, Reply& reply) {
        const bool saved = save_settings(reply.snapshot.config, reply.snapshot.calibration, reply.snapshot.calibration_quality, reply.snapshot.frictionless_config, reply.snapshot.reference_modes);
        // An unsuccessful NVS write can also have suspended callbacks. Always
        // rebuild sampling after an attempted write, without clearing a fault.
        Request internal{};
        internal.id = request.id;
        internal.op = Op::restart_sampling;
        internal.renew_lease = false;
        const int64_t deadline = Board::esp_time() + 400000;
        if (!transact(internal, reply, deadline) || reply.error) {
            reply.error = "hardware_restart_failed";
            return;
        }
        internal.op = Op::status;
        while (Board::esp_time() < deadline) {
            if (!transact(internal, reply, deadline) || reply.error) break;
            // Completed slots were cleared by the restart. This status must
            // contain a new complete ADC burst, not the pre-flash snapshot.
            // A pre-existing bus/encoder fault may remain; save never clears it.
            if (reply.snapshot.adc_sample_valid && !reply.snapshot.sampling_restart_required &&
                reply.snapshot.adc_completed_mask == 7) {
                reply.error = saved ? nullptr : "save_failed";
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        reply.error = "hardware_restart_failed";
    }
    void process_line(const char* line) {
        Request request{};
        const char* decode_error = decode_request(line, request);
        // A rejected frame still reports real state, without renewing the lease.
        if (decode_error) { request.op = Op::status; request.renew_lease = false; }
        Reply reply{};
        {
            const int64_t deadline = Board::esp_time() + 200000;
            const bool matched = transact(request, reply, deadline);
            if (!matched) reply.error = "control_unavailable";
            else if (decode_error) reply.error = decode_error;
            else if (!reply.error && request.op == Op::save) save_and_restart(request, reply);
        }
        cJSON* root = encode_reply(request.id, reply);
        if (!root) return;
        char* encoded = cJSON_PrintUnformatted(root);
        if (encoded) {
            // Leading LF resynchronizes after boot logs or an interrupted TX.
            if (console_write("\n", 1) && console_write(encoded, std::strlen(encoded))) console_write("\n", 1);
            cJSON_free(encoded);
        }
        cJSON_Delete(root);
    }
    void main() override {
        char line[2048]{}; size_t size = 0; bool discard = false;
        while (!should_exit()) {
            uint8_t bytes[128];
            const int count = console_read(bytes, sizeof(bytes));
            for (int i = 0; i < count; ++i) {
                const uint8_t c = bytes[i];
                if (c == '\n') {
                    if (!discard && size) { line[size] = 0; process_line(line); }
                    size = 0; discard = false;
                } else if (c != '\r') {
                    if (c < 32 || size >= sizeof(line) - 1) discard = true;
                    else if (!discard) line[size++] = c;
                }
            }
        }
    }
};
#if CONFIG_KNOB_DIAGNOSTICS
// Owns only the test sequence. All motor state, timing and peripherals stay on
// the control task; a separate request/reply pair cannot steal host replies.
class DiagnosticTask final : public AppTask {
public:
    explicit DiagnosticTask(Motor& motor) : AppTask("knob_test", 12288, 4, 0), motor_(motor) {}
private:
    Motor& motor_;
    int sequence_ = 0;
    bool transact(Request request, Reply& reply) {
        request.id = ++sequence_;
        if (xQueueSend(motor_.test_requests, &request, pdMS_TO_TICKS(50)) != pdTRUE) return false;
        const int64_t deadline = Board::esp_time() + 200000;
        while (Board::esp_time() < deadline)
            if (xQueueReceive(motor_.test_replies, &reply, pdMS_TO_TICKS(10)) == pdTRUE &&
                reply.request_id == request.id) return !reply.error;
        return false;
    }
    void run(const DiagnosticPlan& plan) {
        Reply reply{};
        Request request{};
        const bool have_original = transact(request, reply);
        const Config original = reply.snapshot.config;
        bool ready = have_original && !reply.snapshot.diagnostic.cancelled && !should_exit();
        if (ready) {
            request.op = Op::configure; request.config = original; request.config.mode = plan.mode;
            ready = transact(request, reply);
        }
        if (ready) {
            request.op = Op::enable; ready = transact(request, reply);
        }
        if (ready) {
            ready = false;
            const int64_t deadline = Board::esp_time() + 2000000;
            while (!should_exit() && Board::esp_time() < deadline) {
                request.op = Op::status;
                if (!transact(request, reply) || reply.snapshot.diagnostic.cancelled ||
                    reply.snapshot.state == State::fault) break;
                if (reply.snapshot.state == State::active) { ready = true; break; }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        if (ready && !should_exit()) {
            request.op = Op::test_begin; request.diagnostic = plan;
            ready = transact(request, reply);
            const int64_t deadline = Board::esp_time() + int64_t(plan.drive_us) + plan.observe_us + 2000000;
            while (ready && !should_exit() && Board::esp_time() < deadline) {
                request.op = Op::status;
                if (!transact(request, reply) || reply.snapshot.diagnostic.stage >= 4 ||
                    reply.snapshot.state == State::fault) break;
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        request.op = Op::disable; transact(request, reply);
        if (have_original) {
            request.op = Op::configure; request.config = original; transact(request, reply);
        }
        request.op = Op::test_end; transact(request, reply);
    }
    void main() override {
        while (!should_exit()) {
            DiagnosticPlan plan{};
            if (xQueueReceive(motor_.test_plans, &plan, pdMS_TO_TICKS(20)) == pdTRUE) run(plan);
        }
    }
};
#endif
class Module final : public AppModule {
public:
    std::string_view name() const override { return "smartknob"; }
private:
    Motor motor_;
    Console console_{motor_};
#if CONFIG_KNOB_DIAGNOSTICS
    DiagnosticTask diagnostic_{motor_};
#endif
    bool console_installed_ = false;
    bool on_initialize() override {
        // Never erase unrelated NVS when recovery is needed.
        if (nvs_flash_init() != ESP_OK) return false;
        load_settings(motor_.state.config, motor_.state.calibration, motor_.state.calibration_quality, motor_.state.frictionless_config, motor_.state.reference_modes);
        motor_.requests = xQueueCreate(1, sizeof(Request)); motor_.replies = xQueueCreate(1, sizeof(Reply));
        if (!motor_.requests || !motor_.replies) return false;
#if CONFIG_KNOB_DIAGNOSTICS
        motor_.test_plans = xQueueCreate(1, sizeof(DiagnosticPlan));
        motor_.test_requests = xQueueCreate(1, sizeof(Request));
        motor_.test_replies = xQueueCreate(1, sizeof(Reply));
        if (!motor_.test_plans || !motor_.test_requests || !motor_.test_replies) return false;
#endif
#if CONFIG_KNOB_CONSOLE_UART
        uart_config_t config{};
        config.baud_rate = 115200; config.data_bits = UART_DATA_8_BITS;
        config.parity = UART_PARITY_DISABLE; config.stop_bits = UART_STOP_BITS_1;
        config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE; config.source_clk = UART_SCLK_DEFAULT;
        if (uart_param_config(UART_NUM_0, &config) != ESP_OK ||
            uart_set_pin(UART_NUM_0, 43, 44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK ||
            uart_driver_install(UART_NUM_0, 2048, 4096, 0, nullptr, 0) != ESP_OK) return false;
#else
        usb_serial_jtag_driver_config_t config{.tx_buffer_size = 4096, .rx_buffer_size = 2048};
        if (usb_serial_jtag_driver_install(&config) != ESP_OK) return false;
#endif
        console_installed_ = true;
        return motor_.start() && console_.start()
#if CONFIG_KNOB_DIAGNOSTICS
            && diagnostic_.start()
#endif
            ;
    }
    bool on_deinitialize() override {
#if CONFIG_KNOB_DIAGNOSTICS
        if (!diagnostic_.stop()) return false;
#endif
        if (!console_.stop() || !motor_.stop()) return false;
        if (console_installed_) {
#if CONFIG_KNOB_CONSOLE_UART
            uart_driver_delete(UART_NUM_0);
#else
            usb_serial_jtag_driver_uninstall();
#endif
            console_installed_ = false;
        }
#if CONFIG_KNOB_DIAGNOSTICS
        if (motor_.test_plans) { vQueueDelete(motor_.test_plans); motor_.test_plans = nullptr; }
        if (motor_.test_requests) { vQueueDelete(motor_.test_requests); motor_.test_requests = nullptr; }
        if (motor_.test_replies) { vQueueDelete(motor_.test_replies); motor_.test_replies = nullptr; }
#endif
        if (motor_.requests) { vQueueDelete(motor_.requests); motor_.requests = nullptr; }
        if (motor_.replies) { vQueueDelete(motor_.replies); motor_.replies = nullptr; }
        return true;
    }
};
}
std::unique_ptr<AppModule> create_module() { return std::make_unique<Module>(); }
}
