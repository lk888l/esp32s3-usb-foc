#include "knob_app.hpp"
#include "app_task.hpp"
#include "board.hpp"
#include "protocol.hpp"
#include "driver/usb_serial_jtag.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace knob {
namespace {
// Compile-time transport adapter: exactly one host owns the motor lease.
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
        const int count = usb_serial_jtag_write_bytes(bytes + sent, size - sent, pdMS_TO_TICKS(10));
        if (count < 0) return false;
        sent += count;
    }
    return sent == size;
#endif
}
class Motor final : public AppTask {
public:
    Motor() : AppTask("knob_control", 6144, 20, 1) {}
    QueueHandle_t requests = nullptr, replies = nullptr;
#if CONFIG_KNOB_DIAGNOSTICS
    QueueHandle_t test_plans = nullptr, test_requests = nullptr, test_replies = nullptr;
#endif
    Snapshot state{};
private:
    Board board_;
    AngleTracker angle_;
    Haptics haptic_;
    HapticSpeedLimiter haptic_speed_;
    CurrentLoop current_loop_;
    CurrentLimiter current_limiter_;
    CurrentTripFilter current_trip_filter_;
    bool ready_ = false, calibrating_ = false, wake_ = false;
    int64_t heartbeat_ = 0, stage_start_ = 0, last_sample_ = 0, pulse_start_ = 0;
    int64_t trip_update_at_ = 0;
    std::array<float, 3> sums_{}, squares_{};
    std::array<int, 3> counts_{};
    float sweep_start_ = 0, offset_start_ = 0;
    Dq alignment_current_{};
    SettlingMonitor settling_;
    unsigned invalid_ = 0;
#if CONFIG_KNOB_DIAGNOSTICS
    DiagnosticPlan test_plan_{};
    std::array<DiagnosticSample, diagnostic_capacity> test_samples_{};
    int64_t test_started_ = 0, test_sampled_ = 0;
#endif
    bool energized() const { return state.state != State::idle && state.state != State::fault; }
    void capture_overcurrent() {
        const auto raw = board_.trip_sample();
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
    }
    void halt(const char* fault = nullptr) {
#if CONFIG_KNOB_DIAGNOSTICS
        if (state.diagnostic.busy && state.diagnostic.stage < 4) {
            state.diagnostic.stage = 5; state.diagnostic.cancelled = true;
        }
#endif
        board_.enable(false); board_.write_duty({0, 0, 0});
        state.state = fault ? State::fault : State::idle; state.fault = fault ? fault : "none";
        state.target = 0; state.duty = {}; state.current_valid = false;
        state.voltage_d = 0; state.voltage_q = 0;
        state.current_limited = false; state.current_target_limit = 0;
        haptic_speed_.reset(); state.haptic_scale = 0;
        state.speed_limited = false; state.haptic_paused = false;
        current_limiter_.reset(state.config.current_limit); trip_update_at_ = 0;
        current_trip_filter_.reset();
        if (!fault) state.current_trip_limit = overcurrent_threshold(state.config.current_limit);
        current_loop_.reset(); pulse_start_ = 0;
    }
    void fail(const char* fault) {
        if (state.state == State::fault) return; // Preserve the first cause until clear.
        if (calibrating_) state.calibration.valid = false;
        halt(fault);
    }
    bool setup_board() {
        if (board_.initialize() != ESP_OK || board_.start_sampling(xTaskGetCurrentTaskHandle()) != ESP_OK) {
            state.hardware_detail = board_.failed_operation();
            state.hardware_error = board_.initialization_error();
            board_.shutdown(); return false;
        }
        state.hardware_detail = "none"; state.hardware_error = 0;
        return true;
    }
    const char* command(const Request& r, int64_t now) {
#if CONFIG_KNOB_DIAGNOSTICS
        if (state.diagnostic.busy && !r.test_command && r.op != Op::status && r.op != Op::disable)
            return "diagnostic_busy";
        if (r.test_command && state.diagnostic.cancelled &&
            r.op != Op::status && r.op != Op::disable && r.op != Op::configure && r.op != Op::test_end)
            return "diagnostic_cancelled";
#endif
        switch (r.op) {
#if CONFIG_KNOB_DIAGNOSTICS
        case Op::diagnose:
            if (state.state != State::idle) return "idle_required";
            if (!valid_calibration(state.calibration)) return "calibrate_first";
            if (xQueueSend(test_plans, &r.diagnostic, 0) != pdTRUE) return "diagnostic_busy";
            state.diagnostic = {}; state.diagnostic.busy = true; state.diagnostic.stage = 1;
            test_started_ = 0; return nullptr;
        case Op::diagnostic_read:
            return r.diagnostic_offset <= state.diagnostic.count ? nullptr : "diagnostic_offset";
        case Op::test_begin:
            if (!r.test_command || !state.diagnostic.busy || state.state != State::active) return "diagnostic_not_ready";
            test_plan_ = r.diagnostic; test_started_ = now; test_sampled_ = 0;
            state.diagnostic.stage = 2; return nullptr;
        case Op::test_end:
            if (!r.test_command || energized()) return "disable_first";
            state.diagnostic.busy = false; return nullptr;
#endif
        case Op::status: return nullptr;
        case Op::disable:
            halt(state.state == State::fault ? state.fault : nullptr); return nullptr;
        case Op::save: return energized() ? "disable_before_save" : nullptr;
        case Op::clear:
            if (energized()) return "disable_first";
            if (!ready_) ready_ = setup_board();
            if (!ready_) return "hardware_unavailable";
            board_.reset_trip(); invalid_ = 0; angle_ = {};
            state.encoder_error = 0; state.fault_magnet = 0;
            state.fault_currents = {}; state.fault_current_valid = false;
            state.fault_current_trip_limit = 0;
            state.fault_sample_interval_us = 0;
            halt(); return nullptr;
        case Op::configure: {
            const auto& c = r.config;
            if (const char* e = validate(c)) return e;
            const bool electrical = c.bus_voltage != state.config.bus_voltage || c.sense_gain != state.config.sense_gain ||
                c.current_kp != state.config.current_kp || c.current_ki != state.config.current_ki || c.voltage_limit != state.config.voltage_limit;
            if (energized() && (electrical || state.state != State::active)) return "disable_before_electrical_config";
            const bool current_changed = c.current_limit != state.config.current_limit;
            if (current_changed) current_trip_filter_.reset();
            if (c.sense_gain != state.config.sense_gain || c.bus_voltage != state.config.bus_voltage) state.calibration.valid = false;
            haptic_.reconfigure(state.config, c, angle_.angle());
            state.config = c;
            current_limiter_.configure(c.current_limit);
            state.target = std::clamp(state.target, -c.current_limit, c.current_limit);
            state.current_target_limit = std::min(state.current_target_limit, c.current_limit);
            const float trip = overcurrent_threshold(c.current_limit);
            if (state.state == State::active && trip < state.current_trip_limit) {
                // The new torque limit is immediate. Allow winding current to
                // decay before tightening the emergency cutoff below its old value.
                if (current_changed) trip_update_at_ = now + 20000;
            } else {
                state.current_trip_limit = trip; trip_update_at_ = 0;
                if (state.state == State::active) board_.set_trip(state.offsets, c.sense_gain, trip);
            }
            state.position = haptic_.position(); state.sub_position = haptic_.sub_position(c, angle_.angle());
            if (electrical) current_loop_.reset();
            pulse_start_ = 0; return nullptr;
        }
        case Op::zero:
            if (state.state != State::idle && state.state != State::active) return "busy";
            haptic_.reset(angle_.angle(), std::clamp(0, state.config.min_position, state.config.max_position));
            state.position = haptic_.position(); state.sub_position = 0; return nullptr;
        case Op::pulse:
            if (state.state != State::active) return "enable_first";
            pulse_start_ = now; return nullptr;
        case Op::calibrate:
        case Op::enable:
            if (!ready_) return "hardware_unavailable";
            if (state.state != State::idle) return "idle_required";
            if (board_.trip()) return "clear_fault_first";
            if (r.op == Op::enable && !valid_calibration(state.calibration)) return "calibrate_first";
            if (!board_.sample().valid || now - board_.sample().timestamp_us > 2000) return "adc_window_invalid";
            calibrating_ = r.op == Op::calibrate;
            if (calibrating_) state.calibration.valid = false;
            stage_start_ = now; wake_ = false; sums_ = {}; squares_ = {}; counts_ = {};
            invalid_ = 0; current_loop_.reset(); alignment_current_ = {}; offset_start_ = angle_.angle();
            current_limiter_.reset(state.config.current_limit);
            board_.write_duty({0,0,0}); state.state = State::offsets; return nullptr;
        }
        return "unknown_op";
    }
    void control(const RawSample& raw, float dt, int64_t now) {
        const auto& c = state.config;
        state.rejected = raw.rejected; state.sample_us = raw.duration_ticks / 80.0f;
        if (state.state == State::offsets) {
            // Allow zero duty to commit before waking the bridge; wait for CSA startup.
            if (now - stage_start_ > 10000 && !wake_) { board_.enable(true); wake_ = true; }
            if (now - stage_start_ < 30000) return;
            if (std::abs(angle_.angle() - offset_start_) > 0.03f) { fail("keep_rotor_still_for_offsets"); return; }
            for (int k = 0; k < 2; ++k) {
                const int p = raw.phase[k]; const float v = board_.volts(p, raw.raw[k]);
                sums_[p] += v; squares_[p] += v * v; ++counts_[p]; state.volts[p] = v;
            }
            if (*std::min_element(counts_.begin(), counts_.end()) < 128) return;
            for (int p = 0; p < 3; ++p) {
                const float zero = sums_[p] / counts_[p];
                if (zero < 0.7f || zero > 2.2f || squares_[p] / counts_[p] - zero * zero > 0.000625f) {
                    fail("invalid_current_offsets_check_mode_gain"); return;
                }
                state.offsets[p] = zero;
            }
            state.current_trip_limit = overcurrent_threshold(c.current_limit);
            board_.set_trip(state.offsets, c.sense_gain, state.current_trip_limit);
            if (calibrating_) { state.state = State::align; stage_start_ = now; settling_.reset(); }
            else { state.state = State::active; haptic_.reset(angle_.angle(), std::clamp(0,c.min_position,c.max_position)); }
            return;
        }
        if (!energized()) return;
#if CONFIG_KNOB_DIAGNOSTICS
        if (test_started_ && state.diagnostic.busy && state.diagnostic.stage < 4) {
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
            if (v < 0.25f || v > 3.05f) { fail("adc_saturation"); return; }
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
        if (current_trip_filter_.step(phase_peak >= state.current_trip_limit, phase_peak >= absolute_current_trip)) {
            state.fault_currents = state.currents; state.fault_current_valid = true;
            state.fault_current_trip_limit = state.current_trip_limit;
            fail("overcurrent"); return;
        }
        float electrical = wrap(state.calibration.direction * state.calibration.pole_pairs * wrap(angle_.angle()) - state.calibration.electrical_zero);
        Dq target{};
        if (state.state == State::active) {
            state.target = haptic_.step(c, angle_.angle(), angle_.haptic_velocity());
            if (pulse_start_) {
                const int64_t elapsed = now - pulse_start_;
                if (elapsed < 6000) state.target += elapsed < 3000 ? 0.08f : -0.08f;
                else pulse_start_ = 0;
            }
            state.target = haptic_speed_.step(state.target, angle_.velocity(), c.current_limit, dt);
            state.haptic_scale = haptic_speed_.scale();
            state.speed_limited = haptic_speed_.limited(); state.haptic_paused = haptic_speed_.paused();
#if CONFIG_KNOB_DIAGNOSTICS
            if (state.diagnostic.busy && state.diagnostic.stage == 2) {
                // Bounded external-motion stimulus. Haptics still track angle
                // and speed, but the test current emulates a hand-driven spin.
                const float ramp = std::min(1.0f, (now - test_started_) / 50000.0f);
                state.target = std::clamp(.015f * (test_plan_.velocity * ramp - angle_.velocity()),
                                          -test_plan_.current, test_plan_.current);
            }
#endif
            // Haptic torque uses sensor coordinates; electromagnetic torque uses electrical coordinates.
            target.q = state.target * state.calibration.direction;
        } else {
            const float elapsed = (now - stage_start_) * 0.000001f;
            target.d = std::min(0.12f, c.current_limit) * (state.state == State::align ? std::min(1.0f, elapsed / 0.2f) : 1.0f);
            electrical = 0;
            settling_.update(angle_.angle(), dt);
            if (state.state == State::align && elapsed > 0.8f) {
                if (!alignment_tracking(target.d)) { fail("calibration_current_tracking"); return; }
                if (settling_.still()) { state.state = State::sweep; stage_start_ = now; sweep_start_ = angle_.angle(); }
                else if (elapsed > 2) { fail("alignment_unstable"); return; }
            } else if (state.state == State::sweep) {
                electrical = tau * std::min(1.0f, elapsed / 3.0f);
                if (elapsed >= 3) { state.state = State::settle; stage_start_ = now; settling_.reset(); }
            } else if (state.state == State::settle && elapsed > 0.5f) {
                if (!alignment_tracking(target.d)) { fail("calibration_current_tracking"); return; }
                const float delta = angle_.angle() - sweep_start_;
                if (std::abs(std::abs(delta) - tau / 7) > 0.18f) {
                    fail("calibration_motion_mismatch_7pp"); return;
                }
                if (settling_.still()) {
                    state.calibration.direction = delta > 0 ? 1 : -1;
                    state.calibration.electrical_zero = wrap(state.calibration.direction * 7 * wrap(angle_.angle()));
                    state.calibration.valid = true; calibrating_ = false; halt(); return;
                }
                if (elapsed > 1.5f) { fail("alignment_unstable"); return; }
            }
        }
        const Dq measured = park(state.currents, electrical);
        if (state.state == State::active) {
            target.q = current_limiter_.step(target.q, measured, phase_peak, c.current_limit, dt);
            state.target = target.q * state.calibration.direction;
            state.current_target_limit = current_limiter_.capacity();
            state.current_limited = current_limiter_.limited();
        }
        state.id = measured.d; state.iq = measured.q;
        const float filter = dt / (0.02f + dt);
        alignment_current_.d += filter * (measured.d - alignment_current_.d);
        alignment_current_.q += filter * (measured.q - alignment_current_.q);
        state.current_valid = true;
        const float limit = state.state == State::active ? c.voltage_limit : std::min(c.voltage_limit, 0.8f);
        const Dq voltage = current_loop_.step({target.d - measured.d, target.q - measured.q}, dt, c.current_kp, c.current_ki, limit);
        state.voltage_d = voltage.d; state.voltage_q = voltage.q;
        state.duty = svpwm(voltage, electrical, c.bus_voltage);
        if (board_.write_duty(state.duty) != ESP_OK) fail("pwm_failure");
#if CONFIG_KNOB_DIAGNOSTICS
        auto& diagnostic = state.diagnostic;
        if (diagnostic.busy && diagnostic.stage >= 2 && diagnostic.stage <= 3) {
            diagnostic.peak_speed = std::max(diagnostic.peak_speed, std::abs(angle_.velocity()));
            diagnostic.peak_current = std::max(diagnostic.peak_current, phase_peak);
            if (diagnostic.stage == 3 && state.haptic_paused && state.haptic_scale <= 1e-6f &&
                std::abs(state.target) <= 1e-5f) ++diagnostic.muted_samples;
            if (diagnostic.stage == 3 && diagnostic.muted_samples && !state.speed_limited)
                diagnostic.recovered = true;
            if ((!test_sampled_ || now - test_sampled_ >= 2000) && diagnostic.count < diagnostic_capacity) {
                test_sampled_ = now;
                test_samples_[diagnostic.count++] = {
                    static_cast<uint32_t>(now - test_started_), angle_.angle(), angle_.velocity(), state.target,
                    state.iq, state.id, state.haptic_scale, state.voltage_q, phase_peak,
                    static_cast<uint32_t>(diagnostic.stage | (state.haptic_paused ? 256 : 0) |
                                          (state.speed_limited ? 512 : 0)), angle_.haptic_velocity()};
            }
        }
#endif
    }
    bool alignment_tracking(float target) const {
        const float tolerance = std::max(0.015f, target * 0.35f);
        return std::abs(alignment_current_.d - target) < tolerance && std::abs(alignment_current_.q) < tolerance;
    }
    void main() override {
        state.current_trip_limit = overcurrent_threshold(state.config.current_limit);
        heartbeat_ = Board::esp_time();
        ready_ = setup_board();
        if (!ready_) fail("hardware_init_failed");
        while (!should_exit()) {
            if (ready_) ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5)); else vTaskDelay(pdMS_TO_TICKS(5));
            const int64_t now = Board::esp_time();
            if (energized() && board_.trip()) {
                if (board_.trip() == 1) capture_overcurrent();
                fail(board_.trip() == 1 ? "overcurrent_isr" : "control_watchdog");
            }
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
                // Well-formed but refused commands cannot renew torque either.
                if (!error && request.renew_lease) heartbeat_ = now;
                Reply reply{error, state, request.id};
#if CONFIG_KNOB_DIAGNOSTICS
                if (!error && request.op == Op::diagnostic_read) {
                    reply.diagnostic_offset = request.diagnostic_offset;
                    reply.diagnostic_count = std::min(4, state.diagnostic.count - request.diagnostic_offset);
                    for (int i = 0; i < reply.diagnostic_count; ++i)
                        reply.diagnostic_samples[i] = test_samples_[request.diagnostic_offset + i];
                }
#endif
                xQueueOverwrite(reply_queue, &reply);
            }
            if (!ready_) continue;
            if (energized() && now - heartbeat_ > 1000000) { fail("host_timeout"); continue; }
            const auto raw = board_.sample();
            // A notification can arrive before sample() reads its newer data.
            // The next wake then refers to a sample already consumed here.
            // Do not run the encoder/control twice or count that wake as a bad
            // ADC conversion. Real missing samples remain covered by the ISR
            // watchdog and the 2 ms interval/age checks below.
            if (raw.timestamp_us <= last_sample_) { ++state.reused_samples; continue; }
            state.current_valid = false;
            state.rejected = raw.rejected; state.sample_us = raw.duration_ticks / 80.0f;
            state.sample_start_tick = raw.start_tick; state.sample_end_tick = raw.end_tick;
            float dt = last_sample_ ? (raw.timestamp_us - last_sample_) * 0.000001f : 0.0005f;
            const bool valid_sample = raw.valid && raw.timestamp_us > last_sample_ && now - raw.timestamp_us < 2000 && dt <= 0.002f;
            if (raw.timestamp_us > last_sample_) last_sample_ = raw.timestamp_us;
            float wrapped = 0; uint8_t magnet = 0;
            const int64_t encoder_started = Board::esp_time();
            const esp_err_t encoder_error = board_.read_angle(wrapped, magnet);
            state.encoder_us = Board::esp_time() - encoder_started;
            state.encoder_retries = board_.encoder_retries();
            if (encoder_error != ESP_OK) {
                state.magnet = magnet;
                ++state.encoder_failures;
                if (state.state != State::fault) {
                    state.encoder_error = encoder_error; state.fault_magnet = magnet;
                }
                // Expose angle diagnostics even when magnet quality is rejected.
                if (!board_.encoder_transport_error() && angle_.update(wrapped, std::clamp(dt, 0.0001f, 0.005f))) {
                    state.angle = angle_.angle(); state.velocity = angle_.velocity(); state.velocity_fast = angle_.haptic_velocity();
                }
                state.loop_us = Board::esp_time() - now;
                fail(board_.encoder_transport_error() ? "encoder_i2c" :
                     !(magnet & 0x20) ? "encoder_missing_magnet" : "encoder_magnet_strength");
                continue;
            }
            state.magnet = magnet;
            if (!angle_.update(wrapped, std::clamp(dt, 0.0001f, 0.005f))) { fail("encoder_jump"); continue; }
            state.angle = angle_.angle(); state.velocity = angle_.velocity(); state.velocity_fast = angle_.haptic_velocity();
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
                current_trip_filter_.reset();
                if (++invalid_ >= 3 && energized()) {
                    state.fault_sample_interval_us = std::lround(dt * 1000000);
                    state.fault_sample_age_us = now - raw.timestamp_us;
                    state.fault_sample_start_tick = raw.start_tick;
                    state.fault_sample_end_tick = raw.end_tick;
                    state.fault_sample_us = raw.duration_ticks / 80.0f;
                    state.fault_sample_valid = raw.valid;
                    fail("adc_window_or_control_deadline");
                }
                continue;
            }
            invalid_ = 0;
            const int64_t control_started = Board::esp_time();
            board_.kick(); control(raw, dt, now);
            state.control_us = Board::esp_time() - control_started;
            state.position = haptic_.position(); state.sub_position = haptic_.sub_position(state.config, angle_.angle());
            state.loop_us = Board::esp_time() - now;
        }
        board_.shutdown();
    }
};
class Console final : public AppTask {
public:
    explicit Console(Motor& motor) : AppTask("knob_console", 8192, 5, 0), motor_(motor) {}
private:
    Motor& motor_;
    void process_line(const char* line) {
        Request request{};
        const char* decode_error = decode_request(line, request);
        // A rejected frame still reports real state, without renewing the lease.
        if (decode_error) { request.op = Op::status; request.renew_lease = false; }
        Reply reply{};
        {
            const int64_t deadline = Board::esp_time() + 200000;
            bool matched = false;
            if (xQueueSend(motor_.requests, &request, pdMS_TO_TICKS(20)) == pdTRUE) {
                while (Board::esp_time() < deadline) {
                    if (xQueueReceive(motor_.replies, &reply, pdMS_TO_TICKS(10)) == pdTRUE && reply.request_id == request.id) {
                        matched = true; break;
                    }
                }
            }
            if (!matched) reply.error = "control_unavailable";
            else if (decode_error) reply.error = decode_error;
            else if (!reply.error && request.op == Op::save && !save_settings(reply.snapshot.config, reply.snapshot.calibration)) reply.error = "save_failed";
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
    explicit DiagnosticTask(Motor& motor) : AppTask("knob_test", 8192, 4, 0), motor_(motor) {}
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
            const int64_t deadline = Board::esp_time() + 4500000;
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
        load_settings(motor_.state.config, motor_.state.calibration);
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
