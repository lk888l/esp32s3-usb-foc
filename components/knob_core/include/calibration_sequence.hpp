#pragma once
#include "calibration_fit.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace knob {

struct CalibrationPlan {
    // The 22008 bench motor needs more than 0.16 A to meet the full-turn
    // mechanical error limits; start gently and permit bounded adaptation.
    float current = .16f;
    float max_current = .25f;
    float voltage_limit = 3.0f;
};

inline const char* validate_calibration_plan(const CalibrationPlan& plan) {
    if (!std::isfinite(plan.current) || !std::isfinite(plan.max_current) ||
        !std::isfinite(plan.voltage_limit)) return "calibration_plan_nonfinite";
    if (plan.current < .02f || plan.current > .25f ||
        plan.max_current < plan.current || plan.max_current > .25f)
        return "calibration_current_range";
    if (plan.voltage_limit < .05f || plan.voltage_limit > 3.0f)
        return "calibration_voltage_range";
    return nullptr;
}

enum class CalibrationStage {
    idle, phase_check, align, forward, reverse, evaluate, verify, complete, failed, cancelled
};

inline const char* calibration_stage_name(CalibrationStage stage) {
    constexpr const char* names[] = {"idle", "phase_check", "align", "forward", "reverse",
        "evaluate", "verify", "complete", "failed", "cancelled"};
    const unsigned index = static_cast<unsigned>(stage);
    return index < sizeof(names) / sizeof(names[0]) ? names[index] : "invalid";
}

struct CalibrationCommand {
    float angle = 0, current = 0, voltage_d = 0;
    bool voltage_mode = false, reset_controller = false, done = false;
    const char* error = nullptr;
};

struct CalibrationSequenceInput {
    int64_t now_us = 0;
    float rotor_angle = 0;
    std::array<float, 3> currents{};
    // These are the two physically acquired phase indices for this ADC group.
    // The third current may be reconstructed, but never counts as phase-check
    // evidence. The caller must reject stale/invalid ADC groups before tick().
    std::array<int, 2> measured_phases{0, 1};
    float output_voltage = 0;
    float available_voltage_limit = 0;
};

// Runtime-only report. Pointer fields are static error strings, not a wire or
// persistence format; persistence must explicitly copy the numeric evidence.
struct CalibrationSequenceReport {
    CalibrationStage stage = CalibrationStage::idle;
    bool passed = false, cancelled = false, phase_check_passed = false;
    const char* error = nullptr;
    float current = 0, voltage_limit = 0;
    unsigned forward_points = 0, reverse_points = 0, attempts = 0, verify_points = 0;
    unsigned point_index = 0;
    float point_target = 0, verification_max = 0;
    float observed_d = 0, observed_q = 0, tracking_error = 0, saturation_fraction = 0;
    CalibrationFitResult fit_result{};
    std::array<std::array<float, 3>, 3> phase_currents{};
    std::array<std::array<unsigned, 3>, 3> phase_samples{};
};

// Bounded open-angle commissioning: current is regulated in the commanded
// stator frame, so it does not require an already trusted encoder zero.
// Higher current is attempted only after mechanical instability or fit
// quality failure; current/voltage/phase/feedback faults never trigger it.
class CalibrationSequence {
public:
    static constexpr int64_t maximum_duration_us = 120000000;
    static constexpr int64_t maximum_point_us = 2000000;
    static constexpr int64_t maximum_feedback_gap_us = 10000;
    static constexpr int64_t feedback_zero_after_us = 2000;
    static constexpr int64_t maximum_phase_age_us = 2000;
    static constexpr unsigned maximum_attempts = 3;
    static constexpr unsigned verification_points = 6;
    static constexpr float maximum_verification_error = .20f;
    static constexpr float maximum_mechanical_range = .0035f;
    static constexpr float electrical_speed = tau;
    static constexpr int64_t current_ramp_us = 200000;
    static constexpr int64_t settling_us = 150000;
    static constexpr int64_t averaging_us = 150000;
    static constexpr unsigned minimum_point_samples = 128;

    bool start(const CalibrationPlan& plan, int64_t now_us) {
        *this = CalibrationSequence{};
        plan_ = plan;
        report_.current = plan.current;
        report_.voltage_limit = plan.voltage_limit;
        started_us_ = last_valid_us_ = now_us;
        if (const char* error = validate_calibration_plan(plan)) {
            fail(error);
            return false;
        }
        if (now_us < 0) { fail("calibration_clock_invalid"); return false; }
        report_.attempts = 1;
        start_phase(0, now_us);
        return true;
    }

    const CalibrationCommand& command() const { return command_; }
    const CalibrationSequenceReport& report() const { return report_; }
    Calibration candidate() const { return report_.passed ? candidate_ : Calibration{}; }
    bool running() const {
        return report_.stage != CalibrationStage::idle && report_.stage != CalibrationStage::complete &&
            report_.stage != CalibrationStage::failed && report_.stage != CalibrationStage::cancelled;
    }

    // May be called when no usable ADC frame arrives. Neither elapsed wall
    // time nor a rejected sample contributes to an acceptance window.
    const CalibrationCommand& check_timeout(int64_t now_us) {
        if (!running()) return command_;
        if (now_us < started_us_ || now_us < last_valid_us_)
            return fail("calibration_clock_invalid");
        if (now_us - started_us_ >= maximum_duration_us)
            return fail("calibration_timeout");
        if (now_us - last_valid_us_ > maximum_feedback_gap_us)
            return fail("calibration_feedback_stale");
        if (report_.stage == CalibrationStage::phase_check) {
            for (unsigned phase = 0; phase < 3; ++phase) {
                const int64_t observed_at = phase_seen_[phase] ? phase_observed_us_[phase] : phase_ramp_started_us_;
                if (now_us - observed_at > maximum_phase_age_us)
                    return fail("calibration_phase_feedback_stale");
            }
        }
        if (now_us - point_started_us_ > maximum_point_us) {
            if (report_.stage == CalibrationStage::phase_check)
                return fail("calibration_phase_timeout");
            if (report_.stage == CalibrationStage::align || report_.stage == CalibrationStage::forward ||
                report_.stage == CalibrationStage::reverse || report_.stage == CalibrationStage::verify) {
                if (electrical_tracking_bad_) return fail("calibration_current_tracking");
                if (mechanically_unstable_ && report_.stage != CalibrationStage::verify && retry(now_us))
                    return command_;
                return fail(mechanically_unstable_ ? "calibration_alignment_unstable" : "calibration_point_timeout");
            }
        }
        return command_;
    }

    const CalibrationCommand& invalidate(int64_t now_us, bool force_zero = false) {
        check_timeout(now_us);
        if (!running()) return command_;
        clear_window();
        // A lone rejected frame keeps the existing bounded feedback hold.
        // Explicit recovery or a prolonged gap removes output and restarts
        // the current ramp on recovery, without extending the point deadline.
        if (force_zero || now_us - last_valid_us_ >= feedback_zero_after_us) {
            recovery_zero_ = true;
            zero_output(false);
        }
        return command_;
    }

    const CalibrationCommand& abort() {
        if (running()) {
            report_.stage = CalibrationStage::cancelled;
            report_.cancelled = true;
            report_.passed = false;
            report_.error = "calibration_cancelled";
            candidate_ = {};
            report_.fit_result.calibration.valid = false;
            zero_output(true);
            command_.error = report_.error;
        }
        return command_;
    }

    const CalibrationCommand& tick(const CalibrationSequenceInput& input) {
        if (!running()) return command_;
        check_timeout(input.now_us);
        if (!running()) return command_;
        if (have_sample_ && input.now_us <= last_valid_us_)
            return fail("calibration_sample_order");
        if (!std::isfinite(input.rotor_angle) || !std::isfinite(input.output_voltage) ||
            !std::isfinite(input.available_voltage_limit) || input.output_voltage < 0 ||
            input.available_voltage_limit <= 0)
            return fail("calibration_sample_invalid");
        for (float current : input.currents)
            if (!std::isfinite(current)) return fail("calibration_sample_invalid");
        if (input.measured_phases[0] < 0 || input.measured_phases[0] > 2 ||
            input.measured_phases[1] < 0 || input.measured_phases[1] > 2 ||
            input.measured_phases[0] == input.measured_phases[1])
            return fail("calibration_phase_sample_invalid");

        const Dq measured = park(input.currents, command_.angle);
        if (have_sample_ && input.now_us - last_valid_us_ > feedback_zero_after_us)
            clear_window();
        last_valid_us_ = input.now_us;
        have_sample_ = true;
        report_.voltage_limit = std::min(plan_.voltage_limit, input.available_voltage_limit);
        command_.reset_controller = false;
        if (recovery_zero_) resume_after_zero(input.now_us);

        switch (report_.stage) {
        case CalibrationStage::phase_check:
            phase_tick(input);
            break;
        case CalibrationStage::align:
            command_.angle = 0;
            set_current(input.now_us);
            if (input.now_us >= power_started_us_ + current_ramp_us + settling_us)
                qualify_point(input, measured);
            break;
        case CalibrationStage::forward:
        case CalibrationStage::reverse:
        case CalibrationStage::verify: {
            const float progress = std::clamp(static_cast<float>(input.now_us - motion_started_us_) /
                static_cast<float>(motion_duration_us_), 0.0f, 1.0f);
            command_.angle = motion_from_ + (report_.point_target - motion_from_) * smooth(progress);
            set_current(input.now_us);
            if (input.now_us >= motion_started_us_ + motion_duration_us_ + settling_us &&
                input.now_us >= power_started_us_ + current_ramp_us + settling_us)
                qualify_point(input, measured);
            break;
        }
        case CalibrationStage::evaluate:
            if (input.now_us - point_started_us_ >= 20000) evaluate_fit(input.now_us);
            break;
        default:
            break;
        }
        return command_;
    }

private:
    struct Window {
        unsigned count = 0, saturated = 0;
        int64_t started_us = 0;
        float angle_origin = 0, angle_sum = 0, minimum = 0, maximum = 0;
        float d_sum = 0, q_sum = 0;
        void add(const CalibrationSequenceInput& input, Dq measured, float voltage_limit) {
            if (!count) {
                started_us = input.now_us;
                angle_origin = minimum = maximum = input.rotor_angle;
            }
            minimum = std::min(minimum, input.rotor_angle);
            maximum = std::max(maximum, input.rotor_angle);
            angle_sum += input.rotor_angle - angle_origin;
            d_sum += measured.d;
            q_sum += measured.q;
            ++count;
            if (input.output_voltage >= voltage_limit * .98f) ++saturated;
        }
        float angle() const { return angle_origin + angle_sum / static_cast<float>(count); }
    };

    static float smooth(float x) { return x * x * (3.0f - 2.0f * x); }
    static int64_t motion_time(float from, float to) {
        // electrical_speed is the mean travel speed. Smoothstep removes the
        // velocity step at each end; its peak is 1.5 times this nominal speed.
        return std::max<int64_t>(100000, static_cast<int64_t>(
            std::abs(to - from) / electrical_speed * 1000000.0f));
    }
    void set_current(int64_t now_us) {
        command_.current = report_.current * smooth(std::clamp(
            static_cast<float>(now_us - power_started_us_) / current_ramp_us, 0.0f, 1.0f));
        command_.voltage_mode = false;
        command_.voltage_d = 0;
    }
    void clear_window() {
        window_ = {};
        phase_sums_ = {};
        phase_counts_ = {};
        phase_window_started_us_ = 0;
    }
    void zero_output(bool done) {
        command_.current = command_.voltage_d = 0;
        command_.voltage_mode = true;
        command_.reset_controller = true;
        command_.done = done;
    }
    const CalibrationCommand& fail(const char* reason) {
        report_.stage = CalibrationStage::failed;
        report_.error = reason;
        report_.passed = false;
        candidate_ = {};
        report_.fit_result.calibration.valid = false;
        zero_output(true);
        command_.error = reason;
        return command_;
    }
    void start_phase(unsigned index, int64_t now_us) {
        report_.stage = CalibrationStage::phase_check;
        report_.point_index = index;
        report_.point_target = index * tau / 3;
        point_started_us_ = phase_ramp_started_us_ = now_us;
        phase_filtered_ = {};
        phase_seen_ = {};
        phase_observed_us_.fill(now_us);
        phase_probe_voltage_ = 0;
        phase_probe_capped_ = false;
        clear_window();
        command_ = {};
        command_.angle = report_.point_target;
        command_.voltage_mode = true;
        command_.reset_controller = true;
    }
    void start_alignment(int64_t now_us, bool before_verify) {
        report_.stage = CalibrationStage::align;
        report_.point_index = 0;
        report_.point_target = 0;
        point_started_us_ = power_started_us_ = now_us;
        align_before_verify_ = before_verify;
        mechanically_unstable_ = false;
        electrical_tracking_bad_ = false;
        clear_window();
        command_ = {};
        command_.reset_controller = true;
    }
    void start_point(CalibrationStage stage, unsigned index, int64_t now_us) {
        report_.stage = stage;
        report_.point_index = index;
        report_.point_target = stage == CalibrationStage::verify ?
            (.5f + 5.0f * index) * (tau / 4) : CalibrationFit::command_angle(index);
        point_started_us_ = motion_started_us_ = now_us;
        motion_from_ = command_.angle;
        motion_duration_us_ = motion_time(motion_from_, report_.point_target);
        mechanically_unstable_ = false;
        electrical_tracking_bad_ = false;
        clear_window();
    }
    void resume_after_zero(int64_t now_us) {
        recovery_zero_ = false;
        power_started_us_ = now_us;
        command_.reset_controller = true;
        clear_window();
        if (report_.stage == CalibrationStage::phase_check) {
            phase_ramp_started_us_ = now_us;
            phase_filtered_ = {};
            phase_seen_ = {};
            phase_observed_us_.fill(now_us);
            phase_probe_voltage_ = 0;
            phase_probe_capped_ = false;
        } else if (report_.stage == CalibrationStage::forward || report_.stage == CalibrationStage::reverse ||
                 report_.stage == CalibrationStage::verify) {
            motion_from_ = command_.angle;
            motion_started_us_ = now_us;
            motion_duration_us_ = motion_time(motion_from_, report_.point_target);
        }
        // point_started_us_ intentionally stays unchanged.
    }

    void phase_tick(const CalibrationSequenceInput& input) {
        command_.angle = report_.point_target;
        command_.current = 0;
        command_.voltage_mode = true;
        // Phase correspondence is still unverified, so use the largest
        // magnitude among the independently observed channels, independent
        // of assumed polarity, to bound this open-voltage probe. It cannot
        // establish absolute current accuracy without a trusted CSA gain.
        for (int phase : input.measured_phases) {
            const float dt = std::max<int64_t>(0, input.now_us - phase_observed_us_[phase]) * 1e-6f;
            phase_observed_us_[phase] = input.now_us;
            phase_seen_[phase] = true;
            phase_filtered_[phase] += dt / (.005f + dt) * (input.currents[phase] - phase_filtered_[phase]);
        }
        // The two smaller currents cannot establish a bound on an unobserved
        // primary current. Require real observations of every phase before
        // energizing, and keep each phase fresh throughout the voltage probe.
        // Reconstructed currents never satisfy this gate.
        const bool all_seen = phase_seen_[0] && phase_seen_[1] && phase_seen_[2];
        if (!all_seen) {
            phase_probe_voltage_ = command_.voltage_d = 0;
            return;
        }
        float filtered_peak = 0;
        for (float current : phase_filtered_) filtered_peak = std::max(filtered_peak, std::abs(current));
        if (filtered_peak > plan_.max_current) { fail("calibration_phase_current_high"); return; }
        if (filtered_peak >= .65f * plan_.current) phase_probe_capped_ = true;
        const float ramp_voltage = std::min(.5f, report_.voltage_limit) * smooth(std::clamp(
            static_cast<float>(input.now_us - phase_ramp_started_us_) / current_ramp_us, 0.0f, 1.0f));
        if (!phase_probe_capped_) phase_probe_voltage_ = ramp_voltage;
        phase_probe_voltage_ = std::min(phase_probe_voltage_, report_.voltage_limit);
        command_.voltage_d = phase_probe_voltage_;
        if (input.now_us < phase_ramp_started_us_ + current_ramp_us + settling_us) return;
        if (!phase_window_started_us_) phase_window_started_us_ = input.now_us;
        for (int phase : input.measured_phases) {
            phase_sums_[phase] += input.currents[phase];
            ++phase_counts_[phase];
        }
        if (input.now_us - phase_window_started_us_ < 200000 ||
            *std::min_element(phase_counts_.begin(), phase_counts_.end()) < 256) return;
        const unsigned vector = report_.point_index;
        for (unsigned phase = 0; phase < 3; ++phase) {
            report_.phase_currents[vector][phase] = phase_sums_[phase] / phase_counts_[phase];
            report_.phase_samples[vector][phase] = phase_counts_[phase];
        }
        const auto& means = report_.phase_currents[vector];
        const float primary = means[vector];
        if (primary < .02f) { fail("calibration_phase_excitation"); return; }
        if (primary > plan_.max_current) {
            fail("calibration_phase_current_high"); return;
        }
        for (unsigned phase = 0; phase < 3; ++phase) {
            if (phase == vector) continue;
            if (means[phase] >= -.003f) { fail("calibration_phase_polarity"); return; }
            if (std::abs(means[phase] + .5f * primary) > std::max(.008f, .25f * primary)) {
                fail("calibration_phase_ratio"); return;
            }
        }
        if (vector < 2) start_phase(vector + 1, input.now_us);
        else {
            report_.phase_check_passed = true;
            start_alignment(input.now_us, false);
        }
    }

    void qualify_point(const CalibrationSequenceInput& input, Dq measured) {
        window_.add(input, measured, report_.voltage_limit);
        if (window_.count < minimum_point_samples || input.now_us - window_.started_us < averaging_us) return;
        const float tolerance = std::max(.008f, .15f * report_.current);
        const float d = window_.d_sum / window_.count;
        const float q = window_.q_sum / window_.count;
        report_.observed_d = d;
        report_.observed_q = q;
        report_.tracking_error = std::max(std::abs(d - report_.current), std::abs(q));
        report_.saturation_fraction = static_cast<float>(window_.saturated) / window_.count;
        if (std::abs(d - report_.current) > tolerance || std::abs(q) > tolerance) {
            if (window_.saturated * 2 >= window_.count && d < report_.current - tolerance) {
                fail("calibration_voltage_saturated");
                return;
            }
            // The existing PI gains may need another complete window after
            // an angle/current ramp. Wait within the ORIGINAL point deadline;
            // an electrical mismatch never justifies increasing excitation.
            electrical_tracking_bad_ = true;
            window_ = {};
            return;
        }
        electrical_tracking_bad_ = false;
        if (window_.maximum - window_.minimum > maximum_mechanical_range) {
            mechanically_unstable_ = true;
            window_ = {};
            return;
        }
        const float angle = window_.angle();
        if (report_.stage == CalibrationStage::align) {
            start_point(align_before_verify_ ? CalibrationStage::verify : CalibrationStage::forward, 0, input.now_us);
            return;
        }
        if (report_.stage == CalibrationStage::verify) {
            const float error = std::abs(CalibrationFit::verify_point(candidate_, report_.point_target, angle));
            if (!std::isfinite(error)) { fail("calibration_verification_invalid"); return; }
            report_.verification_max = std::max(report_.verification_max, error);
            if (error > maximum_verification_error) { fail("calibration_verification"); return; }
            ++report_.verify_points;
            if (report_.verify_points == verification_points) {
                report_.stage = CalibrationStage::complete;
                report_.passed = true;
                report_.fit_result.calibration = candidate_;
                zero_output(true);
            } else start_point(CalibrationStage::verify, report_.verify_points, input.now_us);
            return;
        }
        const bool reverse = report_.stage == CalibrationStage::reverse;
        if (!fit_.add(reverse, report_.point_index, angle)) {
            fail(fit_.collection_error() ? fit_.collection_error() : "calibration_point_invalid"); return;
        }
        report_.forward_points = fit_.forward_count();
        report_.reverse_points = fit_.reverse_count();
        if (!reverse) {
            if (report_.point_index + 1 < CalibrationFit::point_count)
                start_point(CalibrationStage::forward, report_.point_index + 1, input.now_us);
            else start_point(CalibrationStage::reverse, report_.point_index, input.now_us);
        } else if (report_.point_index)
            start_point(CalibrationStage::reverse, report_.point_index - 1, input.now_us);
        else {
            report_.stage = CalibrationStage::evaluate;
            point_started_us_ = input.now_us;
            zero_output(false);
        }
    }

    bool retry(int64_t now_us) {
        if (report_.attempts >= maximum_attempts || report_.current >= plan_.max_current - .00001f)
            return false;
        report_.current = std::min(plan_.max_current, report_.current * 1.5f);
        ++report_.attempts;
        report_.forward_points = report_.reverse_points = report_.verify_points = 0;
        report_.verification_max = 0;
        fit_.reset();
        candidate_ = {};
        start_alignment(now_us, false);
        return true;
    }
    void evaluate_fit(int64_t now_us) {
        report_.fit_result = fit_.evaluate();
        if (!report_.fit_result.valid()) {
            const char* error = report_.fit_result.error ? report_.fit_result.error : "calibration_fit_invalid";
            const bool mechanical = !std::strcmp(error, "calibration_hysteresis") ||
                !std::strcmp(error, "calibration_direction_bias") || !std::strcmp(error, "calibration_residual");
            if (mechanical && retry(now_us)) return;
            fail(error);
            return;
        }
        candidate_ = report_.fit_result.calibration;
        // The fitted candidate is not an accepted calibration until the new
        // held-out points pass. Report numeric fit evidence meanwhile.
        report_.fit_result.calibration.valid = false;
        start_alignment(now_us, true);
    }

    CalibrationPlan plan_{};
    CalibrationCommand command_{};
    CalibrationSequenceReport report_{};
    CalibrationFit fit_{};
    Calibration candidate_{};
    Window window_{};
    std::array<float, 3> phase_sums_{};
    std::array<unsigned, 3> phase_counts_{};
    std::array<float, 3> phase_filtered_{};
    std::array<bool, 3> phase_seen_{};
    std::array<int64_t, 3> phase_observed_us_{};
    int64_t started_us_ = 0, last_valid_us_ = 0, point_started_us_ = 0;
    int64_t phase_ramp_started_us_ = 0, phase_window_started_us_ = 0, power_started_us_ = 0;
    int64_t motion_started_us_ = 0, motion_duration_us_ = 100000;
    float motion_from_ = 0;
    float phase_probe_voltage_ = 0;
    bool have_sample_ = false, recovery_zero_ = false, align_before_verify_ = false;
    bool mechanically_unstable_ = false;
    bool electrical_tracking_bad_ = false;
    bool phase_probe_capped_ = false;
};

} // namespace knob
