#pragma once
#include "control.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
namespace knob {
constexpr int diagnostic_capacity = 2048;
// Explicit, diagnostics-only plateau. Never renew its deadline while active.
// The caller retains the ordinary current/speed limits and fault handling.
class DiagnosticHold {
public:
    bool start(float current, float velocity, float angle, float width, float limit, int64_t now) {
        if (active() || !std::isfinite(current) || !std::isfinite(velocity) ||
            !std::isfinite(angle) || !std::isfinite(width) || !std::isfinite(limit) ||
            now < 0 || std::abs(current) < .04f || std::abs(current) > .25f ||
            std::abs(current) > limit || std::abs(velocity) >= 2 || width <= 0) return false;
        current_ = current; origin_ = angle; excursion_ = std::min(width * .25f, .05f);
        until_ = now + 750000; return true;
    }
    bool active() const { return until_ != 0; }
    bool must_stop(int64_t now, float angle, float velocity) const {
        return active() && (now >= until_ || !std::isfinite(angle) || !std::isfinite(velocity) ||
               std::abs(angle - origin_) > excursion_ || std::abs(velocity) >= 5);
    }
    float current() const { return current_; }
    void reset() { *this = {}; }
private:
    int64_t until_ = 0;
    float current_ = 0, origin_ = 0, excursion_ = 0;
};
enum class DiagnosticAxis { q, d };
struct DiagnosticPlan {
    Mode mode = Mode::detents;
    float velocity = 50, current = .15f;
    int drive_us = 300000, observe_us = 2000000;
    DiagnosticAxis axis = DiagnosticAxis::q;
};
inline bool valid_diagnostic_plan(const DiagnosticPlan& plan, float current_limit) {
    if (static_cast<int>(plan.mode) < 0 || static_cast<int>(plan.mode) >= mode_count ||
        !std::isfinite(plan.current) || !std::isfinite(plan.velocity) ||
        !std::isfinite(current_limit)) return false;
    if (plan.axis == DiagnosticAxis::d)
        return plan.velocity == 0 && plan.current >= .05f && plan.current <= 3.0f && plan.current <= current_limit &&
            plan.drive_us >= 50000 && plan.drive_us <= 30000000 &&
            plan.observe_us >= 100000 && plan.observe_us <= 1000000;
    return plan.axis == DiagnosticAxis::q && plan.current >= .02f && plan.current <= .25f &&
        std::abs(plan.velocity) >= 1 && std::abs(plan.velocity) <= 60 &&
        plan.drive_us >= 50000 && plan.drive_us <= 3000000 &&
        plan.observe_us >= 500000 && plan.observe_us <= 10000000;
}
enum class DiagnosticDAction { drive, release, complete, motion_fault };
// These deadlines and motion bounds have no dependency on ADC validity. The
// motor task evaluates them before accepting or rejecting a current frame.
KNOB_IRAM inline DiagnosticDAction diagnostic_d_action(const DiagnosticPlan& plan,
        int64_t elapsed_us, float displacement, float velocity) {
    if (!std::isfinite(displacement) || !std::isfinite(velocity) ||
        std::abs(displacement) > .15f || std::abs(velocity) > 5.0f)
        return DiagnosticDAction::motion_fault;
    if (elapsed_us < 0 || elapsed_us >= int64_t(plan.drive_us) + plan.observe_us)
        return DiagnosticDAction::complete;
    return elapsed_us >= plan.drive_us ? DiagnosticDAction::release : DiagnosticDAction::drive;
}
KNOB_IRAM inline float diagnostic_d_target(const DiagnosticPlan& plan, int64_t elapsed_us) {
    if (elapsed_us < 0 || elapsed_us >= plan.drive_us) return 0;
    const float t = std::min(1.0f, elapsed_us / 200000.0f);
    return plan.current * t * t * (3.0f - 2.0f * t);
}
KNOB_IRAM inline float limit_diagnostic_d_current(CurrentLimiter& limiter, float requested,
        Dq measured, float phase_peak, float limit, float dt) {
    // The normal limiter reserves its d input for the orthogonal axis. During
    // positive d injection, actual Iq occupies that reserve; Id is regulated.
    return limiter.step(requested, {measured.q, measured.d}, phase_peak, limit, dt);
}
constexpr int diagnostic_capture_period_us(const DiagnosticPlan& plan) {
    const int total = plan.drive_us + plan.observe_us;
    return std::max(200, (total + diagnostic_capacity - 2) / (diagnostic_capacity - 1));
}
struct DiagnosticSample {
    uint32_t elapsed_us = 0;
    float angle = 0, velocity = 0, target = 0, iq = 0, id = 0;
    float scale = 0, voltage_q = 0, phase_peak = 0;
    uint32_t flags = 0; // stage: low byte, 8: paused, 9: speed limited, 10: quiet, 11: hold, 16/18: phase indices
    float velocity_fast = 0, voltage_d = 0;
    uint16_t adc_raw_a = 0, adc_raw_b = 0;
    int16_t sample_start_tick = 0, sample_end_tick = 0;
    float target_d = 0;
    float hold_phase = 0, hold_reference = 0, hold_correction = 0, hold_feedforward = 0;
};
struct DiagnosticSummary {
    bool busy = false, cancelled = false, recovered = false;
    DiagnosticAxis axis = DiagnosticAxis::q;
    int stage = 0; // 0 idle, 1 preparing, 2 drive, 3 release, 4 complete, 5 aborted, 6 capture, 7 frozen
    int count = 0;
    uint32_t muted_samples = 0;
    float peak_speed = 0, peak_current = 0;
};
}
