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
struct DiagnosticPlan {
    Mode mode = Mode::detents;
    float velocity = 50, current = .15f;
    int drive_us = 300000, observe_us = 2000000;
};
struct DiagnosticSample {
    uint32_t elapsed_us = 0;
    float angle = 0, velocity = 0, target = 0, iq = 0, id = 0;
    float scale = 0, voltage_q = 0, phase_peak = 0;
    uint32_t flags = 0; // stage: low byte, 8: paused, 9: speed limited, 10: quiet, 11: hold, 16/18: phase indices
    float velocity_fast = 0, voltage_d = 0;
    uint16_t adc_raw_a = 0, adc_raw_b = 0;
    int16_t sample_start_tick = 0, sample_end_tick = 0;
};
struct DiagnosticSummary {
    bool busy = false, cancelled = false, recovered = false;
    int stage = 0; // 0 idle, 1 preparing, 2 drive, 3 release, 4 complete, 5 aborted, 6 capture, 7 frozen
    int count = 0;
    uint32_t muted_samples = 0;
    float peak_speed = 0, peak_current = 0;
};
}
