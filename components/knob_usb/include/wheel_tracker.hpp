#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace knob::usb {
// Read-only encoder consumer. No motor configuration or haptic state is used.
// A Schmitt quantizer retains fractional travel and prevents boundary chatter.
class WheelTracker {
public:
    static constexpr int steps_per_turn = 36;
    static constexpr float tau = 6.2831853071795864769f;
    static constexpr float step_radians = tau / steps_per_turn;
    static constexpr int64_t maximum_gap_us = 20000;
    void reset() { anchored_ = false; residual_ = 0; pending_ = 0; timestamp_ = 0; }
    void update(float angle, int64_t timestamp, bool valid) {
        if (!valid || !std::isfinite(angle) || timestamp <= 0) { reset(); return; }
        if (anchored_ && timestamp == timestamp_) return;
        if (!anchored_ || timestamp < timestamp_ || timestamp - timestamp_ > maximum_gap_us) {
            reset(); anchored_ = true; previous_ = angle; timestamp_ = timestamp; return;
        }
        const float delta = std::remainder(angle - previous_, tau);
        const float max_delta = .02f + 200.0f * static_cast<float>(timestamp - timestamp_) * 1e-6f;
        previous_ = angle; timestamp_ = timestamp;
        if (std::abs(delta) > max_delta) { reset(); return; }
        residual_ += delta / step_radians;
        int ticks = 0;
        if (residual_ > .55f) ticks = static_cast<int>(std::ceil(residual_ - .55f));
        else if (residual_ < -.55f) ticks = static_cast<int>(std::floor(residual_ + .55f));
        residual_ -= static_cast<float>(ticks);
        // Bound congestion. A long host stall is separately rebased by the sender.
        pending_ = std::clamp(pending_ + ticks, -127, 127);
    }
    int8_t pending() const { return static_cast<int8_t>(pending_); }
    void sent(int8_t ticks) { pending_ -= ticks; }
private:
    bool anchored_ = false;
    float previous_ = 0, residual_ = 0;
    int pending_ = 0;
    int64_t timestamp_ = 0;
};
}
