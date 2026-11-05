#pragma once
#include "control.hpp"
#include <algorithm>
#include <cstdint>

namespace knob {
struct ControlInterval {
    float elapsed_s = 0;
    float integration_s = 0;
};

// Advance only when fresh ADC feedback and the encoder allow a control update.
// Raw-frame timestamps have a separate owner: rejecting a frame must not erase
// the time elapsed since the PI last ran. Recovery resets this clock with PI.
class ControlSampleClock {
public:
    KNOB_IRAM void reset() { previous_us_ = 0; }
    KNOB_IRAM ControlInterval accept(int64_t timestamp_us, unsigned nominal_us) {
        if (timestamp_us <= 0 || timestamp_us <= previous_us_) return {};
        const int64_t elapsed_us = previous_us_ ? timestamp_us - previous_us_ : nominal_us;
        previous_us_ = timestamp_us;
        return {elapsed_us * 1e-6f, std::min<int64_t>(elapsed_us, 2000) * 1e-6f};
    }
private:
    int64_t previous_us_ = 0;
};
}
