#pragma once
#include "control.hpp"
#include <cstdint>

namespace knob {
// A rejected current window is recoverable. Keep PI frozen while blind, then
// request one zero vector to reopen the low-side windows without disabling the
// bridge. Only a fresh, physically valid sample ends this recovery episode.
class SamplingRecovery {
public:
    static constexpr int64_t recovery_delay_us = 2000;
    static constexpr int64_t maximum_sample_age_us = 2000;
    KNOB_IRAM void reset(int64_t now = 0) { last_good_us_ = now; recovering_ = false; }
    KNOB_IRAM bool observe(bool valid, int64_t now) {
        if (valid) { reset(now); return false; }
        if (!last_good_us_) last_good_us_ = now;
        if (!recovering_ && now - last_good_us_ > recovery_delay_us) {
            recovering_ = true;
            return true;
        }
        return false;
    }
    static KNOB_IRAM bool sample_usable(bool physical_valid, int64_t timestamp,
            int64_t previous_timestamp, int64_t now) {
        return physical_valid && timestamp > 0 && timestamp > previous_timestamp &&
            now >= timestamp && now - timestamp <= maximum_sample_age_us;
    }
    // Entering offsets only queues zero voltage. Physical validity is checked
    // again for every offset observation, before the bridge can wake or the
    // required per-phase offset counts can advance. One rejected frame must
    // not prevent starting this acquisition while the transport is fresh.
    static KNOB_IRAM bool can_start_offsets(bool synchronized, int64_t timestamp, int64_t now) {
        return synchronized && timestamp > 0 && now >= timestamp &&
            now - timestamp <= maximum_sample_age_us;
    }
    static KNOB_IRAM float integration_dt(float elapsed_s) {
        return elapsed_s > .002f ? .002f : elapsed_s;
    }
private:
    int64_t last_good_us_ = 0;
    bool recovering_ = false;
};

enum class EncoderFreshnessAction { ready, zero_voltage, waiting, lost };
// Electrical-angle prediction is bounded to 2 ms. A brief encoder delay must
// therefore mute voltage, rather than continuing with an outdated rotor angle.
// A fresh frame resumes control; a 10 ms loss remains a real sensor fault.
class EncoderFreshness {
public:
    static constexpr int64_t maximum_prediction_us = 2000;
    static constexpr int64_t loss_timeout_us = 10000;
    KNOB_IRAM void reset() { recovering_ = false; }
    KNOB_IRAM EncoderFreshnessAction observe(int64_t age_us) {
        if (age_us > loss_timeout_us) return EncoderFreshnessAction::lost;
        if (age_us <= maximum_prediction_us) {
            reset();
            return EncoderFreshnessAction::ready;
        }
        if (recovering_) return EncoderFreshnessAction::waiting;
        recovering_ = true;
        return EncoderFreshnessAction::zero_voltage;
    }
private:
    bool recovering_ = false;
};
}
