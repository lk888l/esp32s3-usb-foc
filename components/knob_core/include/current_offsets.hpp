#pragma once
#include "control.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace knob {

// Engineering acceptance limits in physical current units. The gain must
// already match the hardware; zero-current collection cannot identify gain.
struct CurrentOffsetLimits {
    float minimum_volts = 0.7f;
    float maximum_volts = 2.2f;
    float maximum_noise_a = 0.12f;
    float maximum_standard_error_a = 0.006f;
    float maximum_drift_a = 0.02f;
    float window_confidence_sigma = 3.0f;
};

// The caller supplies only fresh, physically valid ADC samples, with the
// bridge at zero output and the rotor stationary. Timeouts/motion/transport
// faults belong to the caller. Every phase must independently fill all four
// windows; a repeatedly sampled phase cannot substitute for a missing phase.
class CurrentOffsetCollector {
public:
    static constexpr unsigned window_samples = 128;
    static constexpr unsigned window_count = 4;
    static constexpr unsigned samples_per_phase = window_samples * window_count;

    explicit CurrentOffsetCollector(float gain = 0.15f, CurrentOffsetLimits limits = {}) {
        reset(gain, limits);
    }

    void reset(float gain, CurrentOffsetLimits limits = {}) {
        gain_ = gain;
        limits_ = limits;
        phases_ = {};
        noise_a_ = {};
        standard_error_a_ = {};
        window_spread_a_ = {};
        drift_a_ = {};
        complete_ = false;
        error_ = nullptr;
        if (!std::isfinite(gain) || gain <= 0) {
            error_ = "offset_gain_invalid";
            return;
        }
        const float values[] = {limits.minimum_volts, limits.maximum_volts,
            limits.maximum_noise_a, limits.maximum_standard_error_a,
            limits.maximum_drift_a, limits.window_confidence_sigma};
        for (float value : values) {
            if (!std::isfinite(value)) {
                error_ = "offset_limits_invalid";
                return;
            }
        }
        if (limits.minimum_volts < 0 || limits.maximum_volts <= limits.minimum_volts ||
            limits.maximum_noise_a <= 0 || limits.maximum_standard_error_a <= 0 ||
            limits.maximum_drift_a < 0 || limits.window_confidence_sigma < 0)
            error_ = "offset_limits_invalid";
    }

    // Once a phase is full it is frozen, so uneven interleaving cannot bias
    // its result. Invalid input latches the first error until reset().
    bool add(int phase, float volts) {
        if (error_) return false;
        if (complete_) return true;
        if (phase < 0 || phase >= 3) {
            error_ = "offset_phase_invalid";
            return false;
        }
        if (!std::isfinite(volts)) {
            error_ = "offset_sample_nonfinite";
            return false;
        }
        auto& p = phases_[static_cast<unsigned>(phase)];
        if (p.total.count == samples_per_phase) return true;
        p.total.add(volts);
        p.window.add(volts);
        if (!std::isfinite(p.total.mean) || !std::isfinite(p.total.m2) ||
            !std::isfinite(p.window.mean) || !std::isfinite(p.window.m2)) {
            error_ = "offset_sample_nonfinite";
            return false;
        }
        if (p.window.count == window_samples) {
            const unsigned index = p.total.count / window_samples - 1;
            p.window_means[index] = p.window.mean;
            p.window_variances[index] = p.window.variance();
            p.window = {};
        }
        if (p.total.count == samples_per_phase) {
            finish_phase(static_cast<unsigned>(phase));
            if (error_) return false;
        }
        complete_ = phases_[0].total.count == samples_per_phase &&
                    phases_[1].total.count == samples_per_phase &&
                    phases_[2].total.count == samples_per_phase;
        return true;
    }

    bool complete() const { return complete_; }
    bool valid() const { return complete_ && !error_; }
    const char* error() const { return error_; }
    const char* validate() const {
        return error_ ? error_ : complete_ ? nullptr : "offset_samples_incomplete";
    }
    std::array<float, 3> means() const {
        return {phases_[0].total.mean, phases_[1].total.mean, phases_[2].total.mean};
    }
    std::array<unsigned, 3> counts() const {
        return {phases_[0].total.count, phases_[1].total.count, phases_[2].total.count};
    }
    const std::array<float, 3>& noise_a() const { return noise_a_; }
    const std::array<float, 3>& standard_error_a() const { return standard_error_a_; }
    const std::array<float, 3>& window_spread_a() const { return window_spread_a_; }
    const std::array<float, 3>& drift_a() const { return drift_a_; }

private:
    struct Moments {
        unsigned count = 0;
        float mean = 0, m2 = 0;
        void add(float value) {
            ++count;
            const float delta = value - mean;
            mean += delta / static_cast<float>(count);
            m2 += delta * (value - mean);
        }
        float variance() const {
            return count > 1 ? std::max(0.0f, m2 / static_cast<float>(count - 1)) : 0;
        }
    };
    struct Phase {
        Moments total{}, window{};
        std::array<float, window_count> window_means{}, window_variances{};
    };

    void finish_phase(unsigned phase) {
        const auto& p = phases_[phase];
        const float variance = p.total.variance();
        noise_a_[phase] = std::sqrt(variance) / gain_;
        // SEM assumes independent samples. The separate window comparison
        // detects slow bias changes that a small nominal SEM could conceal.
        standard_error_a_[phase] = noise_a_[phase] / std::sqrt(static_cast<float>(samples_per_phase));
        for (unsigned a = 0; a < window_count; ++a) {
            for (unsigned b = a + 1; b < window_count; ++b) {
                const float difference = std::abs(p.window_means[a] - p.window_means[b]) / gain_;
                const float uncertainty = limits_.window_confidence_sigma *
                    std::sqrt((p.window_variances[a] + p.window_variances[b]) /
                              static_cast<float>(window_samples)) / gain_;
                window_spread_a_[phase] = std::max(window_spread_a_[phase], difference);
                // Compare excess drift after allowing for random error in
                // the two window means; keep the raw spread for inspection.
                drift_a_[phase] = std::max(drift_a_[phase], std::max(0.0f, difference - uncertainty));
            }
        }
        if (!std::isfinite(noise_a_[phase]) || !std::isfinite(standard_error_a_[phase]) ||
            !std::isfinite(window_spread_a_[phase]) || !std::isfinite(drift_a_[phase]))
            error_ = "offset_sample_nonfinite";
        else if (p.total.mean < limits_.minimum_volts || p.total.mean > limits_.maximum_volts)
            error_ = "offset_mean_out_of_range";
        else if (noise_a_[phase] > limits_.maximum_noise_a)
            error_ = "offset_current_noise";
        else if (standard_error_a_[phase] > limits_.maximum_standard_error_a)
            error_ = "offset_mean_uncertain";
        else if (drift_a_[phase] > limits_.maximum_drift_a)
            error_ = "offset_mean_drift";
    }

    float gain_ = 0.15f;
    CurrentOffsetLimits limits_{};
    std::array<Phase, 3> phases_{};
    std::array<float, 3> noise_a_{}, standard_error_a_{}, window_spread_a_{}, drift_a_{};
    bool complete_ = false;
    const char* error_ = nullptr;
};

} // namespace knob
