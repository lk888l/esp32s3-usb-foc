#include "current_offsets.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

using namespace knob;

namespace {
bool error_is(const CurrentOffsetCollector& c, const char* expected) {
    return c.error() && std::strcmp(c.error(), expected) == 0;
}

template <typename Sample>
void collect(CurrentOffsetCollector& c, Sample sample) {
    for (unsigned n = 0; n < CurrentOffsetCollector::samples_per_phase; ++n)
        for (int phase = 0; phase < 3; ++phase)
            c.add(phase, sample(phase, n));
}
}

int main() {
    constexpr float gain = 0.15f;
    CurrentOffsetCollector c(gain);
    assert(!c.complete() && !c.valid() && !c.error());
    assert(std::strcmp(c.validate(), "offset_samples_incomplete") == 0);
    const std::array<float, 3> centers{1.48f, 1.51f, 1.53f};

    // Quantization-scale noise on three distinct zero offsets must not be
    // mistaken for physical current, and all phases must be represented.
    collect(c, [&](int p, unsigned n) { return centers[p] + (n % 2 ? 1 : -1) * 0.0004f; });
    assert(c.complete() && c.valid() && !c.validate());
    for (unsigned p = 0; p < 3; ++p) {
        assert(c.counts()[p] == 512);
        assert(std::abs(c.means()[p] - centers[p]) < 0.000003f);
        assert(c.noise_a()[p] > 0.002f && c.noise_a()[p] < 0.004f);
        assert(c.standard_error_a()[p] < 0.0002f);
        assert(c.window_spread_a()[p] < 0.00002f);
    }
    const auto fixed = c.means();
    c.add(0, 2.0f);
    assert(c.means() == fixed && c.valid());

    // Uneven channel ordering freezes completed phases but never pretends
    // that repeated samples from one phase fill either missing phase.
    c.reset(gain);
    for (unsigned n = 0; n < 900; ++n) assert(c.add(0, centers[0]));
    assert(!c.complete() && c.counts()[0] == 512 && c.counts()[1] == 0);
    for (unsigned n = 0; n < 512; ++n) assert(c.add(2, centers[2]));
    assert(!c.complete());
    for (unsigned n = 0; n < 511; ++n) assert(c.add(1, centers[1]));
    assert(!c.complete());
    assert(c.add(1, centers[1]) && c.valid());

    // The same current noise has the same acceptance and metrics for all
    // supported CSA gains. Welford retains small noise around a 1.5 V bias.
    for (float test_gain : {0.15f, 0.3f, 0.6f, 1.2f}) {
        c.reset(test_gain);
        collect(c, [&](int p, unsigned n) {
            return centers[p] + test_gain * (n % 2 ? 0.055f : -0.055f);
        });
        assert(c.valid());
        for (float noise : c.noise_a()) assert(noise > 0.054f && noise < 0.056f);
        for (float sem : c.standard_error_a()) assert(sem < 0.0025f);
    }

    // Moderate window-mean variation can be explained by noise. Keep the
    // raw 0.03 A spread visible while applying its sampling uncertainty.
    c.reset(gain);
    collect(c, [&](int p, unsigned n) {
        const float bias = (static_cast<int>(n / 128) - 1.5f) * 0.010f;
        const float noise = n % 2 ? 0.10f : -0.10f;
        return centers[p] + gain * (bias + noise);
    });
    assert(c.valid());
    assert(c.window_spread_a()[0] > 0.029f && c.window_spread_a()[0] < 0.031f);
    assert(c.drift_a()[0] < 0.001f);

    // A systematic change in zero is rejected even if overall RMS and
    // nominal SEM are comfortably inside the noise acceptance envelope.
    c.reset(gain);
    collect(c, [&](int p, unsigned n) {
        const float bias = n >= 256 ? 0.04f : 0.0f;
        return centers[p] + gain * (bias + (n % 2 ? 0.002f : -0.002f));
    });
    assert(error_is(c, "offset_mean_drift") && !c.complete());
    assert(c.noise_a()[0] < 0.03f && c.standard_error_a()[0] < 0.002f);
    assert(c.drift_a()[0] > 0.038f);

    c.reset(gain);
    collect(c, [&](int p, unsigned n) { return centers[p] + gain * (n % 2 ? 0.16f : -0.16f); });
    assert(error_is(c, "offset_current_noise") && !c.valid());

    CurrentOffsetLimits tight_sem;
    tight_sem.maximum_standard_error_a = 0.001f;
    c.reset(gain, tight_sem);
    collect(c, [&](int p, unsigned n) { return centers[p] + gain * (n % 2 ? 0.05f : -0.05f); });
    assert(error_is(c, "offset_mean_uncertain"));

    // Mean range is tested per phase, without averaging faults away across
    // the three current channels.
    for (float invalid_mean : {0.69f, 2.21f}) {
        c.reset(gain);
        collect(c, [&](int p, unsigned) { return p == 1 ? invalid_mean : centers[p]; });
        assert(error_is(c, "offset_mean_out_of_range"));
    }

    // Nonfinite values and malformed phase indices latch their first error;
    // later plausible samples cannot turn the record into a valid offset.
    for (float invalid : {std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::infinity(),
                          -std::numeric_limits<float>::infinity()}) {
        c.reset(gain);
        assert(!c.add(0, invalid));
        assert(error_is(c, "offset_sample_nonfinite"));
        collect(c, [&](int p, unsigned) { return centers[p]; });
        assert(error_is(c, "offset_sample_nonfinite") && !c.complete());
        assert(c.counts() == (std::array<unsigned, 3>{0, 0, 0}));
    }
    for (int phase : {-1, 3, 1000}) {
        c.reset(gain);
        assert(!c.add(phase, 1.5f));
        assert(error_is(c, "offset_phase_invalid"));
        assert(!c.add(0, 1.5f) && !c.complete());
    }
    for (float bad_gain : {0.0f, -0.15f, std::numeric_limits<float>::quiet_NaN(),
                           std::numeric_limits<float>::infinity()}) {
        c.reset(bad_gain);
        assert(error_is(c, "offset_gain_invalid"));
        assert(!c.add(0, 1.5f) && !c.complete());
    }
    CurrentOffsetLimits bad_limits;
    bad_limits.maximum_noise_a = -1;
    c.reset(gain, bad_limits);
    assert(error_is(c, "offset_limits_invalid"));
    bad_limits = {};
    bad_limits.window_confidence_sigma = std::numeric_limits<float>::infinity();
    c.reset(gain, bad_limits);
    assert(error_is(c, "offset_limits_invalid"));

    // Reset recovers the collector after errors, erases stale metrics, and
    // supports a fresh startup with different offsets and gain.
    c.reset(0.3f);
    assert(!c.error() && !c.complete());
    assert(c.noise_a() == (std::array<float, 3>{}));
    collect(c, [](int p, unsigned) { return 1.60f + 0.01f * p; });
    assert(c.valid() && std::abs(c.means()[2] - 1.62f) < 0.000001f);
    std::cout << "PASS: per-phase zero coverage, stable variance, gain-scaled noise, drift confidence, and terminal faults.\n";
}
