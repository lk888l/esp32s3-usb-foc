#include "control.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace knob;

int main() {
    CurrentTripFilter trips;
    using Level = CurrentTripLevel;
    // One 200 us outlier, followed by sufficient valid recovery, must not
    // disable the bridge even when it crosses the fixed peak band.
    for (int n = 0; n < 100; ++n) {
        assert(!trips.step(Level::peak, 200));
        assert(!trips.step(Level::normal, 800));
    }
    // Both duration bands are independent of control rate. A gross excursion
    // bypasses qualification, including on the first sample after reset.
    for (unsigned period_us : {100U, 200U, 250U, 500U, 1000U}) {
        for (auto [level, duration] : {std::pair{Level::overload, current_overload_trip_us},
                                      std::pair{Level::peak, current_peak_trip_us}}) {
            trips.reset();
            for (unsigned elapsed = period_us; elapsed < duration; elapsed += period_us)
                assert(!trips.step(level, period_us));
            assert(trips.step(level, period_us));
        }
        trips.reset(); assert(trips.step(Level::emergency, period_us));
    }
    trips.reset(); assert(trips.step(Level::emergency, 0));
    // Repeated 50% duty overload cannot evade protection by dipping under the
    // threshold. Recovery is 4:1 slower, so this eventually exhausts both bands.
    for (auto [level, duration] : {std::pair{Level::overload, current_overload_trip_us},
                                  std::pair{Level::peak, current_peak_trip_us}}) {
        trips.reset();
        bool tripped = false;
        for (unsigned elapsed = 0; elapsed < 3 * duration; elapsed += 400) {
            if (trips.step(level, 200)) { tripped = true; break; }
            assert(!trips.step(Level::normal, 200));
        }
        assert(tripped);
        // Invalid ADC readings neither qualify new current nor erase the
        // preceding exposure. A later valid sample completes qualification.
        trips.reset();
        assert(!trips.step(level, duration - 200));
        assert(!trips.step(Level::invalid, 1000000));
        assert(trips.step(level, 200));
        // Explicit reset (disable/clear/new current measurement) starts anew.
        trips.reset(); assert(!trips.step(level, 200));
    }
    // Quantized timing and short dips must preserve fractional recovery:
    // 800 us of peaks, then 200 us safe -> 750 us accumulated, not zero.
    trips.reset(); assert(!trips.step(Level::peak, 800));
    assert(!trips.step(Level::normal, 200));
    assert(!trips.step(Level::peak, 249));
    assert(trips.step(Level::peak, 1));
    // At the maximum operating limit the ordinary and peak thresholds meet;
    // the 1 ms peak band still takes precedence over the 20 ms overload band.
    assert(overcurrent_threshold(max_motor_current) == peak_current_trip);
    assert(max_motor_current == 4.5f);
    assert(peak_current_trip == 5.0f && absolute_current_trip == 6.0f);
    assert(max_motor_current < peak_current_trip);
    assert(peak_current_trip < absolute_current_trip);
    Config invalid;
    invalid.current_limit = std::nextafter(max_motor_current, std::numeric_limits<float>::infinity());
    assert(validate(invalid));
    for (float limit : {.02f, .2f, .65f, .8f, 1.0f, 2.5f, max_motor_current}) {
        Config config; config.current_limit = limit;
        assert(!validate(config));
        assert(overcurrent_threshold(limit) > limit);
        assert(overcurrent_threshold(limit) <= peak_current_trip);
        CurrentLimiter limiter; limiter.reset(limit);
        // The first sample carries the complete requested detent peak; a
        // reversal or release must not retain the preceding direction.
        assert(limiter.step(10, {}, 0, limit, .0002f) == limit);
        assert(limiter.limited());
        assert(limiter.step(-10, {}, 0, limit, .0002f) == -limit);
        assert(limiter.step(0, {}, 0, limit, .0002f) == 0);
        assert(limiter.step(limit * .5f, {}, 0, limit, .0002f) == limit * .5f);
        assert(!limiter.limited());
        // Measured excess folds the target down without a fault or restart.
        const float before = limiter.capacity();
        for (int n = 0; n < 10; ++n) limiter.step(-10, {0, -limit * 1.1f}, limit * 1.1f, limit, .0005f);
        assert(limiter.capacity() < before);
        for (int n = 0; n < 300; ++n) limiter.step(10, {}, 0, limit, .0005f);
        assert(std::abs(limiter.capacity() - limit) < 1e-5f);
    }
    CurrentLimiter limiter; limiter.reset(max_motor_current);
    assert(limiter.step(max_motor_current, {}, 0, max_motor_current, .0002f) == max_motor_current);
    limiter.configure(.2f);
    assert(limiter.capacity() <= .2f);
    assert(std::abs(limiter.step(max_motor_current, {0, max_motor_current}, max_motor_current, .2f, .0002f)) <= .2f);
    for (float limit : {1.0f, max_motor_current}) {
        limiter.reset(limit);
        const float d = limit * .8f;
        for (int n = 0; n < 50; ++n) {
            const float q = limiter.step(limit, {d, 0}, d, limit, .0002f);
            assert(q > 0 && q < limit);
            assert(q * q + d * d <= limit * limit + 1e-5f);
        }
    }
    assert(limiter.step(std::numeric_limits<float>::quiet_NaN(), {}, 0, 1, .0005f) == 0);
    assert(limiter.step(1, {}, 0, invalid.current_limit, .0002f) == 0);

    // Closed-loop RL model with one sample of actuation delay, abrupt torque
    // reversal, external back-EMF, and a live reduction of the operating limit.
    // This checks controller behaviour, not a physical peak-current guarantee.
    for (float resistance : {2.0f, 3.6f}) {
        CurrentLoop loop; limiter.reset(.8f);
        float current = 0, delayed_voltage = 0;
        bool foldback_observed = false;
        for (int n = 0; n < 2500; ++n) {
            const float dt = n % 101 == 0 ? .001f : .0005f;
            const float limit = n < 1300 ? .8f : n < 1800 ? 1.0f : .2f;
            const float demand = n < 800 ? 5.0f : n < 1300 ? -5.0f : 5.0f;
            const float emf = n >= 500 && n < 700 ? .3f : 0;
            const float target = limiter.step(demand, {0, current}, std::abs(current), limit, dt);
            assert(std::abs(target) <= limit);
            const auto voltage = loop.step({0, target - current}, dt, 1.5f, 300, 2.35f);
            const float decay = std::exp(-resistance * dt / .00086f);
            current = decay * current + (1 - decay) * (delayed_voltage + emf) / resistance;
            delayed_voltage = voltage.q;
            foldback_observed |= limiter.capacity() < limit - .01f;
            // A downward change gets the same 20 ms settling grace as firmware.
            if (n < 1800 || n > 1840) assert(std::abs(current) < overcurrent_threshold(limit));
            if (n > 2000) assert(std::abs(current) <= limit + .02f);
        }
        assert(foldback_observed);
        assert(current > .15f); // It recovers torque rather than staying disabled.
    }
    // A short detent pulse must release promptly. Feedforward is included in
    // saturation/anti-windup and tolerates winding resistance model error.
    for (float resistance : {3.6f, 4.5f, 5.4f}) {
        float release_energy[2]{};
        for (int ff = 0; ff < 2; ++ff) {
            CurrentLoop loop;
            float current = 0, delayed_voltage = 0;
            constexpr float dt = .0002f;
            const float decay = std::exp(-resistance * dt / .00086f);
            for (int n = 0; n < 400; ++n) {
                const float target = n < 50 ? .25f : n < 150 ? 0 : n < 200 ? -.25f : 0;
                const auto v = loop.step({0, target-current}, dt, 2, 600, 2.35f,
                                        {0, ff ? 4.5f*target : 0});
                assert(v.d*v.d + v.q*v.q <= 2.35f*2.35f + 1e-5f);
                current = decay*current + (1-decay)*delayed_voltage/resistance;
                delayed_voltage = v.q;
                assert(std::abs(current) < .5f);
                if (n >= 50 && n < 150) release_energy[ff] += std::abs(current)*dt;
                if (n > 350) assert(std::abs(current) < .01f);
            }
        }
        assert(release_energy[1] < release_energy[0]*.6f);
    }
    CurrentLoop saturation;
    for (int n = 0; n < 100; ++n) {
        const auto v = saturation.step({1,1}, .0002f, 2, 600, 2.35f, {4.5f,4.5f});
        assert(v.d*v.d + v.q*v.q <= 2.35f*2.35f + 1e-5f);
    }
    const auto released = saturation.step({}, .0002f, 2, 600, 2.35f);
    assert(std::abs(released.d) < 1e-5f && std::abs(released.q) < 1e-5f);
    std::cout << "PASS: current limits, staged rate-independent trips, repeated/invalid samples, transients, feedforward release and anti-windup.\n";
}
