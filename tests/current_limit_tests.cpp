#include "control.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace knob;

int main() {
    CurrentTripFilter trips;
    for (int n = 0; n < 100; ++n) {
        assert(!trips.step(true, false)); // An isolated pulse must not latch off.
        assert(!trips.step(false, false));
    }
    for (int n = 0; n < 3; ++n) assert(!trips.step(true, false));
    assert(trips.step(true, false)); // Sustained excess must still shut down.
    trips.reset(); assert(trips.step(false, true)); // Severe current trips immediately.
    trips.reset(); assert(!trips.step(true, false));
    assert(!trips.step(true, false, false)); // Invalid samples break the qualification.
    for (int n = 0; n < 3; ++n) assert(!trips.step(true, false));
    assert(trips.step(true, false));
    for (float limit : {.02f, .2f, .65f, .8f, 1.0f}) {
        Config config; config.current_limit = limit;
        assert(!validate(config));
        assert(overcurrent_threshold(limit) > limit);
        assert(overcurrent_threshold(limit) <= 1.25f);
        CurrentLimiter limiter; limiter.reset(limit);
        float previous = 0;
        for (int n = 0; n < 50; ++n) {
            const float q = limiter.step(10, {}, 0, limit, .0005f);
            assert(q >= previous && q <= limit);
            assert(q - previous <= .050001f);
            previous = q;
        }
        assert(limiter.limited());
        assert(std::abs(previous - limit) < 1e-5f);
        for (int n = 0; n < 50; ++n) {
            const float q = limiter.step(-10, {}, 0, limit, .0005f);
            assert(q <= previous && q >= -limit);
            assert(previous - q <= .050001f);
            previous = q;
        }
        // Measured excess folds the target down without a fault or restart.
        const float before = limiter.capacity();
        for (int n = 0; n < 10; ++n) limiter.step(-10, {0, -limit * 1.1f}, limit * 1.1f, limit, .0005f);
        assert(limiter.capacity() < before);
        for (int n = 0; n < 300; ++n) limiter.step(10, {}, 0, limit, .0005f);
        assert(std::abs(limiter.capacity() - limit) < 1e-5f);
    }
    CurrentLimiter limiter; limiter.reset(1);
    for (int n = 0; n < 30; ++n) limiter.step(1, {}, 0, 1, .0005f);
    limiter.configure(.2f);
    assert(limiter.capacity() <= .2f);
    assert(std::abs(limiter.step(1, {0, .9f}, .9f, .2f, .0005f)) <= .2f);
    limiter.reset(1);
    for (int n = 0; n < 50; ++n) {
        const float q = limiter.step(1, {.8f, 0}, .8f, 1, .0005f);
        assert(q * q + .8f * .8f <= 1.000001f);
    }
    assert(limiter.step(std::numeric_limits<float>::quiet_NaN(), {}, 0, 1, .0005f) == 0);

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
    std::cout << "PASS: 1 A range, slew, reversal, measured foldback/recovery, d-axis budget, live lowering, delayed RL transients.\n";
}
