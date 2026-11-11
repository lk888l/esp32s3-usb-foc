#include "control.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <algorithm>
using namespace knob;
static bool near(float a, float b, float tolerance = 1e-4f) { return std::abs(a - b) < tolerance; }
static bool valid_duty(float value) {
    return (pwm_zero_clamp && value == 0) || (value >= duty_floor && value <= duty_ceiling);
}
static void test_voltage_capacity() {
    Config config;
    // Old saved voltage limits occupy a reserved slot but never constrain
    // output or invalidate an otherwise usable configuration.
    config.bus_voltage = 11.62f;
    config.legacy_voltage_limit = 2.35f;
    assert(validate(config) == nullptr);
    config.bus_voltage = 19.4f;
    config.legacy_voltage_limit = std::numeric_limits<float>::quiet_NaN();
    assert(validate(config) == nullptr);
    for (float bus : {4.5f, 5.0f, 11.62f, 19.4f, 24.0f, 30.0f}) {
        config.bus_voltage = bus;
        assert(validate(config) == nullptr);
        const float radius = max_output_voltage(bus);
        if constexpr (pwm_full_range) assert(near(radius, bus / std::sqrt(3.0f)));
        // Verify line-to-line voltage at the accepted circle boundary through
        // all sectors, rather than testing only that duty has been clipped.
        for (int k = 0; k <= 360; ++k) for (float sign : {-1.0f, 1.0f}) {
            const float angle = k * tau / 360;
            const Dq voltage{radius * .6f, sign * radius * .8f};
            const auto pwm = svpwm(voltage, angle, bus);
            for (float value : pwm) assert(valid_duty(value));
            const float alpha = voltage.d * std::cos(angle) - voltage.q * std::sin(angle);
            const float beta = voltage.d * std::sin(angle) + voltage.q * std::cos(angle);
            assert(near((pwm[0] - pwm[1]) * bus, 1.5f * alpha - .8660254f * beta, 2e-4f));
            assert(near((pwm[1] - pwm[2]) * bus, 1.7320508f * beta, 2e-4f));
        }
    }
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    for (float bus : {-1.0f, 0.0f, 4.49f, 30.01f, nan, infinity, -infinity}) {
        assert(max_output_voltage(bus) == 0);
        config.bus_voltage = bus;
        assert(validate(config));
        const auto pwm = svpwm({1, 2}, 0, bus);
        for (float duty : pwm) assert(duty == duty_floor);
    }

    // Simulate bus sag/recovery while both PI axes are demanding saturation.
    // The PI must observe the same reduced voltage circle as the modulator,
    // and reconstructing line voltages must not expose extra PWM clipping.
    CurrentLoop loop;
    for (float bus : {19.4f, 11.62f, 4.5f, 24.0f, 19.4f}) {
        const float limit = max_output_voltage(bus);
        for (int n = 0; n < 100; ++n) {
            const Dq output = loop.step({1.0f, 2.0f}, .0002f, .4f, 200.0f, limit);
            assert(output.d * output.d + output.q * output.q <= limit * limit + 1e-5f);
            const float angle = n * .17f;
            const auto pwm = svpwm(output, angle, bus);
            const float alpha = output.d * std::cos(angle) - output.q * std::sin(angle);
            const float beta = output.d * std::sin(angle) + output.q * std::cos(angle);
            assert(near((pwm[0] - pwm[1]) * bus, 1.5f * alpha - .8660254f * beta, 2e-4f));
            assert(near((pwm[1] - pwm[2]) * bus, 1.7320508f * beta, 2e-4f));
        }
    }
    const auto stopped = loop.step({1.0f, 2.0f}, .0002f, .4f, 200.0f,
                                   max_output_voltage(nan));
    assert(stopped.d == 0 && stopped.q == 0);

    // A recovered/rising supply restores full capacity without any nominal
    // or persisted-voltage clamp; every calculation uses the present sample.
    assert(max_output_voltage(24.0f) > max_output_voltage(19.4f));
    assert(near(max_output_voltage(12.0f) * 2, max_output_voltage(24.0f)));
    if constexpr (pwm_full_range) assert(near(max_output_voltage(12.0f), 6.928203f));
}
static void test_zero_clamp_modulation() {
    if constexpr (!pwm_zero_clamp) return;
    constexpr float bus = 19.15f;
    const float radius = max_output_voltage(bus);
    assert(near(radius, bus * duty_ceiling / std::sqrt(3.0f)));
    bool saw_clamped = false, saw_offset = false;
    std::array<float, 3> previous{};
    for (float fraction : {0.0f, .001f, .01f, .1f, .5f, .9f, 1.0f, 1.001f, 2.0f, 100.0f}) {
        for (int n = 0; n <= 7200; ++n) {
            const float angle = n * tau / 7200;
            const Dq requested{radius * fraction, 0};
            const auto pwm = svpwm(requested, angle, bus);
            for (float pulse : pwm) assert(valid_duty(pulse));
            const float smallest = *std::min_element(pwm.begin(), pwm.end());
            if (fraction == 1) {
                saw_clamped |= smallest == 0;
                saw_offset |= smallest == duty_floor;
            }
            const float alpha = requested.d * std::cos(angle);
            const float beta = requested.d * std::sin(angle);
            const float expected_ab = 1.5f * alpha - .8660254f * beta;
            const float expected_bc = 1.7320508f * beta;
            const float actual_ab = (pwm[0] - pwm[1]) * bus;
            const float actual_bc = (pwm[1] - pwm[2]) * bus;
            if (fraction <= 1) {
                assert(near(actual_ab, expected_ab, 2e-4f));
                assert(near(actual_bc, expected_bc, 2e-4f));
                if (n && fraction == 1) {
                    // Common-mode switching can jump at the clamp threshold,
                    // but the line voltages must remain smooth across it and
                    // across all six electrical sector boundaries.
                    assert(std::abs(actual_ab - (previous[0] - previous[1]) * bus) < .02f);
                    assert(std::abs(actual_bc - (previous[1] - previous[2]) * bus) < .02f);
                }
            } else {
                // Over-range requests may shrink uniformly, never rotate the
                // requested vector or create a pulse below the minimum.
                const float denominator = expected_ab * expected_ab + expected_bc * expected_bc;
                const float scale = (actual_ab * expected_ab + actual_bc * expected_bc) / denominator;
                assert(scale > 0 && scale <= 1.00001f);
                assert(near(actual_ab, scale * expected_ab, 2e-4f));
                assert(near(actual_bc, scale * expected_bc, 2e-4f));
            }
            previous = pwm;
        }
    }
    assert(saw_clamped && (pwm_zero_floor || saw_offset));
    // At the largest line span, the zero clamp preserves the requested voltage
    // right up to the profile's linear circle, including the sampling ceiling.
    const float voltage = std::min(7.2f, radius);
    const auto pwm = svpwm({voltage, 0}, pi / 6, bus);
    assert(*std::min_element(pwm.begin(), pwm.end()) == 0);
    assert(near(*std::max_element(pwm.begin(), pwm.end()), std::sqrt(3.0f) * voltage / bus));
}
static void test_zero_floor_modulation() {
    if constexpr (!pwm_zero_floor) return;
    assert(duty_floor == 0 && pwm_zero_clamp);
    if constexpr (pwm_full_range) {
        assert(duty_ceiling == 1);
        assert(near(max_output_voltage(12), 6.928203f));
    } else if constexpr (duty_ceiling == .65f) {
        assert(pwm_profile_hz == 20000 && duty_ceiling == .65f);
        assert(near(max_output_voltage(12), 4.503332f));
        // The quantized ceiling reserves 700 timer ticks on each side of TEZ.
        assert(std::lround(2000 * (1 - duty_ceiling)) == 700);
    } else {
        assert(pwm_profile_hz == 20000 && duty_ceiling == .90f);
        assert(near(max_output_voltage(12), 6.235383f));
        assert(near(max_output_voltage(12), .9f * 12 / std::sqrt(3.0f)));
        assert(std::lround(2000 * (1 - duty_ceiling)) == 200);
    }
    bool reaches_ceiling = false;
    for (float bus : {4.5f, 12.0f, 19.4f, 30.0f}) {
        const float radius = max_output_voltage(bus);
        for (int n = 0; n <= 7200; ++n) {
            const float angle = n * tau / 7200;
            const auto pwm = svpwm({radius, 0}, angle, bus);
            assert(*std::min_element(pwm.begin(), pwm.end()) == 0);
            reaches_ceiling |= near(*std::max_element(pwm.begin(), pwm.end()), duty_ceiling, 1e-6f);
            const float common = (pwm[0] + pwm[1] + pwm[2]) / 3;
            std::array<float, 3> reconstructed{};
            for (int p = 0; p < 3; ++p) reconstructed[p] = (pwm[p] - common) * bus;
            const auto dq = park(reconstructed, angle);
            assert(near(dq.d, radius, 3e-4f));
            assert(near(dq.q, 0, 3e-4f));
        }
    }
    assert(reaches_ceiling);
    // Reducing the sampling ceiling must not reactivate the old 4% pulse
    // floor: tiny nonzero requests stay proportional through every sector.
    for (float voltage : {1e-5f, 1e-3f, .01f}) {
        for (int n = 0; n < 720; ++n) {
            const float angle = n * tau / 720;
            const auto pwm = svpwm({voltage, 0}, angle, 12);
            const float peak = *std::max_element(pwm.begin(), pwm.end());
            assert(*std::min_element(pwm.begin(), pwm.end()) == 0);
            assert(peak > 0 && peak < .04f);
            const float alpha = voltage * std::cos(angle);
            const float beta = voltage * std::sin(angle);
            assert(near((pwm[0] - pwm[1]) * 12, 1.5f * alpha - .8660254f * beta, 1e-7f));
            assert(near((pwm[1] - pwm[2]) * 12, 1.7320508f * beta, 1e-7f));
        }
    }
    const float largest = std::numeric_limits<float>::max();
    const float infinity = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (float angle : {0.0f, .1f, -tau, 2*tau, largest, -largest}) {
        for (Dq requested : {Dq{largest, largest}, Dq{largest, -largest}, Dq{-largest, largest}, Dq{-largest, -largest}})
            for (float duty : svpwm(requested, angle, 12)) assert(std::isfinite(duty) && valid_duty(duty));
    }
    for (float invalid : {nan, infinity, -infinity}) {
        for (Dq requested : {Dq{invalid, 1}, Dq{1, invalid}})
            for (float duty : svpwm(requested, 0, 12)) assert(duty == 0);
        for (float duty : svpwm({1, 2}, invalid, 12)) assert(duty == 0);
    }
    for (float duty : svpwm({}, 1.2f, 12)) assert(duty == 0);
}
static void test_direct_angle_prediction() {
    // Every encoder position is available to haptics on its first update.
    // Exercise both supported resolutions and reset across the wrap boundary.
    for (unsigned counts : {4096U, 16384U}) {
        const float code = tau / counts;
        AngleTracker tracker;
        for (int run = 0; run < 2; ++run) {
            assert(tracker.update(tau - code, .0004f));
            assert(near(tracker.haptic_angle(), tau - code, 1e-6f));
            assert(tracker.update(0, .0004f));
            assert(near(tracker.angle(), tau, 1e-6f));
            assert(tracker.update(code, .0004f));
            assert(near(tracker.haptic_angle(), tau + code, 1e-6f));
            assert(tracker.haptic_velocity() > 0);
            assert(tracker.update(tau - code, .0004f));
            assert(near(tracker.haptic_angle(), tau - code, 1e-6f));
            // Invalid samples leave the last trustworthy position intact.
            const float saved = tracker.haptic_angle();
            assert(!tracker.update(1, .0004f));
            assert(!tracker.update(0, 0));
            assert(!tracker.update(std::numeric_limits<float>::quiet_NaN(), .0004f));
            assert(tracker.haptic_angle() == saved);
            tracker.reset();
            assert(tracker.angle() == 0 && tracker.wrapped_angle() == 0);
            assert(tracker.velocity() == 0 && tracker.haptic_velocity() == 0);
        }
    }
    // Timestamp compensation follows constant motion, including multi-turn
    // travel, unequal encoder periods, and reversals through zero.
    for (float speed : {-20.0f, -2.0f, 2.0f, 20.0f}) {
        AngleTracker tracker;
        float position = speed < 0 ? .01f : tau - .01f;
        assert(tracker.update(position, .0004f));
        for (int n = 0; n < 1000; ++n) {
            const float dt = n % 2 ? .0003f : .0005f;
            position += speed * dt;
            assert(tracker.update(wrap(position), dt));
            assert(near(tracker.haptic_angle(), position, 3e-6f));
        }
        assert(near(tracker.haptic_velocity(), speed, .01f));
        for (float horizon : {0.0f, .000005f, .0004f, .002f})
            assert(near(tracker.haptic_angle(horizon), position + speed * horizon, 2e-5f));
        assert(tracker.haptic_angle(-1) == tracker.angle());
        assert(tracker.haptic_angle(1) == tracker.haptic_angle(.002f));
        for (int n = 0; n < 2; ++n) {
            position -= speed * .0004f;
            assert(tracker.update(wrap(position), .0004f));
            assert(near(tracker.haptic_angle(), position, 3e-6f));
        }
        assert(tracker.haptic_velocity() * speed < 0); // Reversal within 0.8 ms.
        assert((tracker.haptic_angle(.0004f) - tracker.angle()) * speed < 0);
    }
}
int main() {
    assert(near(phase_current(1.80f, 1.65f, .15f), 1.0f));
    assert(near(phase_current(1.50f, 1.65f, .15f), -1.0f));
    assert(near(phase_current(1.65f, 1.65f, .3f), 0.0f));
    Config c;
    assert(validate(c) == nullptr);
    c.current_limit = max_motor_current; assert(validate(c) == nullptr);
    c.current_limit = max_motor_current + .01f; assert(validate(c));
    c = {}; c.strength = max_haptic_strength; assert(validate(c) == nullptr);
    c.strength = max_haptic_strength + .01f; assert(validate(c));
    c.strength = std::nextafter(max_haptic_strength, std::numeric_limits<float>::infinity()); assert(validate(c));
    c = {}; c.strength = 1000; assert(validate(c) == nullptr);
    assert(max_motor_current == 4.5f); // Strength range never expands the motor current limit.
    c = {}; c.sense_gain = .2f; assert(validate(c));
    c = {}; c.width = 0; assert(validate(c));
    c = {}; c.snap = .49f; assert(validate(c));
    for (int mode = 0; mode < mode_count; ++mode) {
        c = {}; c.mode = static_cast<Mode>(mode); assert(!validate(c));
    }
    c = {}; c.mode = static_cast<Mode>(mode_count); assert(validate(c));
    c.mode = static_cast<Mode>(-1); assert(validate(c));
    c = {}; c.current_kp = std::numeric_limits<float>::quiet_NaN(); assert(validate(c));
    c = {}; c.bus_voltage = 4.5; c.legacy_voltage_limit = 999; assert(validate(c) == nullptr);
    c = {}; c.min_position = 12; c.max_position = 11; assert(validate(c));
    test_voltage_capacity();
    test_zero_clamp_modulation();
    test_zero_floor_modulation();
    Calibration cal{true, -1, 7, 1}; assert(valid_calibration(cal));
    cal.pole_pairs = 12; assert(!valid_calibration(cal));
    // Line voltage must match the inverse Park/Clarke result across every sector,
    // including at sector boundaries and negative torque.
    for (int k = 0; k < 721; ++k) for (float q : {-3.0f, -1.0f, 0.0f, 1.0f, 3.0f}) {
        const float a = k * tau / 360;
        const Dq v{0.25f, q}; const auto pwm = svpwm(v, a, 11.62f);
        for (float duty : pwm) assert(valid_duty(duty));
        const float alpha = v.d * std::cos(a) - v.q * std::sin(a);
        const float beta = v.d * std::sin(a) + v.q * std::cos(a);
        assert(near((pwm[0] - pwm[1]) * 11.62f, 1.5f * alpha - .8660254f * beta));
        const std::array<float, 3> currents{alpha, -.5f * alpha + .8660254f * beta, -.5f * alpha - .8660254f * beta};
        const auto dq = park(currents, a); assert(near(dq.d, v.d) && near(dq.q, v.q));
    }
    for (float v : {10.0f, 100.0f, -100.0f}) for (int k = 0; k < 100; ++k) {
        for (float duty : svpwm({v,v}, k*.07f, 5)) assert(valid_duty(duty));
    }
    for (float duty : svpwm({1,1}, 0, 0)) assert(duty == duty_floor);
    Pi controller;
    for (int i = 0; i < 10000; ++i) assert(near(controller.step(10, .0005, 1, 50, 1), 1));
    assert(controller.step(-.1, .0005, 1, 50, 1) < 0); // no windup after saturation
    controller.reset(); assert(near(controller.step(0, .0005, 1, 50, 1), 0));
    CurrentLoop current_loop;
    assert(near(current_loop.step({0,10}, .0005, 1.5, 300, 2.2).q, 2.2f));
    for (int n = 0; n < 1000; ++n) {
        const auto v = current_loop.step({std::sin(n*.1f)*5, std::cos(n*.1f)*5}, .0005, 1.5, 300, 2.2);
        assert(v.d*v.d + v.q*v.q <= 2.2f*2.2f + 1e-5f);
    }
    current_loop.reset();
    float simulated_current = 0, delayed_voltage = 0;
    const float decay = std::exp(-3.6f*.0005f/.00086f);
    for (int n = 0; n < 2000; ++n) {
        const auto v = current_loop.step({0,.6f-simulated_current}, .0005, 1.5, 300, 2.4);
        simulated_current = decay*simulated_current + (1-decay)*delayed_voltage/3.6f;
        delayed_voltage = v.q;
        assert(std::abs(simulated_current) < .8f);
    }
    assert(near(simulated_current, .6f, .005f));
    AngleTracker angle;
    assert(angle.update(tau-.02f, .0005)); assert(angle.update(.01f, .0005));
    assert(near(angle.angle(), tau+.01f)); assert(angle.velocity() > 0);
    assert(angle.update(tau-.01f, .0005)); assert(near(angle.angle(), tau-.01f));
    assert(!angle.update(2, .0005)); assert(!angle.update(0, 0));
    // Fast damping feedback changes sign during a reversal while the quieter
    // speed-envelope estimate is still decaying from the previous direction.
    // Damping must reverse within 1 ms; the old 2 ms filter failed this bound.
    AngleTracker reversal; float position = 0;
    assert(reversal.update(0, .0005f));
    for (int i = 0; i < 200; ++i) {
        position += .01f; assert(reversal.update(wrap(position), .0005f));
    }
    for (int i = 0; i < 2; ++i) {
        position -= .01f; assert(reversal.update(wrap(position), .0005f));
    }
    assert(reversal.haptic_velocity() < 0 && reversal.velocity() > 0);
    test_direct_angle_prediction();
    SettlingMonitor settling;
    for (int n = 0; n < 150; ++n) settling.update(.003f*std::sin(n), .001f);
    assert(!settling.still()); // A single short quiet interval is insufficient.
    for (int n = 0; n < 200; ++n) settling.update(.003f*std::sin(n), .001f);
    assert(settling.still());  // Quantization/weak-field jitter is not rotor motion.
    for (int n = 0; n < 100; ++n) settling.update(n*.001f, .001f);
    assert(!settling.still()); // Sustained rotation invalidates the alignment.
    settling.reset(); assert(!settling.still());
    Haptics h; c = {}; c.width = .1f; c.strength = 1; c.damping = 0; c.snap = .6f;
    h.reset(0); assert(h.step(c, .04f, 0) < 0); assert(h.position() == 0);
    h.step(c, .061f, 0); assert(h.position() == 1);
    h.step(c, .059f, 0); assert(h.position() == 1); // hysteresis prevents chatter
    h.step(c, .041f, 0); assert(h.position() == 1);
    h.step(c, .039f, 0); assert(h.position() == 0);
    h.reset(0); h.step(c, .45f, 0); assert(h.position() == 4);
    c.mode = Mode::bounded; c.min_position = 0; c.max_position = 2; h.reset(0);
    assert(h.step(c, -.2f, 0) > 0); assert(h.position() == 0);
    assert(h.step(c, 1, 0) < 0); assert(h.position() == 2);
    c.mode = Mode::free; c.damping = .02f; h.reset(0);
    assert(near(h.step(c, .2f, 0), 0)); assert(h.step(c, .2f, 1) < 0);
    c.mode = Mode::spring; c.damping = 0; h.reset(1);
    assert(h.step(c, 1.1f, 0) < 0); assert(h.step(c, .9f, 0) > 0);
    assert(near(h.step(c, 100, 0), -c.current_limit));
    // Speed attenuation is shared by all effects after position tracking.
    assert(near(h.step(c, 100, 41), -c.current_limit));
    // Online strength tuning preserves the detent and its center.
    c = {}; c.width = .1f; c.damping = 0;
    h.reset(0); h.step(c, .42f, 0); assert(h.position() == 4);
    const float sub = h.sub_position(c, .42f);
    Config next = c; next.strength = .7f; next.current_limit = .3f;
    h.reconfigure(c, next, .42f);
    assert(h.position() == 4); assert(near(h.sub_position(next, .42f), sub));
    assert(near(h.step(next, .42f, 0), -next.strength * (.02f - pi / 180)));
    // Mode and spacing changes establish a center at the current angle.
    c = next; next.mode = Mode::bounded; next.min_position = 3; next.max_position = 5;
    h.reconfigure(c, next, 1.2f);
    assert(h.position() == 3); assert(near(h.step(next, 1.2f, 0), 0));
    assert(h.step(next, 1.1f, 0) > 0);
    c = next; next.width = .2f; h.reconfigure(c, next, 1.1f);
    assert(near(h.sub_position(next, 1.1f), 0));
    // Spring tuning must retain the original return position.
    c = next; next.mode = Mode::spring; h.reconfigure(c, next, 2);
    c = next; next.strength = .5f; h.reconfigure(c, next, 2.1f);
    assert(h.step(next, 2.1f, 0) < 0);
    std::cout << "PASS: CSA polarity, configuration/voltage limits, sector transforms, SVPWM voltage and duty bounds, PI anti-windup, angle wrapping, detents, endstops, damping, spring.\n";
}
