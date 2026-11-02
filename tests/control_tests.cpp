#include "control.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <algorithm>
using namespace knob;
static bool near(float a, float b, float tolerance = 1e-4f) { return std::abs(a - b) < tolerance; }
int main() {
    assert(near(phase_current(1.80f, 1.65f, .15f), 1.0f));
    assert(near(phase_current(1.50f, 1.65f, .15f), -1.0f));
    assert(near(phase_current(1.65f, 1.65f, .3f), 0.0f));
    Config c;
    assert(validate(c) == nullptr);
    c.current_limit = 1; assert(validate(c) == nullptr);
    c.current_limit = 1.01f; assert(validate(c));
    c = {}; c.sense_gain = .2f; assert(validate(c));
    c = {}; c.width = 0; assert(validate(c));
    c = {}; c.snap = .49f; assert(validate(c));
    c = {}; c.mode = static_cast<Mode>(5); assert(validate(c));
    c = {}; c.current_kp = std::numeric_limits<float>::quiet_NaN(); assert(validate(c));
    c = {}; c.bus_voltage = 4.5; c.voltage_limit = 1.5; assert(validate(c));
    c = {}; c.min_position = 12; c.max_position = 11; assert(validate(c));
    Calibration cal{true, -1, 7, 1}; assert(valid_calibration(cal));
    cal.pole_pairs = 12; assert(!valid_calibration(cal));
    // Line voltage must match the inverse Park/Clarke result across every sector,
    // including at sector boundaries and negative torque.
    for (int k = 0; k < 721; ++k) for (float q : {-1.0f, 0.0f, 1.0f}) {
        const float a = k * tau / 360;
        const Dq v{0.25f, q}; const auto pwm = svpwm(v, a, 11.62f);
        for (float duty : pwm) assert(duty >= duty_floor && duty <= duty_ceiling);
        const float alpha = v.d * std::cos(a) - v.q * std::sin(a);
        const float beta = v.d * std::sin(a) + v.q * std::cos(a);
        assert(near((pwm[0] - pwm[1]) * 11.62f, 1.5f * alpha - .8660254f * beta));
        const std::array<float, 3> currents{alpha, -.5f * alpha + .8660254f * beta, -.5f * alpha - .8660254f * beta};
        const auto dq = park(currents, a); assert(near(dq.d, v.d) && near(dq.q, v.q));
    }
    for (float v : {10.0f, 100.0f, -100.0f}) for (int k = 0; k < 100; ++k) {
        for (float duty : svpwm({v,v}, k*.07f, 5)) assert(duty >= duty_floor && duty <= duty_ceiling);
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
    AngleTracker reversal; float position = 0;
    assert(reversal.update(0, .0005f));
    for (int i = 0; i < 200; ++i) {
        position += .01f; assert(reversal.update(wrap(position), .0005f));
    }
    for (int i = 0; i < 6; ++i) {
        position -= .01f; assert(reversal.update(wrap(position), .0005f));
    }
    assert(reversal.haptic_velocity() < 0 && reversal.velocity() > 0);
    SettlingMonitor settling;
    for (int n = 0; n < 150; ++n) settling.update(.003f*std::sin(n), .001f);
    assert(!settling.still()); // A single short quiet interval is insufficient.
    for (int n = 0; n < 200; ++n) settling.update(.003f*std::sin(n), .001f);
    assert(settling.still());  // Quantization/weak-field jitter is not rotor motion.
    for (int n = 0; n < 100; ++n) settling.update(n*.001f, .001f);
    assert(!settling.still()); // Sustained rotation invalidates the alignment.
    settling.reset(); assert(!settling.still());
    Haptics h; c = {}; c.width = .1f; c.strength = 1; c.damping = 0;
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
    assert(h.step(next, .42f, 0) < 0);
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
    std::cout << "PASS: CSA polarity, configuration limits, 4326 transform checks, SVPWM bounds, PI anti-windup, angle wrapping, detents, endstops, damping, spring.\n";
}
