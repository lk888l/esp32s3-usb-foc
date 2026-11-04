#include "control.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
using namespace knob;

int main() {
    for (float width : {pi/180, pi/18, pi/6}) {
        Config c; c.width = width; c.strength = 10; c.current_limit = .8f; c.damping = .003f;
        Haptics h; h.reset(0);
        // The middle 60% of each interval has no commanded torque, even in motion.
        for (int n = -29; n <= 29; ++n)
            assert(std::abs(h.step(c, width*n/100, 8)) < 1e-6f);
        c.damping = 0;
        h.reset(0);
        assert(h.step(c, width*.4f, 0) < 0);
        assert(h.step(c, width*.6f, 0) > 0);
        // A full turn through an ideal detent must not create net mechanical work.
        for (float sign : {-1.0f, 1.0f}) {
            h.reset(0); float work = 0, previous = 0, peak = 0;
            constexpr int points = 20000;
            for (int n = 0; n <= points; ++n) {
                const float torque = h.step(c, sign*width*n/points, 0);
                work += torque*sign*width/points;
                assert(std::abs(torque-previous) < .005f);
                peak = std::max(peak, std::abs(torque)); previous = torque;
            }
            assert(std::abs(work) < 1e-5f);
            assert(peak > std::min(c.current_limit, c.strength*width*.5f)*.99f);
        }
        // Logical snap hysteresis must never move the physical torque barrier.
        Haptics forward, backward; forward.reset(0); backward.reset(0);
        backward.step(c, width, 0);
        for (float unit : {.32f, .4f, .49f, .51f, .6f, .68f})
            assert(std::abs(forward.step(c, unit*width, 0)-backward.step(c, unit*width, 0)) < 1e-5f);
    }
    // Low-speed crossings retain the full snap into the next detent. Fast
    // flicks still lose assist before the global speed cutoff, while the
    // approaching barrier retains its resisting torque.
    for (float direction : {-1.0f, 1.0f}) {
        Config c; c.damping = 0; c.current_limit = .8f;
        Haptics h; h.reset(0);
        const float resisting = h.step(c, direction*.4f*c.width, direction*30);
        assert(resisting*direction < -.7f);
        assert(std::abs(h.step(c, direction*.6f*c.width, direction*30)) < 1e-6f);
        const float full = h.step(c, direction*.6f*c.width, 0);
        assert(full*direction > .7f);
        for (float speed : {0.1f, 5.0f, detent_assist_fade_start})
            assert(std::abs(h.step(c, direction*.6f*c.width, direction*speed) - full) < 1e-5f);
        const float tapered = h.step(c, direction*.6f*c.width,
                                      direction*(detent_assist_fade_start + detent_assist_cutoff)*.5f);
        assert(std::abs(tapered - full*.5f) < 1e-5f);
        assert(std::abs(h.step(c, direction*.6f*c.width, direction*detent_assist_cutoff)) < 1e-6f);
    }
    // The expanded strength range can actually request the full 4.5 A at a
    // 10-degree pitch. Raising current_limit alone used to leave a 0.87 A cap.
    for (float direction : {-1.0f, 1.0f}) {
        Config c; c.current_limit = max_motor_current; c.strength = 60; c.damping = 0;
        assert(validate(c) == nullptr);
        Haptics h; h.reset(0);
        assert(std::abs(h.step(c, direction*.4f*c.width, direction*5) + direction*max_motor_current) < 1e-4f);
        assert(std::abs(h.step(c, direction*.6f*c.width, direction*5) - direction*max_motor_current) < 1e-4f);
        assert(h.step(c, direction*c.width, direction*5) == 0);
        h.reset(0);
        float work = 0, previous = 0;
        constexpr int points = 20000;
        for (int n = 0; n <= points; ++n) {
            const float target = h.step(c, direction*c.width*n/points, direction*5);
            work += target*direction*c.width/points;
            assert(std::abs(target) <= max_motor_current);
            assert(std::abs(target - previous) < .005f);
            previous = target;
        }
        assert(std::abs(work) < 1e-5f);
    }
    // Do not drag torque from the last detent into a released gap or reversal.
    Config elastic; elastic.mode = Mode::spring; elastic.damping = .003f;
    Haptics spring; spring.reset(0);
    assert(std::abs(spring.step(elastic, 0, 5) + elastic_min_damping*5) < 1e-5f);
    elastic.mode = Mode::detents; spring.reset(0);
    assert(spring.step(elastic, 0, 5) == 0); // Elastic damping never leaks into a detent gap.
    // An idle zero request must be electrically quiet, while even a tiny
    // intentional detent torque must resume continuously (no amplitude relay).
    QuietOutput quiet;
    for (int n = 0; n < 200; ++n) quiet.step(true, .1f, .0002f);
    assert(quiet.step(true, .1f, .0002f));
    assert(quiet.step(true, .5f, .0002f)); // Speed hysteresis rejects sensor chatter.
    assert(!quiet.step(false, 0, .0002f)); // Any nonzero intent resumes now.
    for (int n = 0; n < 200; ++n) quiet.step(true, 0, .0002f);
    assert(!quiet.step(true, 1, .0002f)); // PI must cancel back-EMF during motion.
    for (int n = 0; n < 200; ++n) assert(!quiet.step(true, 2, .0002f));
    for (float dt : {.0001f, .0002f, .0005f, .001f}) {
        CurrentLimiter limit; limit.reset(max_motor_current);
        assert(limit.step(max_motor_current, {}, 0, max_motor_current, dt) == max_motor_current);
        assert(limit.step(0, {}, 0, max_motor_current, dt) == 0);
        assert(limit.step(max_motor_current, {}, 0, max_motor_current, dt) == max_motor_current);
        assert(limit.step(-max_motor_current, {}, 0, max_motor_current, dt) == -max_motor_current);
    }
    std::cout << "PASS: quiet gaps, localized bidirectional barriers, zero net detent work, continuous snap, immediate release/reversal.\n";
}
