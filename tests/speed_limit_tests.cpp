#include "control.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace knob;
static bool near(float a, float b, float e = 1e-5f) { return std::abs(a-b) <= e; }

static void boundaries() {
    for (float sign : {-1.0f, 1.0f}) {
        HapticSpeedLimiter speed;
        assert(near(speed.step(.6f, sign*25, 1, .0005f), .6f));
        assert(!speed.limited() && !speed.paused());
        float previous = 1;
        // Saturated effects must progressively fade before the cutoff.
        for (float velocity : {26.0f, 28.0f, 32.5f, 37.0f, 39.9f}) {
            float out = 0;
            for (int i = 0; i < 30; ++i) out = speed.step(20, sign*velocity, 1, .0005f);
            assert(out > 0 && out < previous && !speed.paused());
            if (velocity == 32.5f) assert(near(out, .5f));
            previous = out;
        }
        for (int i = 0; i < 30; ++i) speed.step(1, sign*40, 1, .0005f);
        assert(speed.paused() && near(speed.scale(), 0));
        // Exact cutoff and recovery boundaries; noise below cutoff cannot chatter.
        for (float v : {39.99f, 35.0f, 25.001f, 39.0f})
            for (int i = 0; i < 500; ++i) {
                assert(near(speed.step(1, sign*v, 1, .0005f), 0));
                assert(speed.paused());
            }
        for (int i = 0; i < 500; ++i) assert(speed.step(1, sign*5.001f, 1, .0005f) == 0);
        // Brief low-speed crossings during a reversal cannot reconnect torque.
        for (int repeat = 0; repeat < 3; ++repeat) {
            for (int i = 0; i < 40; ++i) assert(speed.step(1, 0, 1, .0005f) == 0);
            assert(speed.step(1, sign*10, 1, .0005f) == 0);
        }
        for (int i = 0; i < 55; ++i) assert(speed.step(1, sign*5, 1, .0005f) == 0);
        for (int i = 0; i < 10; ++i) previous = speed.step(1, sign*5, 1, .0005f);
        assert(previous > 0 && previous < .01f && !speed.paused());
        for (int i = 0; i < 1020; ++i) {
            const float out = speed.step(1, 0, 1, .0005f);
            assert(out >= previous && out-previous <= .001001f);
            previous = out;
        }
        assert(near(previous, 1) && !speed.limited());
    }
}

static void slew() {
    for (float dt : {.00025f, .0005f, .001f, .002f})
        for (float limit : {.02f, .2f, .65f, 1.0f})
            for (float sign : {-1.0f, 1.0f}) {
                HapticSpeedLimiter speed;
                float previous = limit;
                float out = std::abs(speed.step(sign*50, 60, limit, dt));
                assert(out > 0 && out < previous);
                for (int i = 1; i < static_cast<int>(.012f/dt)+1; ++i) {
                    assert(out <= previous+1e-6f);
                    assert(previous-out <= limit*dt/.010f+1e-6f);
                    previous = out;
                    out = std::abs(speed.step(sign*50, -60, limit, dt));
                }
                assert(near(out, 0) && speed.paused());
                // An interrupted recovery must fade again without a kick.
                for (int i = 0; i < static_cast<int>(.05f/dt); ++i)
                    out = std::abs(speed.step(sign*50, 0, limit, dt));
                assert(out > 0 && out < limit*.11f);
                previous = out;
                out = std::abs(speed.step(sign*50, 40, limit, dt));
                assert(out <= previous && previous-out <= limit*dt/.010f+1e-6f);
                for (int i = 0; i < static_cast<int>(.012f/dt)+1; ++i)
                    out = std::abs(speed.step(sign*50, 45, limit, dt));
                assert(near(out, 0));
                speed.reset();
                assert(!speed.paused() && !speed.limited());
                assert(near(speed.step(sign*50, 0, limit, dt), sign*limit));
            }
}

static void integrated_motion() {
    // Production ordering: wrapped encoder, detents, clicks, speed envelope,
    // electrical direction, current target slew. Test fine detents at speed.
    for (Mode mode : {Mode::free, Mode::detents, Mode::bounded, Mode::spring})
        for (float sign : {-1.0f, 1.0f})
            for (float electrical_sign : {-1.0f, 1.0f})
                for (float dt : {.0005f, .001f, .002f}) {
                    Config c; c.mode = mode; c.width = pi/180;
                    c.min_position = -12; c.max_position = 12;
                    c.current_limit = 1; c.strength = 10; c.endstop = 20; c.damping = .5f;
                    AngleTracker angle; assert(angle.update(0, dt));
                    Haptics h; h.reset(0);
                    HapticSpeedLimiter speed;
                    CurrentLimiter current; current.reset(1);
                    float position = 0, previous_target = 0;
                    for (int i = 0; i < static_cast<int>(.3f/dt); ++i) {
                        position += sign*60*dt;
                        assert(angle.update(wrap(position), dt));
                        const float click = i%12 < 6 ? .08f : -.08f;
                        const float request = h.step(c, angle.angle(), angle.haptic_velocity())+click;
                        const float faded = speed.step(request, angle.velocity(), 1, dt);
                        const float target = current.step(faded*electrical_sign, {}, 0, 1, dt);
                        assert(std::abs(target) <= 1+1e-6f);
                        assert(std::abs(target-previous_target) <= 100*dt+1e-5f);
                        previous_target = target;
                        if (i*dt > .05f) {
                            assert(speed.paused() && near(speed.scale(), 0));
                            assert(near(target, 0)); // clicks must not bypass the fade
                        }
                        if (mode == Mode::detents || mode == Mode::free)
                            assert(std::abs(h.sub_position(c, angle.angle())) <= c.snap+.002f);
                    }
                    if (mode == Mode::bounded) assert(h.position() == static_cast<int>(sign*12));
                    else if (mode == Mode::spring) assert(h.position() == 0);
                    else assert(std::abs(h.position()) > 900);
                    // Reconfiguration and zeroing must not reset the speed latch.
                    const Config old = c; c.width *= 2;
                    h.reconfigure(old, c, angle.angle());
                    assert(near(speed.step(.08f, angle.velocity(), 1, dt), 0));
                    h.reset(angle.angle());
                    assert(near(speed.step(.08f, angle.velocity(), 1, dt), 0));
                    // Measured speed decays and torque returns without re-enabling.
                    float previous_scale = 0;
                    for (int i = 0; i < static_cast<int>(.7f/dt)+1; ++i) {
                        assert(angle.update(wrap(position), dt));
                        const float faded = speed.step(.68f, angle.velocity(), 1, dt);
                        const float target = current.step(faded*electrical_sign, {}, 0, 1, dt);
                        assert(speed.scale() >= previous_scale);
                        assert(speed.scale()-previous_scale <= dt/.5f+1e-6f);
                        assert(std::abs(target-previous_target) <= 100*dt+1e-5f);
                        previous_scale = speed.scale(); previous_target = target;
                    }
                    assert(!speed.paused() && !speed.limited());
                    assert(near(previous_target, .68f*electrical_sign));
                }
    for (Mode mode : {Mode::spring, Mode::bounded}) {
        Config c; c.mode = mode;
        Haptics h; h.reset(0);
        HapticSpeedLimiter speed;
        for (int i = 0; i < 30; ++i)
            speed.step(h.step(c, -100, -60), -60, c.current_limit, .0005f);
        assert(near(speed.scale(), 0));
        const float request = h.step(c, -100, 0);
        assert(near(request, c.current_limit)); // original center is retained
        assert(speed.step(request, 0, c.current_limit, .0005f) == 0);
        float restored = 0;
        for (int i = 0; i < 65; ++i) restored = speed.step(request, 0, c.current_limit, .0005f);
        assert(restored > 0 && restored < .01f);
    }
}

static void elastic_return() {
    for (float direction : {-1.0f, 1.0f}) {
        Config c; c.mode = Mode::spring; c.damping = 0;
        Haptics h; h.reset(0);
        const float angle = direction * 10;
        // Toward the origin: full at 10 rad/s, less at 20, none at 25.
        assert(near(h.step(c, angle, -direction*10), -direction*c.current_limit));
        assert(near(h.step(c, angle, -direction*20), -direction*c.current_limit/3));
        assert(near(h.step(c, angle, -direction*25), 0));
        // Away from the origin: braking remains available at any speed.
        assert(near(h.step(c, angle, direction*30), -direction*c.current_limit));
        c.mode = Mode::bounded; c.min_position = -1; c.max_position = 1;
        h.reset(0);
        assert(near(h.step(c, angle, -direction*20), -direction*c.current_limit/3));
        assert(near(h.step(c, angle, -direction*25), 0));
        assert(near(h.step(c, angle, direction*30), -direction*c.current_limit));
    }
}

static void invalid_inputs() {
    for (float v : {std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::infinity(),
                    -std::numeric_limits<float>::infinity()}) {
        HapticSpeedLimiter speed;
        assert(speed.step(v, 0, 1, .0005f) == 0);
        speed.reset(); assert(speed.step(1, v, 1, .0005f) == 0);
        speed.reset(); assert(speed.step(1, 0, v, .0005f) == 0);
        speed.reset(); assert(speed.step(1, 0, 1, v) == 0);
        assert(speed.paused() && speed.scale() == 0);
        assert(speed.step(1, 0, 1, .0005f) < .01f);
    }
    for (float dt : {0.0f, -.001f, .0021f}) {
        HapticSpeedLimiter speed; assert(speed.step(1, 0, 1, dt) == 0);
    }
    for (float limit : {0.0f, -.1f, 1.01f}) {
        HapticSpeedLimiter speed; assert(speed.step(1, 0, limit, .0005f) == 0);
    }
}
int main() {
    boundaries(); slew(); integrated_motion(); elastic_return(); invalid_inputs();
    std::cout << "PASS: speed fade/recovery, hysteresis, all modes, fine detents, clicks, current slew, invalid inputs.\n";
}
