#include "control.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace knob;

static bool near(float a, float b, float tolerance = 1e-5f) {
    return std::abs(a - b) <= tolerance;
}

static void snap_tracks_position() {
    for (float width : {pi / 180, pi / 18, pi / 6})
        for (float snap : {.5f, .55f, .75f, 1.0f})
            for (float direction : {-1.0f, 1.0f}) {
                Config c; c.width = width; c.snap = snap;
                c.strength = 1; c.damping = 0;
                Haptics h; h.reset(0);
                const float at_four_tenths = direction * h.step(c, direction * width * .4f, 0);
                const float before_midpoint = direction * h.step(c, direction * width * .49f, 0);
                assert(before_midpoint < at_four_tenths);
                assert(h.position() == 0);
                h.reset(0);
                float previous = 0;
                // Slow approach is one continuous restoring ramp. The old
                // profile peaked at 0.4 pitch and vanished before the snap.
                constexpr int points = 2000;
                for (int n = 1; n < points; ++n) {
                    const float angle = direction * width * snap * n / points;
                    const float opposing = direction * h.step(c, angle, 0);
                    assert(h.position() == 0);
                    assert(opposing <= previous + 1e-6f);
                    assert(previous - opposing <= width * snap / points + 2e-6f);
                    previous = opposing;
                }
                const float threshold = direction * width * snap;
                const float before = h.step(c, threshold, 0);
                assert(h.position() == 0); // Exact threshold is not a crossing.
                assert(before * direction < 0);
                const float after = h.step(c, threshold + direction * width * .0001f, 0);
                assert(h.position() == static_cast<int>(direction));
                assert(after * direction >= 0); // snap=1 lands in the center dead zone.
                assert((after - before) * direction > 0);
                assert(near(h.sub_position(c, threshold), direction * (snap - 1)));

                if (snap > .5f) {
                    // Returning across the forward threshold cannot bounce
                    // the logical center back or insert another force kick.
                    h.step(c, threshold - direction * width * .0001f, 0);
                    assert(h.position() == static_cast<int>(direction));
                    const float reverse = direction * width * (1 - snap);
                    h.step(c, reverse + direction * width * .0001f, 0);
                    assert(h.position() == static_cast<int>(direction));
                    const float returned = h.step(c, reverse - direction * width * .0001f, 0);
                    assert(h.position() == 0);
                    assert(returned * direction <= 0);
                }
            }
}

static void dead_zone_and_damping() {
    for (float width : {pi / 180, pi / 18, pi / 6})
        for (float direction : {-1.0f, 1.0f}) {
            Config c; c.width = width; c.strength = 2; c.damping = .003f;
            const float dead_zone = std::min(width * .2f, pi / 180);
            Haptics h; h.reset(0);
            for (float speed : {0.0f, .01f, .05f}) {
                assert(h.step(c, direction * dead_zone * .99f, direction * speed) == 0);
                assert(h.position() == 0);
            }
            // The reference keeps damping throughout a moving detent, even
            // at its center; only near-stationary center intent is quieted.
            for (float speed : {.051f, 1.0f, 5.0f}) {
                const float damped = h.step(c, direction * dead_zone * .5f, direction * speed);
                assert(near(damped, -direction * c.damping * speed));
            }
            const float outside = direction * (dead_zone + 1e-6f);
            const float tiny = h.step(c, outside, 0);
            assert(tiny * direction < 0 && std::abs(tiny) < 1e-5f);
            // No current-amplitude deadband may swallow a small restoring request.
            assert(near(tiny, -direction * c.strength * 1e-6f, 1e-8f));
            assert(h.step(c, 0, 0) == 0);
        }
}

static void exact_multi_detent_boundaries() {
    for (float snap : {.5f, .75f, 1.0f})
        for (int steps : {1, 4, 16})
            for (float direction : {-1.0f, 1.0f}) {
                Config c; c.width = .125f; c.snap = snap; c.damping = 0;
                Haptics h; h.reset(0);
                // Binary-exact geometry isolates strict > behavior from
                // rounding. 1+floor(excess/width) incorrectly advanced twice.
                const float threshold = direction * (steps + snap) * c.width;
                h.step(c, threshold, 0);
                assert(h.position() == static_cast<int>(direction) * steps);
                assert(h.sub_position(c, threshold) == direction * snap);
                const float beyond = std::nextafter(threshold,
                    direction * std::numeric_limits<float>::infinity());
                h.step(c, beyond, 0);
                assert(h.position() == static_cast<int>(direction) * (steps + 1));
                assert(std::abs(h.sub_position(c, beyond)) <= snap);
            }
}

static void detents_dissipate_work() {
    for (float width : {pi / 180, pi / 18, pi / 6})
        for (float snap : {.5f, .55f, .75f, 1.0f})
            for (float direction : {-1.0f, 1.0f}) {
                Config c; c.width = width; c.snap = snap; c.damping = 0;
                c.strength = 10; c.current_limit = .8f;
                Haptics h; h.reset(0);
                double work = 0;
                constexpr int points = 20000;
                for (int n = 0; n <= points; ++n) {
                    const float target = h.step(c, direction * width * n / points, 0);
                    assert(std::abs(target) <= c.current_limit);
                    work += target * direction * width / points;
                }
                // Hysteresis intentionally dissipates energy. Requiring the
                // old periodic curve's zero work would erase the snap again.
                const float integration_error = 3 * c.current_limit * width / points;
                assert(work <= integration_error);
                if (snap == .5f) assert(std::abs(work) <= integration_error);
                else assert(work < -integration_error);
            }
}

static void bounded_detents_and_tuning() {
    for (float direction : {-1.0f, 1.0f}) {
        Config c; c.mode = Mode::bounded; c.min_position = -2; c.max_position = 2;
        c.width = .125f; c.strength = 2; c.damping = 0;
        Haptics bounded, unbounded; bounded.reset(0); unbounded.reset(0);
        Config free_detents = c; free_detents.mode = Mode::detents;
        for (float unit : {.2f, .4f, .54f, .56f, .7f, 1.0f, 1.4f, 1.6f, 1.9f}) {
            const float angle = direction * unit * c.width;
            assert(near(bounded.step(c, angle, 0), unbounded.step(free_detents, angle, 0)));
            assert(bounded.position() == unbounded.position());
        }
        assert(bounded.step(c, direction * 3 * c.width, 0) * direction < 0);
        assert(bounded.position() == static_cast<int>(direction) * 2);
        assert(bounded.step(c, direction * 4 * c.width, 0) * direction < 0);
        assert(bounded.position() == static_cast<int>(direction) * 2);
        assert(bounded.step(c, direction * 2 * c.width, 0) == 0);

        Haptics h; h.reset(0);
        const float held_angle = direction * c.width * .4f;
        const float before = h.step(c, held_angle, 0);
        Config next = c; next.strength *= .5f;
        h.reconfigure(c, next, held_angle);
        assert(h.position() == 0);
        assert(near(h.step(next, held_angle, 0), before * .5f));
        assert(near(h.sub_position(next, held_angle), direction * .4f));
    }
}

static void assist_limits_and_current_range() {
    for (float direction : {-1.0f, 1.0f}) {
        Config c; c.damping = 0; c.current_limit = .8f;
        Haptics h; h.reset(0);
        assert(h.step(c, direction * .4f * c.width, direction * 30) * direction < -.7f);
        assert(near(h.step(c, direction * .6f * c.width, direction * 30), 0));
        const float full = h.step(c, direction * .6f * c.width, 0);
        assert(full * direction > .7f);
        for (float speed : {.1f, 5.0f, detent_assist_fade_start})
            assert(near(h.step(c, direction * .6f * c.width, direction * speed), full));
        const float halfway = (detent_assist_fade_start + detent_assist_cutoff) * .5f;
        assert(near(h.step(c, direction * .6f * c.width, direction * halfway), full * .5f));
        assert(near(h.step(c, direction * .6f * c.width, direction * detent_assist_cutoff), 0));

        c.current_limit = max_motor_current; c.strength = 100;
        assert(validate(c) == nullptr);
        h.reset(0);
        assert(near(h.step(c, direction * .4f * c.width, direction * 5), -direction * max_motor_current));
        assert(near(h.step(c, direction * .6f * c.width, direction * 5), direction * max_motor_current));
        assert(h.step(c, direction * c.width, direction * 5) == 0);
    }
}

static void quantized_slow_motion_pipeline() {
    // Production order with 14-bit encoder samples at 2.5 kHz and haptics at
    // 5 kHz. A slow hand turn must preserve the snap through prediction and
    // current limiting, including the angle wrap in either direction.
    constexpr float dt = .0002f, code = tau / 16384;
    for (float width : {pi / 180, pi / 18})
        for (float speed : {.05f, .5f, 5.0f})
            for (float direction : {-1.0f, 1.0f}) {
                Config c; c.width = width; c.damping = .003f;
                AngleTracker angle; assert(angle.update(0, dt));
                Haptics h; h.reset(0);
                HapticSpeedLimiter speed_limit;
                CurrentLimiter current; current.reset(c.current_limit);
                int previous_position = 0, crossings = 0;
                float previous_target = 0;
                const int iterations = static_cast<int>(3 * width / (speed * dt));
                for (int n = 1; n <= iterations; ++n) {
                    if (n % 2 == 0) {
                        const float measured = std::round(direction * speed * n * dt / code) * code;
                        assert(angle.update(wrap(measured), 2 * dt));
                    }
                    const float request = h.step(c, angle.haptic_angle(.0005f), angle.haptic_velocity());
                    const float faded = speed_limit.step(request, angle.velocity(), c.current_limit, dt);
                    const float target = current.step(faded, {}, 0, c.current_limit, dt);
                    assert(near(target, request));
                    assert(!speed_limit.paused() && !speed_limit.limited());
                    if (h.position() != previous_position) {
                        assert(h.position() - previous_position == static_cast<int>(direction));
                        assert(previous_target * direction < 0);
                        assert(target * direction > 0);
                        ++crossings;
                    } else {
                        // Include the 5 rad/s encoder step and predictor
                        // startup; a full-current kick must require a snap.
                        assert(std::abs(target - previous_target) < .08f);
                    }
                    previous_position = h.position(); previous_target = target;
                }
                assert(crossings == 3);
            }
}

static void high_strength_preserves_geometry_and_current_limit() {
    for (float width : {pi / 180, pi / 18})
        for (float snap : {.55f, .75f})
            for (float limit : {.02f, .8f, max_motor_current})
                for (float direction : {-1.0f, 1.0f}) {
                    Config c; c.strength = 1000; c.width = width; c.snap = snap;
                    c.current_limit = limit; c.damping = 0;
                    assert(validate(c) == nullptr);
                    Haptics h; h.reset(0);
                    const float dead_zone = std::min(width * .2f, pi / 180);
                    assert(h.step(c, direction * dead_zone * .99f, 0) == 0);
                    const float just_outside = direction * (dead_zone + 1e-7f);
                    const float tiny = h.step(c, just_outside, 0);
                    assert(tiny * direction < 0 && std::abs(tiny) < .0002f);
                    const float threshold = direction * width * snap;
                    assert(near(h.step(c, threshold, 0), -direction * limit));
                    assert(h.position() == 0);
                    const float after = h.step(c, threshold + direction * width * .0001f, 0);
                    assert(h.position() == static_cast<int>(direction));
                    assert(after * direction > 0 && std::abs(after) <= limit);
                    assert(h.step(c, direction * width, 0) == 0);
                    h.reset(0);
                    for (int n = 0; n <= 1000; ++n) {
                        const float target = h.step(c, direction * width * 3 * n / 1000, 0);
                        assert(std::abs(target) <= limit);
                    }
                }
}

static void quiet_output_and_immediate_targets() {
    Config elastic; elastic.mode = Mode::spring; elastic.damping = .003f;
    Haptics spring; spring.reset(0);
    assert(near(spring.step(elastic, 0, 5), -elastic_min_damping * 5));
    elastic.mode = Mode::detents; spring.reset(0);
    assert(near(spring.step(elastic, 0, 5), -elastic.damping * 5));
    // The elastic minimum damping must not leak into ordinary detents.
    QuietOutput quiet;
    for (int n = 0; n < 200; ++n) quiet.step(true, .1f, .0002f);
    assert(quiet.step(true, .1f, .0002f));
    assert(quiet.step(true, .5f, .0002f));
    assert(!quiet.step(false, 0, .0002f)); // Any nonzero intent resumes now.
    for (int n = 0; n < 200; ++n) quiet.step(true, 0, .0002f);
    assert(!quiet.step(true, 1, .0002f));
    for (int n = 0; n < 200; ++n) assert(!quiet.step(true, 2, .0002f));
    for (float dt : {.0001f, .0002f, .0005f, .001f}) {
        CurrentLimiter limit; limit.reset(max_motor_current);
        assert(limit.step(max_motor_current, {}, 0, max_motor_current, dt) == max_motor_current);
        assert(limit.step(0, {}, 0, max_motor_current, dt) == 0);
        assert(limit.step(max_motor_current, {}, 0, max_motor_current, dt) == max_motor_current);
        assert(limit.step(-max_motor_current, {}, 0, max_motor_current, dt) == -max_motor_current);
    }
}

int main() {
    snap_tracks_position();
    dead_zone_and_damping();
    exact_multi_detent_boundaries();
    detents_dissipate_work();
    bounded_detents_and_tuning();
    assist_limits_and_current_range();
    quantized_slow_motion_pipeline();
    high_strength_preserves_geometry_and_current_limit();
    quiet_output_and_immediate_targets();
    std::cout << "PASS: center-based detent snap, strict multi-step thresholds, hysteresis, continuous ramps, damping, passive detents, endstops and immediate current targets.\n";
}
