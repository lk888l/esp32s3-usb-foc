#include "frictionless.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace knob;
using namespace knob::frictionless_detail;
constexpr float dt = .0002f;

static float run(Frictionless& controller, const FrictionlessConfig& config,
                 float velocity, float seconds, float limit = .8f, bool drive = false) {
    float output = 0;
    for (int i = 0; i < std::lround(seconds / dt); ++i) {
        output = controller.step(config, velocity, dt, limit, drive);
        assert(std::isfinite(output) && std::abs(output) <= limit + 1e-7f);
    }
    return output;
}

static void rest_and_motion_gate() {
    Frictionless c;
    FrictionlessConfig cfg;
    for (int i = 0; i < 100000; ++i) {
        const float noise = .15f * std::sin(i * .01f);
        assert(c.step(cfg, noise, dt, .8f) == 0);
        assert(c.reference() == 0);
    }
    assert(run(c, cfg, .4f, 1) == 0);
    c.reset();
    for (int i = 0; i < 20; ++i) assert(c.step(cfg, 2, dt, .8f) == 0);
    assert(run(c, cfg, 2, .04f) > 0);
    assert(c.phase() == FRICTIONLESS_HOLD_ACQUIRE);
    run(c, cfg, 2, .7f);
    assert(c.phase() == FRICTIONLESS_HOLD_TRACK);
    assert(std::abs(c.reference() - 2) < .01f);
    assert(run(c, cfg, 0, .1f) == 0);
    assert(c.phase() == FRICTIONLESS_HOLD_REST && c.reference() == 0);
    assert(run(c, cfg, 0, 2) == 0);
}

static void observed_handover_and_periodic_disturbance() {
    Frictionless c;
    FrictionlessConfig cfg;
    assert(run(c, cfg, 8, .7f, .8f, true) == 0);
    assert(c.phase() == FRICTIONLESS_HOLD_OBSERVE && c.reference() == 0);
    const float first = c.step(cfg, 8, dt, .8f);
    assert(first > 0 && c.phase() == FRICTIONLESS_HOLD_TRACK);
    const float reference = c.reference();
    assert(std::abs(reference - 8) < .001f);
    run(c, cfg, 8, .5f);
    for (int i = 0; i < 25000; ++i) {
        const float velocity = 8 + .4f * std::sin(2 * pi * 7 * i * dt);
        c.step(cfg, velocity, dt, .8f);
        assert(c.phase() == FRICTIONLESS_HOLD_TRACK);
        assert(c.reference() == reference);
    }
}

static void gesture_yield_and_recapture() {
    FrictionlessConfig cfg;
    for (int sign : {-1, 1}) {
        Frictionless c;
        run(c, cfg, sign * 8, .5f, .8f, true);
        run(c, cfg, sign * 8, .6f);
        assert(c.phase() == FRICTIONLESS_HOLD_TRACK);
        bool yielded = false;
        // A sustained hand acceleration must release the old velocity goal.
        for (int i = 0; i < 3000; ++i) {
            const float v = sign * (8 + std::min(8.0f, i * dt * 160));
            c.step(cfg, v, dt, .8f);
            yielded |= c.phase() == FRICTIONLESS_HOLD_YIELD;
        }
        assert(yielded);
        run(c, cfg, sign * 16, 1.6f);
        assert(c.phase() == FRICTIONLESS_HOLD_TRACK);
        assert(std::abs(c.reference() - sign * 16) < .5f);
        // A slower deliberate push also releases after persistent extra effort.
        yielded = false;
        for (int i = 0; i < 10000; ++i) {
            const float v = sign * (16 - std::min(5.0f, i * dt * 5));
            c.step(cfg, v, dt, .8f);
            yielded |= c.phase() == FRICTIONLESS_HOLD_YIELD;
        }
        assert(yielded);
        run(c, cfg, sign * 11, 1.6f);
        assert(c.phase() == FRICTIONLESS_HOLD_TRACK);
        assert(std::abs(c.reference() - sign * 11) < .5f);
    }
}

static void reversal_and_restart() {
    Frictionless c;
    FrictionlessConfig cfg;
    run(c, cfg, 8, .6f, .8f, true);
    run(c, cfg, 8, .5f);
    bool crossed = false;
    int zero_ticks = 0;
    for (int i = 0; i < 700; ++i) {
        const float out = c.step(cfg, -8, dt, .8f);
        if (!crossed && c.velocity() < -.2f) crossed = true;
        if (crossed && zero_ticks < 190) {
            assert(out == 0 && c.reference() == 0);
            ++zero_ticks;
        }
    }
    assert(zero_ticks == 190);
    assert(run(c, cfg, -8, .5f) < 0);
    assert(c.reference() < -7.9f);
}

static void budgets_and_overcompensation() {
    FrictionlessConfig cfg;
    cfg.friction = .8f; cfg.scale = 10;
    Frictionless c;
    run(c, cfg, 8, .5f, .8f, true);
    run(c, cfg, 8, .1f);
    assert(c.feedforward() <= .060001f);
    // Hold must be able to cancel all nominal feedforward before requesting
    // opposing current, including when the extra-current cap is smaller.
    const float braking = run(c, cfg, 12, .04f);
    assert(braking < 0);
    for (const float limit : {.02f, .005f, .0001f}) {
        for (int i = 0; i < 1000; ++i) {
            const float out = c.step(cfg, 12, dt, limit);
            assert(std::abs(out) <= limit + 1e-7f);
            assert(std::abs(c.feedforward()) <= limit + 1e-7f);
        }
    }
    assert(run(c, cfg, 12, .021f, 0) == 0 && c.reference() == 0);
    cfg.scale = 0;
    assert(run(c, cfg, 12, 1) == 0);
    assert(c.reference() == 0 && c.correction() == 0 && c.feedforward() == 0);
}

static void invalid_input_clears_state() {
    FrictionlessConfig cfg;
    assert(!validate_frictionless(cfg));
    for (float FrictionlessConfig::* field : {&FrictionlessConfig::friction,
         &FrictionlessConfig::viscous, &FrictionlessConfig::scale,
         &FrictionlessConfig::hold_kp, &FrictionlessConfig::hold_ki}) {
        auto invalid = cfg; invalid.*field = std::numeric_limits<float>::quiet_NaN();
        assert(validate_frictionless(invalid));
        Frictionless c;
        run(c, cfg, 8, .7f);
        assert(c.step(invalid, 8, dt, .8f) == 0 && c.reference() == 0);
    }
    for (const float bad : {0.0f, -.001f, .00201f,
                            std::numeric_limits<float>::infinity()}) {
        Frictionless c;
        run(c, cfg, 8, .7f);
        assert(c.step(cfg, 8, bad, .8f) == 0 && c.reference() == 0);
    }
    Frictionless c;
    run(c, cfg, 8, .7f);
    assert(c.step(cfg, std::numeric_limits<float>::infinity(), dt, .8f) == 0);
    assert(c.reference() == 0 && c.feedforward() == 0 && c.correction() == 0);
}

static void closed_loop_loss_residual() {
    // A simple rotor model checks the feedback signs and long-term drift. It
    // is not a physical calibration: choose an intentionally wrong nominal
    // loss, add angle-periodic load, and require PI to cancel the residual.
    FrictionlessConfig cfg;
    cfg.friction = .026f;
    for (int sign : {-1, 1}) {
        Frictionless c;
        float velocity = sign * 8.0f, angle = 0;
        run(c, cfg, velocity, .8f, .8f, true);
        c.step(cfg, velocity, dt, .8f);
        const float reference = c.reference();
        float tail_sum = 0;
        int tail_count = 0;
        for (int i = 0; i < 50000; ++i) {
            const float out = c.step(cfg, velocity, dt, .8f);
            const float load = sign * .022f + .000304f * velocity +
                               .003f * std::sin(12 * angle);
            velocity += dt * (out - load) / .0004f;
            angle += dt * velocity;
            assert(c.phase() == FRICTIONLESS_HOLD_TRACK);
            assert(c.reference() == reference);
            assert(velocity * sign > 0 && std::abs(velocity) < 10);
            if (i >= 40000) { tail_sum += velocity; ++tail_count; }
        }
        assert(std::abs(tail_sum / tail_count - reference) < .03f);
        assert(sign * c.correction() < -.003f);
    }
}

static void temporary_budget_loss() {
    FrictionlessConfig cfg;
    for (const float duration : {.0002f, .001f, .019f}) {
        Frictionless c;
        run(c, cfg, 10, .5f, .8f, true);
        run(c, cfg, 10, .5f);
        const float reference = c.reference();
        const float normal_output = c.step(cfg, 10, dt, .15f);
        assert(c.phase() == FRICTIONLESS_HOLD_TRACK);
        assert(run(c, cfg, 10, duration, 0) == 0);
        assert(c.phase() == FRICTIONLESS_HOLD_TRACK && c.reference() == reference);
        assert(c.correction() == 0);
        const float resumed = c.step(cfg, 10, dt, .15f);
        assert(c.phase() == FRICTIONLESS_HOLD_TRACK && c.reference() == reference);
        assert(std::abs(resumed - normal_output) < 1e-6f);
        // A tiny positive budget still runs anti-windup without recapturing.
        assert(std::abs(c.step(cfg, 10, dt, .0001f)) <= .0001001f);
        assert(c.reference() == reference && c.phase() == FRICTIONLESS_HOLD_TRACK);
    }
    Frictionless c;
    run(c, cfg, 10, .5f, .8f, true);
    run(c, cfg, 10, .5f);
    assert(run(c, cfg, 10, .02f, 0) == 0);
    assert(c.reference() == 0 && c.phase() == FRICTIONLESS_HOLD_REST);
    assert(c.feedforward() == 0 && c.correction() == 0);
    run(c, cfg, 10, .01f);
    assert(c.phase() == FRICTIONLESS_HOLD_ACQUIRE);

    // Zero capacity must not suspend the velocity filter or stop/reversal
    // detection. A strong reversal crosses the 8 ms filter within the grace.
    c.reset();
    run(c, cfg, 10, .5f, .8f, true);
    run(c, cfg, 10, .5f);
    assert(run(c, cfg, -10, .012f, 0) == 0);
    assert(c.velocity() < -.2f && c.reference() == 0);
    assert(c.phase() == FRICTIONLESS_HOLD_REST);
    assert(c.step(cfg, -10, dt, .15f) == 0);
    assert(run(c, cfg, 0, .1f, 0) == 0 && c.reference() == 0);

    c.reset();
    run(c, cfg, .8f, .5f, .8f, true);
    run(c, cfg, .8f, .2f);
    run(c, cfg, .3f, .2f);
    assert(c.reference() != 0);
    assert(run(c, cfg, 0, .015f, 0) == 0);
    assert(c.velocity() < .2f && c.reference() == 0);

    // Preserve a learned nonzero integral, not merely a zero-error reference.
    c.reset();
    auto integral_cfg = cfg;
    integral_cfg.hold_kp = 0;
    run(c, integral_cfg, 10, .5f, .8f, true);
    run(c, integral_cfg, 10, .5f);
    run(c, integral_cfg, 9.8f, .3f);
    assert(c.correction() > .001f);
    auto uninterrupted = c;
    const float expected = uninterrupted.step(integral_cfg, 9.8f, dt, .15f);
    assert(run(c, integral_cfg, 9.8f, .001f, 0) == 0);
    assert(std::abs(c.step(integral_cfg, 9.8f, dt, .15f) - expected) < 1e-7f);

    for (const float invalid : {-.01f, std::numeric_limits<float>::quiet_NaN(),
                                std::numeric_limits<float>::infinity()}) {
        run(c, cfg, 10, .6f);
        assert(c.reference() != 0);
        assert(c.step(cfg, 10, dt, invalid) == 0);
        assert(c.reference() == 0 && c.velocity() == 0 && c.feedforward() == 0);
    }
}

static void periodic_motion_does_not_retarget() {
    FrictionlessConfig cfg;
    for (const float frequency : {3.0f, 5.0f, 10.0f, 20.0f, 30.0f}) {
        for (const int sign : {-1, 1}) {
            Frictionless c;
            run(c, cfg, sign * 10, .5f, .8f, true);
            run(c, cfg, sign * 10, .5f);
            const float reference = c.reference();
            for (int i = 0; i < 25000; ++i) {
                const float velocity = sign * (10 + 3 * std::sin(2 * pi * frequency * i * dt));
                c.step(cfg, velocity, dt, .8f);
                if (c.phase() != FRICTIONLESS_HOLD_TRACK || c.reference() != reference)
                    std::cerr << "Periodic motion retargeted at " << frequency << " Hz, sign "
                              << sign << ", t=" << i * dt << "\n";
                assert(c.phase() == FRICTIONLESS_HOLD_TRACK);
                assert(c.reference() == reference);
            }
        }
    }
}

static void adapted_correction_budget() {
    FrictionlessConfig cfg;
    cfg.hold_kp = .1f; // Exercise the actual correction boundaries, not just a loose bound.
    for (const int sign : {-1, 1}) {
        for (const float budget : {.02f, .05f, .08f, .8f}) {
            Frictionless c;
            run(c, cfg, sign * 5, .5f, budget, true);
            run(c, cfg, sign * 5, .5f, budget);
            float peak_assist = 0, peak_opposing = 0;
            // Sharp bounded excursions exercise both limits before the
            // separate hand-motion detector has time to yield a new goal.
            for (const float v : {1.0f, 10.0f}) {
                for (int i = 0; i < 250; ++i) {
                    const float out = sign * c.step(cfg, sign * v, dt, budget);
                    const float nominal = sign * c.feedforward();
                    assert(out <= std::min(budget, nominal + .060f) + 1e-7f);
                    assert(out >= -std::min(budget, .060f) - 1e-7f);
                    peak_assist = std::max(peak_assist, out);
                    peak_opposing = std::max(peak_opposing, -out);
                }
            }
            if (budget >= .08f) {
                assert(peak_assist > .075f);
                assert(peak_opposing > .05f);
            }
            if (budget >= .8f) {
                assert(peak_assist > .08f);
                assert(peak_opposing >= .05999f);
            }
            assert(run(c, cfg, 0, .2f, budget) == 0);
            assert(c.reference() == 0);
        }
    }
}

int main() {
    rest_and_motion_gate();
    observed_handover_and_periodic_disturbance();
    gesture_yield_and_recapture();
    reversal_and_restart();
    budgets_and_overcompensation();
    invalid_input_clears_state();
    closed_loop_loss_residual();
    temporary_budget_loss();
    periodic_motion_does_not_retarget();
    adapted_correction_budget();
    std::cout << "PASS: frictionless rest, acquisition, immutable hold, yielding, reversal, budgets and invalid input.\n";
}
