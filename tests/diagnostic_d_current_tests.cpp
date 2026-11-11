#include "diagnostics.hpp"
#include <cassert>
#include <cmath>
#include <limits>
using namespace knob;

int main() {
    DiagnosticPlan plan{Mode::free, 0, 3, 30000000, 1000000, DiagnosticAxis::d};
    assert(valid_diagnostic_plan(plan, 3));
    assert(!valid_diagnostic_plan(plan, 2.99f));
    plan.velocity = 10;
    assert(!valid_diagnostic_plan(plan, 3));
    plan.velocity = 0;
    plan.current = 3.001f;
    assert(!valid_diagnostic_plan(plan, 4.5f));
    plan.current = -.1f;
    assert(!valid_diagnostic_plan(plan, 3));
    plan.current = .049f;
    assert(!valid_diagnostic_plan(plan, 3));
    plan.current = .05f;
    assert(valid_diagnostic_plan(plan, 3));
    plan.drive_us = 30000001;
    assert(!valid_diagnostic_plan(plan, 3));
    plan.drive_us = 49999;
    assert(!valid_diagnostic_plan(plan, 3));
    plan.drive_us = 50000; plan.observe_us = 100000;
    assert(valid_diagnostic_plan(plan, 3));
    plan.observe_us = 99999;
    assert(!valid_diagnostic_plan(plan, 3));
    plan.observe_us = 1000001;
    assert(!valid_diagnostic_plan(plan, 3));
    plan = {Mode::free, 0, 3, 300000, 500000, DiagnosticAxis::d};

    // A short run cannot bypass the fixed 200 ms smooth ramp. The requested
    // current is not a claim that the plateau was reached before cutoff.
    assert(diagnostic_d_target(plan, -1) == 0);
    assert(diagnostic_d_target(plan, 0) == 0);
    assert(std::abs(diagnostic_d_target(plan, 100000) - 1.5f) < 1e-6f);
    assert(diagnostic_d_target(plan, 200000) == 3);
    assert(diagnostic_d_target(plan, 299999) == 3);
    assert(diagnostic_d_target(plan, 300000) == 0);
    auto short_plan = plan; short_plan.drive_us = 50000;
    assert(diagnostic_d_target(short_plan, 49999) < .47f);
    assert(diagnostic_d_target(short_plan, 50000) == 0);

    using Action = DiagnosticDAction;
    assert(diagnostic_d_action(plan, 299999, 0, 0) == Action::drive);
    assert(diagnostic_d_action(plan, 300000, 0, 0) == Action::release);
    assert(diagnostic_d_action(plan, 799999, 0, 0) == Action::release);
    assert(diagnostic_d_action(plan, 800000, 0, 0) == Action::complete);
    // No ADC/control step is needed between these observations: an arbitrarily
    // missing burst still reaches the fixed stop, rather than extending a run.
    assert(diagnostic_d_action(plan, 900000, 0, 0) == Action::complete);
    assert(diagnostic_d_action(plan, 100, .15f, 5) == Action::drive);
    assert(diagnostic_d_action(plan, 100, .1501f, 0) == Action::motion_fault);
    assert(diagnostic_d_action(plan, 100, -.1501f, 0) == Action::motion_fault);
    assert(diagnostic_d_action(plan, 100, 0, 5.01f) == Action::motion_fault);
    assert(diagnostic_d_action(plan, 100, 0, -5.01f) == Action::motion_fault);
    assert(diagnostic_d_action(plan, 100, 0, std::numeric_limits<float>::quiet_NaN()) == Action::motion_fault);

    CurrentLimiter limiter;
    limiter.reset(3);
    // Intended Id consumes the regulated axis, not the orthogonal reserve.
    assert(std::abs(limit_diagnostic_d_current(limiter, 3, {2.5f, 0}, 2.5f, 3, .0002f) - 3) < 1e-6f);
    limiter.reset(3);
    assert(std::abs(limit_diagnostic_d_current(limiter, 3, {0, 2}, 2, 3, .0002f) - std::sqrt(5.0f)) < 1e-6f);
    // Genuine measured excess retains the same foldback as ordinary Iq control.
    limiter.reset(3);
    assert(limit_diagnostic_d_current(limiter, 3, {3.5f, 0}, 3.5f, 3, .0002f) < 3);
    assert(limit_diagnostic_d_current(limiter, 0, {2, 0}, 2, 3, .0002f) == 0);

    // The complete longest run, including release, fits the fixed trace buffer.
    plan.drive_us = 30000000; plan.observe_us = 1000000;
    const int period = diagnostic_capture_period_us(plan);
    assert(period >= 200);
    assert((plan.drive_us + plan.observe_us - 1) / period + 1 <= diagnostic_capacity);
    DiagnosticPlan legacy;
    assert(valid_diagnostic_plan(legacy, .08f)); // Existing q stimulus is still limited by the live current budget.
    assert(legacy.axis == DiagnosticAxis::q);
    DiagnosticPlan hold{Mode::frictionless, 1, .1f, 3000000, 10000000};
    assert(valid_diagnostic_plan(hold, .1f));
    const int hold_period = diagnostic_capture_period_us(hold);
    assert((hold.drive_us + hold.observe_us - 1) / hold_period + 1 <= diagnostic_capacity);
    hold.drive_us++; assert(!valid_diagnostic_plan(hold, .1f)); hold.drive_us--;
    hold.observe_us++; assert(!valid_diagnostic_plan(hold, .1f)); hold.observe_us--;
    hold.velocity = .999f; assert(!valid_diagnostic_plan(hold, .1f));
    hold.velocity = -1; assert(valid_diagnostic_plan(hold, .1f));
    hold.mode = static_cast<Mode>(mode_count); assert(!valid_diagnostic_plan(hold, .1f));
}
