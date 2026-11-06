#include "calibration_sequence.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

using namespace knob;

namespace {
bool error_is(const CalibrationSequence& sequence, const char* text) {
    return sequence.report().error && std::strcmp(sequence.report().error, text) == 0;
}

struct Simulation {
    CalibrationSequence sequence;
    CalibrationPlan plan{};
    int64_t now = 1000;
    unsigned samples = 0;
    int direction = 1;
    float pole_pairs = 7, electrical_zero = 6.25f;
    float tracking_ratio = 1, tracking_q = 0;
    float phase_resistance = 8;
    bool saturated = false, missing_phase = false, missing_primary = false, swapped_phases = false;
    bool verify_corruption = false, mechanical_jitter = false;
    bool retry_hysteresis = false;
    unsigned jitter_attempts = 0;
    float commanded_peak_current = 0;

    explicit Simulation(CalibrationPlan requested = {}) : plan(requested) {
        assert(sequence.start(plan, now));
    }

    CalibrationSequenceInput input() {
        const auto command = sequence.command();
        const auto& report = sequence.report();
        CalibrationSequenceInput value;
        value.now_us = now;
        value.available_voltage_limit = plan.voltage_limit;
        value.rotor_angle = direction * (command.angle + electrical_zero) / pole_pairs;
        if ((mechanical_jitter || report.attempts <= jitter_attempts) && report.stage == CalibrationStage::align)
            value.rotor_angle += (samples % 2 ? .006f : -.006f);
        if (retry_hysteresis && report.attempts == 1) {
            if (report.stage == CalibrationStage::forward) value.rotor_angle += direction * .10f / pole_pairs;
            if (report.stage == CalibrationStage::reverse) value.rotor_angle -= direction * .10f / pole_pairs;
        }
        if (verify_corruption && report.stage == CalibrationStage::verify)
            value.rotor_angle += direction * .25f / pole_pairs;
        const float d = command.voltage_mode ? command.voltage_d / phase_resistance : command.current * tracking_ratio;
        const float q = command.voltage_mode ? 0 : tracking_q;
        const float alpha = d * std::cos(command.angle) - q * std::sin(command.angle);
        const float beta = d * std::sin(command.angle) + q * std::cos(command.angle);
        value.currents = {alpha, -.5f * alpha + .8660254038f * beta, -.5f * alpha - .8660254038f * beta};
        if (swapped_phases) std::swap(value.currents[0], value.currents[1]);
        value.measured_phases = missing_primary ? std::array<int, 2>{1, 2} :
            missing_phase ? std::array<int, 2>{0, 1} :
            std::array<int, 2>{static_cast<int>(samples % 3), static_cast<int>((samples + 1) % 3)};
        value.output_voltage = command.voltage_mode ? command.voltage_d :
            saturated ? plan.voltage_limit : 8.0f * d;
        return value;
    }

    void step(int interval_us = 1000) {
        now += interval_us;
        ++samples;
        const auto before = sequence.command();
        commanded_peak_current = std::max(commanded_peak_current, before.current);
        assert(before.current <= plan.max_current + .000001f);
        assert(before.voltage_d <= std::min(.5f, plan.voltage_limit) + .000001f);
        sequence.tick(input());
        if (!sequence.report().passed) assert(!sequence.candidate().valid);
    }

    void run() {
        while (sequence.running() && samples < 150000) step();
        assert(!sequence.running());
        assert(sequence.command().done);
        assert(sequence.command().current == 0 && sequence.command().voltage_d == 0);
    }

    void until(CalibrationStage target) {
        while (sequence.running() && sequence.report().stage != target && samples < 150000) step();
        assert(sequence.report().stage == target);
    }
};
}

int main() {
    // The actual ADC samples always correspond to the PREVIOUS command.
    // Both encoder directions and a zero near 2*pi must produce a valid
    // seven-pole fit and six independent held-out alignments.
    for (int direction : {-1, 1}) {
        Simulation sim;
        sim.direction = direction;
        sim.run();
        const auto& report = sim.sequence.report();
        if (!report.passed) std::cerr << "ideal failure: " << (report.error ? report.error : "none") << '\n';
        assert(report.passed && report.stage == CalibrationStage::complete);
        assert(report.phase_check_passed && report.attempts == 1);
        assert(report.forward_points == 29 && report.reverse_points == 29 && report.verify_points == 6);
        assert(report.verification_max < .002f);
        const auto candidate = sim.sequence.candidate();
        assert(candidate.valid && candidate.direction == direction && candidate.pole_pairs == 7);
        assert(std::abs(wrap_signed(candidate.electrical_zero - sim.electrical_zero)) < .002f);
        for (unsigned vector = 0; vector < 3; ++vector) {
            for (unsigned phase = 0; phase < 3; ++phase) assert(report.phase_samples[vector][phase] >= 256);
            assert(report.phase_currents[vector][vector] > .05f);
            assert(report.phase_currents[vector][vector] <= sim.plan.max_current);
        }
    }

    // The old single-cycle tolerance admitted six and eight pole pairs.
    // Full-scan fitting rejects both, without increasing excitation.
    for (float poles : {6.0f, 8.0f}) {
        Simulation sim;
        sim.pole_pairs = poles;
        sim.run();
        assert(!sim.sequence.report().passed && sim.sequence.report().attempts == 1);
        assert(sim.sequence.report().error);
        assert(sim.sequence.report().fit_result.pole_pairs > poles - .01f &&
               sim.sequence.report().fit_result.pole_pairs < poles + .01f);
        assert(!sim.sequence.candidate().valid);
    }

    // Reconstructed third-phase values never substitute for physically
    // observing that phase, even when the synthetic current sum is perfect.
    {
        Simulation sim;
        sim.missing_phase = true;
        sim.run();
        assert(error_is(sim.sequence, "calibration_phase_feedback_stale"));
        assert(!sim.sequence.report().phase_check_passed);
    }
    // A missing primary phase must keep the probe de-energized. Its two
    // half-sized currents otherwise allow the unseen primary to exceed the
    // requested current cap while the phase-coverage check waits.
    {
        Simulation sim({.04f, .04f, 2});
        sim.missing_primary = true;
        while (sim.sequence.running()) {
            assert(sim.sequence.command().voltage_d == 0);
            sim.step();
        }
        assert(error_is(sim.sequence, "calibration_phase_feedback_stale"));
        assert(sim.sequence.command().voltage_mode && sim.sequence.command().voltage_d == 0);
        assert(sim.now < 10000);
    }
    // Losing the highest-current channel after the ramp has started must
    // stop within its own freshness budget, even if other ADC data continues.
    {
        Simulation sim({.04f, .04f, 2});
        while (sim.now < 100000) sim.step();
        assert(sim.sequence.running() && sim.sequence.command().voltage_d > 0);
        sim.missing_primary = true;
        const auto lost_at = sim.now;
        while (sim.sequence.running()) sim.step();
        assert(error_is(sim.sequence, "calibration_phase_feedback_stale"));
        assert(sim.now - lost_at <= 3000);
        assert(sim.sequence.command().voltage_mode && sim.sequence.command().voltage_d == 0);
    }
    {
        Simulation sim;
        while (sim.now < 100000) sim.step();
        sim.sequence.check_timeout(sim.now + 2001);
        assert(error_is(sim.sequence, "calibration_phase_feedback_stale"));
        assert(sim.sequence.command().voltage_mode && sim.sequence.command().voltage_d == 0);
    }
    {
        Simulation sim;
        sim.swapped_phases = true;
        sim.run();
        assert(error_is(sim.sequence, "calibration_phase_excitation"));
        assert(sim.sequence.report().attempts == 1);
    }
    {
        Simulation sim({.08f, .16f, .05f});
        sim.run();
        assert(error_is(sim.sequence, "calibration_phase_excitation"));
    }
    // A small requested budget cannot be bypassed by the voltage-mode phase
    // probe. Lower resistance must lower its probe voltage or cause a stop.
    {
        Simulation sim({.02f, .02f, 2});
        sim.phase_resistance = 1;
        sim.run();
        assert(!sim.sequence.report().passed && sim.sequence.report().attempts == 1);
        assert(error_is(sim.sequence, "calibration_phase_excitation") ||
               error_is(sim.sequence, "calibration_phase_current_high"));
    }
    {
        Simulation sim({.08f, .08f, 2});
        sim.phase_resistance = .25f;
        sim.run();
        if (sim.sequence.report().phase_check_passed) {
            for (unsigned p = 0; p < 3; ++p)
                assert(sim.sequence.report().phase_currents[p][p] <= sim.plan.max_current);
        } else assert(error_is(sim.sequence, "calibration_phase_current_high"));
    }

    // Sustained under-current cannot pass through the previous 35% gap.
    // A saturated driver and a failed current loop are reported distinctly.
    for (bool saturated : {false, true}) {
        Simulation sim;
        sim.tracking_ratio = .70f;
        sim.saturated = saturated;
        sim.run();
        assert(error_is(sim.sequence, saturated ? "calibration_voltage_saturated" : "calibration_current_tracking"));
        assert(sim.sequence.report().attempts == 1);
    }
    {
        Simulation sim;
        sim.tracking_q = .025f;
        sim.run();
        assert(error_is(sim.sequence, "calibration_current_tracking"));
    }
    {
        Simulation sim;
        sim.until(CalibrationStage::align);
        const auto aligned_at = sim.now;
        while (sim.sequence.running() && sim.now - aligned_at < 750000) {
            sim.tracking_ratio = sim.now - aligned_at < 700000 ? .70f : 1.0f;
            sim.step();
        }
        sim.tracking_ratio = 1;
        sim.run();
        assert(sim.sequence.report().passed && sim.sequence.report().attempts == 1);
    }
    {
        Simulation sim;
        sim.tracking_ratio = .70f;
        sim.mechanical_jitter = true;
        sim.run();
        assert(error_is(sim.sequence, "calibration_current_tracking"));
        assert(sim.sequence.report().attempts == 1);
    }

    // Mechanical instability can trigger a bounded current increase; an
    // always-moving rotor reaches the attempt cap without infinite retries.
    {
        Simulation sim({.08f, .16f, 2.0f});
        sim.jitter_attempts = 1;
        sim.run();
        assert(sim.sequence.report().passed && sim.sequence.report().attempts == 2);
        assert(std::abs(sim.sequence.report().current - .12f) < .000001f);
    }
    {
        Simulation sim({.08f, .16f, 2.0f});
        sim.mechanical_jitter = true;
        sim.run();
        assert(error_is(sim.sequence, "calibration_alignment_unstable"));
        assert(sim.sequence.report().attempts == 3 && sim.sequence.report().current == .16f);
    }
    {
        Simulation sim;
        sim.retry_hysteresis = true;
        sim.run();
        assert(sim.sequence.report().passed && sim.sequence.report().attempts == 2);
        assert(sim.sequence.report().forward_points == 29 && sim.sequence.report().reverse_points == 29);
    }

    // New held-out data must be able to reject an apparently perfect fit.
    {
        Simulation sim;
        sim.verify_corruption = true;
        sim.run();
        assert(error_is(sim.sequence, "calibration_verification"));
        assert(sim.sequence.report().fit_result.residual_rms < .002f);
        assert(sim.sequence.report().verification_max > .24f);
        assert(sim.sequence.report().verify_points == 0 && !sim.sequence.candidate().valid);
    }

    // A single rejected frame clears the qualification window without
    // restarting the trajectory/current ramp or granting more point time.
    {
        Simulation sim;
        sim.until(CalibrationStage::forward);
        for (unsigned n = 0; n < 200; ++n) sim.step();
        const auto before = sim.sequence.command();
        sim.sequence.invalidate(sim.now + 500);
        assert(sim.sequence.command().current == before.current);
        assert(sim.sequence.command().angle == before.angle);
        sim.run();
        assert(sim.sequence.report().passed);
    }
    // Explicit feedback recovery removes drive and requires a fresh ramp;
    // the old average cannot satisfy a point immediately after recovery.
    {
        Simulation sim;
        sim.until(CalibrationStage::forward);
        for (unsigned n = 0; n < 200; ++n) sim.step();
        sim.sequence.invalidate(sim.now + 500, true);
        assert(sim.sequence.command().current == 0 && sim.sequence.command().reset_controller);
        assert(sim.sequence.command().voltage_mode && sim.sequence.command().voltage_d == 0);
        const unsigned points = sim.sequence.report().forward_points;
        sim.step();
        assert(sim.sequence.command().current == 0);
        for (unsigned n = 0; n < 100; ++n) sim.step();
        assert(sim.sequence.report().forward_points == points);
        sim.run();
        assert(sim.sequence.report().passed);
    }
    {
        Simulation sim;
        sim.until(CalibrationStage::align);
        const auto& command = sim.sequence.invalidate(sim.now + 2001);
        assert(command.current == 0 && command.reset_controller);
        sim.sequence.check_timeout(sim.now + 10001);
        assert(error_is(sim.sequence, "calibration_feedback_stale"));
        assert(sim.sequence.command().done && sim.sequence.command().current == 0);
    }
    // Repeated invalidation can never fabricate a stable window from wall
    // time. Fresh frames keep the feedback alive, while the point expires.
    {
        Simulation sim;
        sim.until(CalibrationStage::forward);
        while (sim.sequence.running()) {
            sim.sequence.invalidate(sim.now + 500);
            sim.step();
        }
        assert(error_is(sim.sequence, "calibration_point_timeout"));
        assert(sim.sequence.report().forward_points == 0);
    }

    // The overall deadline and cancellation operate without further ADC
    // input, preserve terminal state, and never expose a candidate.
    {
        Simulation sim;
        sim.sequence.check_timeout(sim.now + CalibrationSequence::maximum_duration_us);
        assert(error_is(sim.sequence, "calibration_timeout"));
        assert(sim.sequence.command().done && !sim.sequence.candidate().valid);
    }
    {
        Simulation sim;
        sim.until(CalibrationStage::forward);
        sim.sequence.abort();
        assert(sim.sequence.report().cancelled && !sim.sequence.report().passed);
        assert(sim.sequence.command().done && sim.sequence.command().current == 0);
        assert(!sim.sequence.candidate().valid);
        sim.step();
        assert(sim.sequence.report().stage == CalibrationStage::cancelled);
    }

    // Malformed plans and sample identity/values fail before acceptance.
    {
        CalibrationSequence sequence;
        assert(!sequence.start({.08f, .04f, 2}, 0));
        assert(error_is(sequence, "calibration_current_range"));
        assert(!sequence.start({.08f, .16f, 3.1f}, 0));
        assert(error_is(sequence, "calibration_voltage_range"));
        assert(!sequence.start({std::numeric_limits<float>::quiet_NaN(), .16f, 2}, 0));
        assert(error_is(sequence, "calibration_plan_nonfinite"));
    }
    {
        Simulation sim;
        auto input = sim.input();
        input.now_us += 1000;
        input.measured_phases = {1, 1};
        sim.sequence.tick(input);
        assert(error_is(sim.sequence, "calibration_phase_sample_invalid"));
    }
    {
        Simulation sim;
        auto input = sim.input();
        input.now_us += 1000;
        input.currents[2] = std::numeric_limits<float>::quiet_NaN();
        sim.sequence.tick(input);
        assert(error_is(sim.sequence, "calibration_sample_invalid"));
    }
    {
        Simulation sim;
        sim.step();
        sim.sequence.tick(sim.input());
        assert(error_is(sim.sequence, "calibration_sample_order"));
    }
    std::cout << "PASS: bounded measured-phase checks, bidirectional calibration, independent verification, adaptive current, feedback gaps and terminal safety.\n";
}
