#include "control.hpp"
#include "control_timing.hpp"
#include "pwm_timing.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>

using namespace knob;

int main() {
    constexpr uint32_t ticks_per_us = sampling_timing::resolution_hz / 1000000;
    constexpr uint32_t half_period = sampling_timing::resolution_hz / (2 * pwm_profile_hz);
    constexpr uint32_t compare = sampling_timing::minimum_compare_ticks;
    constexpr int start = sampling_timing::start_tick;
    constexpr int done = sampling_timing::completion_budget_tick;
    static_assert(pwm_profile_hz == 15000 && half_period == 2666);
    static_assert(sampling_timing::resolution_hz / (2 * half_period) >= 15000);
    static_assert(sampling_timing::settling_ticks == 6 * ticks_per_us);
    static_assert(sampling_timing::rising_guard_ticks == ticks_per_us);
    static_assert(sampling_timing::adc_sar_hz == 5000000);
    static_assert(sampling_timing::adc_conversion_clocks *
        (sampling_timing::resolution_hz / sampling_timing::adc_sar_hz) == 5 * ticks_per_us);
#if defined(CONFIG_KNOB_ADC_FAST_TIMING) && CONFIG_KNOB_ADC_FAST_TIMING
    static_assert(sampling_timing::adc_controller_hz == 40000000);
    static_assert(compare == 569 + sampling_timing::reserve_ticks);
#if defined(CONFIG_KNOB_ADC_RESERVE_TICKS) && CONFIG_KNOB_ADC_RESERVE_TICKS == 0
    static_assert(compare == 569 && sampling_timing::reserve_ticks == 0);
    assert(std::abs(duty_ceiling - .78657164f) < 1e-7f);
#elif defined(CONFIG_KNOB_ADC_RESERVE_TICKS) && CONFIG_KNOB_ADC_RESERVE_TICKS == 8
    static_assert(compare == 577 && sampling_timing::reserve_ticks == 8);
    assert(std::abs(duty_ceiling - .78357089f) < 1e-7f);
#else
    static_assert(compare == 600 && 2 * compare == 15 * ticks_per_us);
    assert(std::abs(duty_ceiling - .77494374f) < 1e-7f);
#endif
    static_assert(start == -80 && done == 488);
#else
    static_assert(sampling_timing::adc_controller_hz == 10000000);
    static_assert(compare == 700 && 4 * compare == 35 * ticks_per_us);
    static_assert(start == -180 && done == 560);
    assert(std::abs(duty_ceiling - .73743436f) < 1e-7f);
#endif

    // The contractual envelope plus its reserve fits at maximum duty. Every
    // possible measured pair must pass, including two high-duty phases.
    const uint16_t boundary[3] = {compare, compare, compare};
    for (unsigned phase = 0; phase < 3; ++phase) {
        assert(pwm_adc_phase_window_valid(phase, start, done, boundary, boundary));
        assert(pwm_adc_phase_window_valid(phase,
            start - static_cast<int>(sampling_timing::reserve_ticks),
            done + sampling_timing::reserve_ticks, boundary, boundary));
    }
    // These are physical edge limits, independent of the scheduled START.
    // Exactly six microseconds of settling is accepted; the rising guard is
    // strict, so completion exactly one microsecond before the edge fails.
    constexpr int earliest = static_cast<int>(sampling_timing::settling_ticks) - static_cast<int>(compare);
    constexpr int latest = compare - sampling_timing::rising_guard_ticks - 1;
    assert(pwm_adc_window_valid(earliest, latest, compare, compare));
    assert(!pwm_adc_window_valid(earliest - 1, latest, compare, compare));
    assert(!pwm_adc_window_valid(earliest, latest + 1, compare, compare));
    assert(!pwm_adc_window_valid(earliest, latest, compare - 1, compare));
    assert(!pwm_adc_window_valid(earliest, latest, compare, compare - 1));
    assert(!pwm_adc_window_valid(start, start - 1, compare, compare));

    constexpr uint32_t elapsed_us = (done - start + ticks_per_us / 2) / ticks_per_us;
    assert(adc_completion_observation_valid(0, 1, start, done, 0, elapsed_us));
    assert(!adc_completion_observation_valid(0, 0, start, done, 0, elapsed_us));
    assert(!adc_completion_observation_valid(0, 1, start - 1, done, 0, elapsed_us));
    assert(!adc_completion_observation_valid(0, 1, start, done, 0, elapsed_us + 67));

    // PWM quantization must preserve the real low-side window through every
    // electrical angle at the linear voltage circle, not just its six peaks.
    for (float bus : {4.5f, 12.0f, 19.4f, 30.0f}) {
        const float radius = max_output_voltage(bus);
        assert(std::abs(radius / (bus / std::sqrt(3.0f)) - duty_ceiling) < 1e-6f);
        for (int step = 0; step < 7200; ++step) {
            const auto duty = svpwm({radius, 0}, step * tau / 7200, bus);
            for (float phase : duty) {
                const auto quantized = static_cast<uint32_t>(half_period * (1 - phase) + .5f);
                assert(quantized >= compare && quantized <= half_period);
                assert(pwm_adc_window_valid(start, done, quantized, quantized));
            }
        }
    }

    // Four carriers at 15 kHz produce alternating 266/267 us timestamps.
    // PI integration follows that measured time rather than retaining a
    // hard-coded 200 us step or accumulating a rounded 267 us each cycle.
    ControlSampleClock clock;
    Pi pi;
    constexpr int64_t epoch = 1000000;
    constexpr uint32_t control_ticks = 4 * 2 * half_period;
    constexpr uint32_t nominal_us = (control_ticks + ticks_per_us - 1) / ticks_per_us;
    static_assert(nominal_us == 267);
    assert(std::abs(clock.accept(epoch, nominal_us).integration_s - .000267f) < 1e-9f);
    float integrated = 0;
    for (int64_t frame = 1; frame <= 3750; ++frame) {
        const auto interval = clock.accept(epoch + frame * control_ticks / ticks_per_us, nominal_us);
        assert(interval.elapsed_s >= .000266f && interval.elapsed_s <= .000267f);
        integrated = pi.step(1, interval.integration_s, 0, 2, 100);
    }
    const float expected = 2.0f * (3750LL * control_ticks / ticks_per_us) * 1e-6f;
    assert(std::abs(integrated - expected) < 1e-4f);
}
