#pragma once
#include <cstdint>

namespace knob {
// ESP32-S3 TIMER_STATUS contains the next count, not the current count.
// Decode one register read so count and direction cannot straddle a reversal.
struct PwmCounter {
    uint16_t count = 0;
    bool down = false;
    constexpr int16_t signed_tick() const { return down ? -static_cast<int16_t>(count) : count; }
    constexpr uint32_t phase_ticks(uint32_t half_period) const {
        return down ? 2 * half_period - count : count;
    }
};
constexpr PwmCounter decode_pwm_counter(uint32_t status, uint32_t half_period) {
    const bool down = (status & (1U << 16)) != 0;
    const uint32_t next = status & 0xffffU;
    return {static_cast<uint16_t>((next + (down ? 1 : half_period)) % (half_period + 1)), down};
}
// A pending TEZ ISR will write the shadow immediately; otherwise it writes at
// the next TEZ. The hardware then latches that shadow at the following TEZ.
constexpr uint32_t pwm_latch_delay_ticks(uint32_t phase, bool tez_pending, uint32_t half_period) {
    const uint32_t period = 2 * half_period;
    return period - phase + (tez_pending ? 0 : period);
}
constexpr uint32_t pwm_direct_latch_delay_ticks(uint32_t phase, uint32_t half_period) {
    return 2 * half_period - phase;
}
// Reject predictions if their TEZ service state changed while calculating
// SVPWM. The guard also prevents committing across the imminent physical TEZ.
constexpr bool pwm_schedule_still_valid(uint32_t sequence, uint32_t current_sequence,
        bool pending, bool current_pending, uint32_t phase, uint32_t current_phase,
        uint32_t period, uint32_t guard_ticks, uint32_t elapsed_us, uint32_t period_us) {
    return sequence == current_sequence && pending == current_pending &&
           current_phase >= phase && current_phase + guard_ticks < period && elapsed_us < period_us;
}
// Conservative common-low check: the whole conversion must remain between
// the last falling edge plus CSA settling and the first rising edge minus guard.
constexpr bool pwm_adc_window_valid(int start_tick, int completion_tick,
        uint32_t falling_compare, uint32_t rising_compare,
        uint32_t settling_ticks = 480, uint32_t guard_ticks = 80) {
    return falling_compare >= settling_ticks && rising_compare > guard_ticks &&
        start_tick >= -static_cast<int>(falling_compare) + static_cast<int>(settling_ticks) &&
        completion_tick >= start_tick && completion_tick < static_cast<int>(rising_compare - guard_ticks);
}
constexpr bool adc_stage_matches(unsigned launched, unsigned received) {
    return launched >= 1 && launched <= 3 && launched == received + 1;
}
constexpr bool adc_next_stage_ready(unsigned launched, unsigned received, unsigned requested, uint8_t completed_mask) {
    return launched >= 1 && launched < 3 && received == launched && requested > launched &&
        (completed_mask & (1U << (launched - 1))) != 0;
}
constexpr bool adc_staggered_timing_valid(uint32_t second_start_us, uint32_t third_start_us,
        uint32_t completion_us) {
    return second_start_us >= 45 && second_start_us <= 55 &&
        third_start_us >= 95 && third_start_us <= 105 && completion_us < 120;
}
constexpr bool pwm_adc_pair_coherent(const uint16_t before[3][3], const uint16_t after[3][3]) {
    for (int p = 0; p < 3; ++p)
        if (before[0][p] != after[0][p] || after[0][p] != before[1][p] || before[1][p] != after[1][p]) return false;
    return true;
}
// Compare at the peak still needs a software force-low: the generator's UP
// compare action otherwise creates a narrow pulse around the peak.
constexpr bool pwm_zero_command(float a, float b, float c) { return a == 0 && b == 0 && c == 0; }
constexpr uint8_t zero_duty_mask(float a, float b, float c) {
    return (a == 0 ? 1U : 0U) | (b == 0 ? 2U : 0U) | (c == 0 ? 4U : 0U);
}
// Follow the same pending -> hardware shadow -> TEZ latch lifecycle as the
// compare values. A newer queued command cannot alter an already staged mask.
struct PwmForceLowState {
    uint8_t pending = 7, shadow = 7, active = 7;
    constexpr void queue(uint8_t mask) { pending = mask & 7U; }
    constexpr void commit_shadow() { shadow = pending; }
    constexpr uint8_t latch() {
        const uint8_t changed = active ^ shadow;
        active = shadow;
        return changed;
    }
};
// Force-low controls update immediately, unlike comparator shadows. Every
// changed phase must be updated before its new rising edge (or before the
// peak for a zero-duty phase), including the reserved register-write margin.
constexpr bool pwm_force_update_fits(PwmCounter counter, uint8_t changed_mask,
        const uint32_t compare[3], uint32_t operation_margin_ticks) {
    if (!changed_mask) return true;
    if (counter.down) return false;
    for (int p = 0; p < 3; ++p) {
        if ((changed_mask & (1U << p)) &&
            (compare[p] <= counter.count || compare[p] - counter.count <= operation_margin_ticks)) return false;
    }
    return true;
}
}
