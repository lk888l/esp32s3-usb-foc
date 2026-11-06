#pragma once
#include <cstdint>
#include "sampling_timing.hpp"

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
// The two useful conversions straddle consecutive TEZ events. A nonzero
// command must preserve the carrier between them and the second TEZ latch.
// Stage 2 begins less than the commit guard before that TEZ, so direct writes
// become legal again only after it. Zero remains available for stop/recovery.
constexpr bool pwm_command_deferred(bool adc_pending, unsigned adc_stage, bool zero_command) {
    return !zero_command && adc_pending && adc_stage == 1;
}
constexpr uint32_t pwm_staggered_latch_delay_ticks(uint32_t phase, uint32_t half_period,
        bool deferred) {
    return pwm_direct_latch_delay_ticks(phase, half_period) + (deferred ? 2 * half_period : 0);
}
enum class PwmQueuedAction { wait, apply, discard };
// A predicted nonzero command belongs to exactly one future TEZ. If an ADC
// slot slips a carrier, never apply its old angle prediction at a later TEZ.
// Generations are at most two carriers apart; unsigned subtraction preserves
// ordering across uint32_t wrap without implementation-defined signed casts.
constexpr PwmQueuedAction pwm_queued_action(uint32_t next_latch_sequence, uint32_t target_sequence,
        bool pair_protected, bool zero_command) {
    if (zero_command) return PwmQueuedAction::apply;
    const uint32_t ahead = target_sequence - next_latch_sequence;
    if (ahead == 0) return pair_protected ? PwmQueuedAction::discard : PwmQueuedAction::apply;
    return ahead < 0x80000000U ? PwmQueuedAction::wait : PwmQueuedAction::discard;
}
// Reject predictions if their TEZ service state changed while calculating
// SVPWM, including ownership of the carrier between the useful ADC phases.
// The guard also prevents committing across the imminent physical TEZ.
constexpr bool pwm_schedule_still_valid(uint32_t sequence, uint32_t current_sequence,
        bool pending, bool current_pending, uint32_t phase, uint32_t current_phase,
        uint32_t period, uint32_t guard_ticks, uint32_t elapsed_us, uint32_t period_us,
        bool deferred = false, bool current_deferred = false) {
    return sequence == current_sequence && pending == current_pending && deferred == current_deferred &&
           current_phase >= phase && current_phase + guard_ticks < period && elapsed_us < period_us;
}
// Conservative common-low check: the whole conversion must remain between
// the last falling edge plus CSA settling and the first rising edge minus guard.
constexpr bool pwm_adc_window_valid(int start_tick, int completion_tick,
        uint32_t falling_compare, uint32_t rising_compare,
        uint32_t settling_ticks = sampling_timing::settling_ticks,
        uint32_t guard_ticks = sampling_timing::rising_guard_ticks) {
    return falling_compare >= settling_ticks && rising_compare > guard_ticks &&
        start_tick >= -static_cast<int>(falling_compare) + static_cast<int>(settling_ticks) &&
        completion_tick >= start_tick && completion_tick < static_cast<int>(rising_compare - guard_ticks);
}
// For the staggered adapter, START is in the configured pre-TEZ interval. A fresh ADC1_DONE must
// belong to this carrier. Counter and wall-clock reads are not atomic; allow
// five microseconds of read/metadata skew, independently of the physical PWM
// edge guards, which continue to use only the coherent hardware counter.
// The <40 us deadline also prevents accepting the same phase one cycle later.
constexpr bool adc_completion_observation_valid(unsigned slot, uint8_t done_mask,
        int start_tick, int done_tick, uint32_t start_elapsed_us, uint32_t done_elapsed_us) {
    if (slot >= 3 || !(done_mask & (1U << slot)) || start_tick < sampling_timing::start_tick || start_tick >= 0 ||
        done_elapsed_us < start_elapsed_us || done_tick < start_tick)
        return false;
    const uint32_t elapsed_us = done_elapsed_us - start_elapsed_us;
    const int elapsed_ticks = done_tick - start_tick;
    const int elapsed_error_ticks = elapsed_ticks - static_cast<int>(elapsed_us * 80);
    return elapsed_us < 40 && elapsed_error_ticks >= -400 && elapsed_error_ticks <= 400;
}
struct AdcCurrentPair {
    uint32_t words[2]{};
    uint16_t raw[2]{};
    uint8_t phase[2]{};
    bool valid = false;
};
// At ADC1_DONE, the status register's low 12 bits contain the completed value
// (as read by IDF adc_ll_adc1_read). Powered measurements show its upper channel
// bits already reflect the next preloaded mux channel. Only DMA supplies the
// result's channel tag. Require that tag to match the mux observed before this
// START, and require the ADC code to match its fresh DONE. The pre-START tag
// prevents an adjacent phase with the same ADC code from passing the join.
// START/frame ownership and the fixed scan order remain independently required.
constexpr AdcCurrentPair adc_current_pair_from_dma_done(const uint32_t dma_words[2],
        const uint32_t armed_words[3], const uint32_t done_words[3], uint8_t done_mask) {
    AdcCurrentPair pair{};
    if ((done_mask & 3) != 3) return pair;
    for (unsigned slot = 0; slot < 2; ++slot) {
        const uint32_t word = dma_words[slot];
        const unsigned channel = (word >> 13) & 0xf;
        if ((word & ~0x1efffU) != 0 || channel < 3 || channel > 5 ||
            channel != ((armed_words[slot] >> 13) & 0xf) ||
            (word & 0xfff) != (done_words[slot] & 0xfff)) return pair;
        pair.words[slot] = word;
        pair.raw[slot] = word & 0xfff;
        pair.phase[slot] = channel - 3;
    }
    pair.valid = pair.phase[0] != pair.phase[1];
    return pair;
}
constexpr bool adc_stage_matches(unsigned launched, unsigned received) {
    return launched >= 1 && launched <= 3 && launched == received + 1;
}
constexpr bool adc_next_stage_ready(unsigned launched, unsigned received, unsigned requested, uint8_t completed_mask) {
    return launched >= 1 && launched < 3 && received == launched && requested > launched &&
        (completed_mask & (1U << (launched - 1))) != 0;
}
constexpr bool adc_staggered_timing_valid(uint32_t second_start_us, uint32_t third_start_us,
        uint32_t completion_us, uint32_t carrier_ticks = 4000) {
    // Compare in timer ticks so 15 kHz's 66.65 us carrier is not rounded to
    // 66/67 us before accumulating the second interval. Timestamp skew is
    // independent of the physical falling/rising edge checks.
    constexpr uint32_t ticks_per_us = sampling_timing::resolution_hz / 1000000;
    constexpr uint32_t skew = 5 * ticks_per_us;
    const uint64_t second = static_cast<uint64_t>(second_start_us) * ticks_per_us;
    const uint64_t third = static_cast<uint64_t>(third_start_us) * ticks_per_us;
    return second + skew >= carrier_ticks && second <= carrier_ticks + skew &&
        third + skew >= 2 * carrier_ticks && third <= 2 * carrier_ticks + skew &&
        completion_us >= third_start_us &&
        static_cast<uint64_t>(completion_us) * ticks_per_us < 2 * carrier_ticks + 20 * ticks_per_us;
}
constexpr bool pwm_adc_pair_coherent(const uint16_t before[3][3], const uint16_t after[3][3]) {
    // The first TEZ may latch a new command while the measured phase remains
    // LOW throughout its independently checked before/after ADC window. The
    // old before[0] command does not drive the carrier between the two samples.
    // Keep that intervening carrier and the second sample's command coherent.
    for (int p = 0; p < 3; ++p)
        if (after[0][p] != before[1][p] || before[1][p] != after[1][p]) return false;
    return true;
}
// DRV8316C senses a phase whenever its own low-side conducts. The third,
// reconstructed phase may be high (including continuously high at full duty).
constexpr bool pwm_adc_phase_window_valid(unsigned phase, int start, int end,
        const uint16_t before[3], const uint16_t after[3]) {
    return phase < 3 && pwm_adc_window_valid(start, end, before[phase], after[phase]);
}
// S3 continuous force has a TEZ shadow latch, just like the comparators.
// Compare=0 needs force HIGH; compare=peak needs force LOW. Interior values
// disable continuous force. Hardware applies all three phases at the boundary.
constexpr uint32_t pwm_force_shadow_bits(uint32_t compare, uint32_t half_period) {
    return 1U | ((compare == 0 ? 2U : compare == half_period ? 1U : 0U) << 6);
}
// Compare at the peak still needs a software force-low: the generator's UP
// compare action otherwise creates a narrow pulse around the peak.
constexpr bool pwm_zero_command(float a, float b, float c) { return a == 0 && b == 0 && c == 0; }
constexpr uint8_t zero_duty_mask(float a, float b, float c) {
    return (a == 0 ? 1U : 0U) | (b == 0 ? 2U : 0U) | (c == 0 ? 4U : 0U);
}
}
