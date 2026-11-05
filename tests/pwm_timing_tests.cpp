#include "pwm_timing.hpp"
#include <cassert>
using namespace knob;
int main() {
    // The default 20 kHz acquisition envelope must fit every rotating pair,
    // including two phases simultaneously at the maximum permitted duty.
    // Exercise real window predicates, not just the ceiling calculation.
    constexpr uint32_t sampling_half_period = 2000;
    constexpr auto sampling_compare = sampling_timing::minimum_compare_ticks;
    static_assert(sampling_compare == 700);
    static_assert(sampling_timing::duty_ceiling(sampling_half_period) == .65f);
    static_assert(sampling_timing::duty_ceiling(sampling_compare) == 0);
    const uint16_t constrained[3] = {sampling_compare, sampling_compare, sampling_compare};
    for (unsigned first = 0; first < 3; ++first) {
        const unsigned second = (first + 1) % 3;
        assert(pwm_adc_phase_window_valid(first, sampling_timing::start_tick,
            sampling_timing::completion_budget_tick, constrained, constrained));
        assert(pwm_adc_phase_window_valid(second, sampling_timing::start_tick,
            sampling_timing::completion_budget_tick, constrained, constrained));
    }
    // The added reserve is real time on both ends of the acquisition envelope.
    assert(pwm_adc_window_valid(sampling_timing::start_tick - static_cast<int>(sampling_timing::reserve_ticks),
        sampling_timing::completion_budget_tick + sampling_timing::reserve_ticks,
        sampling_compare, sampling_compare));
    assert(!pwm_adc_window_valid(-221, 560, sampling_compare, sampling_compare));
    assert(pwm_adc_window_valid(-220, 619, sampling_compare, sampling_compare));
    assert(!pwm_adc_window_valid(-220, 620, sampling_compare, sampling_compare));
    // A real shadow transition may shorten either edge independently: neither
    // a known command nor a complete DMA frame makes a short window valid.
    assert(!pwm_adc_window_valid(-180, 560, 659, sampling_compare));
    assert(!pwm_adc_window_valid(-180, 560, sampling_compare, 640));
    // 90% gives only +/-2.5 us LOW. It cannot contain this acquisition, even
    // with the former -1.4 us START; lowering 100% to 90% alone cannot fix it.
    assert(!pwm_adc_window_valid(-112, 560, 200, 200));
    assert(!pwm_adc_window_valid(sampling_timing::start_tick,
        sampling_timing::completion_budget_tick, 200, 200));
    // At the 90% circle's sector boundary, [0, .7794, .7794] duty leaves only
    // one measurable phase. Merely choosing the two lowest duties is insufficient.
    const uint16_t sector_boundary[3] = {2000, 441, 441};
    assert(pwm_adc_phase_window_valid(0, -112, 560, sector_boundary, sector_boundary));
    assert(!pwm_adc_phase_window_valid(1, -112, 560, sector_boundary, sector_boundary));
    assert(!pwm_adc_phase_window_valid(2, -112, 560, sector_boundary, sector_boundary));
    // GDMA notification may be delayed by the 1 ms RTOS tick while the ADC
    // conversion itself finished inside the physical window. A fresh DONE
    // observation proves completion; the callback remains transport evidence.
    assert(adc_completion_observation_valid(0, 1, -180, 400, 0, 7));
    assert(pwm_adc_phase_window_valid(0, -180, 400, constrained, constrained));
    assert(!pwm_adc_phase_window_valid(0, -180, 780, constrained, constrained));
    assert(adc_completion_observation_valid(1, 3, -180, 400, 50, 57));
    // Completion before TEZ can also be valid; direction alone is not failure.
    assert(adc_completion_observation_valid(0, 1, -180, -20, 0, 2));
    assert(!adc_completion_observation_valid(0, 0, -180, 400, 0, 7));
    assert(!adc_completion_observation_valid(1, 1, -180, 400, 50, 57));
    assert(!adc_completion_observation_valid(3, 7, -180, 400, 0, 7));
    // Same apparent PWM phase in the next carrier is not this conversion.
    assert(!adc_completion_observation_valid(0, 1, -180, 400, 0, 57));
    assert(!adc_completion_observation_valid(0, 1, -180, -190, 0, 7));
    assert(!adc_completion_observation_valid(1, 3, -180, 400, 50, 49));
    assert(!adc_completion_observation_valid(0, 1, -181, 400, 0, 7));
    assert(!adc_completion_observation_valid(0, 1, 0, 400, 0, 5));
    // Captured on the board: counter spans 8.5 us while non-atomic timestamp
    // metadata spans 11 us. Accept this envelope without changing PWM guards.
    assert(adc_completion_observation_valid(1, 3, -170, 510, 49, 60));
    // Even under the 40 us deadline, inconsistent counter/time observations
    // cannot be used as a shortened conversion bound.
    assert(!adc_completion_observation_valid(0, 1, -180, 400, 0, 30));
    // Real powered capture: DONE low12 values match the owned DMA result,
    // while DONE upper tags already point to the next mux phase (A/B).
    // The actual DMA-tagged measurements are C/A, raw1830/1943.
    const uint32_t done_words[3] = {26406, 34711, 0};
    const uint32_t dma_words[2] = {42790, 26519};
    // Synthetic pre-START fixture for those C/A measurements; its previous
    // ADC value is deliberately unrelated. Only the armed mux tag is used.
    const uint32_t armed_words[3] = {(5U << 13) | 100, (3U << 13) | 200, 0};
    const auto done_pair = adc_current_pair_from_dma_done(dma_words, armed_words, done_words, 7);
    assert(done_pair.valid && done_pair.phase[0] == 2 && done_pair.phase[1] == 0);
    assert(done_pair.raw[0] == 1830 && done_pair.raw[1] == 1943);
    assert(done_pair.words[0] == dma_words[0] && done_pair.words[1] == dma_words[1]);
    assert(adc_current_pair_from_dma_done(dma_words, armed_words, done_words, 3).valid); // trailing DONE is not a measured phase
    assert(!adc_current_pair_from_dma_done(dma_words, armed_words, done_words, 1).valid);
    assert(!adc_current_pair_from_dma_done(dma_words, armed_words, done_words, 2).valid);
    const uint32_t mismatched_first[3] = {done_words[0] + 1, done_words[1], 0};
    const uint32_t mismatched_second[3] = {done_words[0], done_words[1] + 1, 0};
    assert(!adc_current_pair_from_dma_done(dma_words, armed_words, mismatched_first, 3).valid);
    assert(!adc_current_pair_from_dma_done(dma_words, armed_words, mismatched_second, 3).valid);
    const uint32_t duplicate_dma[2] = {(5U << 13) | 1830, (5U << 13) | 1943};
    const uint32_t duplicate_mux[3] = {5U << 13, 5U << 13, 0};
    assert(!adc_current_pair_from_dma_done(duplicate_dma, duplicate_mux, done_words, 7).valid);
    // Equal ADC values do not authorize a stale/shifted channel association.
    const uint32_t wrong_mux[3] = {(4U << 13) | 1830, armed_words[1], 0};
    assert(!adc_current_pair_from_dma_done(dma_words, wrong_mux, done_words, 7).valid);
    const uint32_t unarmed_mux[3] = {0, armed_words[1], 0};
    assert(!adc_current_pair_from_dma_done(dma_words, unarmed_mux, done_words, 7).valid);
    const uint32_t invalid_words[] = {(2U << 13) | 1830, (6U << 13) | 1830,
        dma_words[0] | (1U << 12), dma_words[0] | (1U << 17)};
    for (uint32_t invalid_word : invalid_words) {
        const uint32_t invalid_dma[2] = {invalid_word, dma_words[1]};
        assert(!adc_current_pair_from_dma_done(invalid_dma, armed_words, done_words, 7).valid);
    }
    // 25 conversion clocks at the specified 5 MHz take 5 us, leaving time
    // within the 65% window for the digital controller and DONE observation.
    // At the former 2.5 MHz, even conversion alone consumes the rising guard.
    constexpr int conversion_ticks = sampling_timing::adc_conversion_clocks *
        (sampling_timing::resolution_hz / sampling_timing::adc_sar_hz);
    static_assert(conversion_ticks == 400);
    assert(pwm_adc_window_valid(-180, -180 + conversion_ticks, sampling_compare, sampling_compare));
    assert(!pwm_adc_window_valid(-180, -180 + 2 * conversion_ticks, sampling_compare, sampling_compare));
    // Full modulation: U is continuously high; valid V/W samples can still
    // reconstruct it. Never accept a sampled high phase or an invalid channel.
    const uint16_t full_before[3] = {0, 1000, 2000};
    const uint16_t full_after[3] = {0, 1000, 2000};
    assert(pwm_adc_phase_window_valid(1, -80, 400, full_before, full_after));
    assert(pwm_adc_phase_window_valid(2, -80, 400, full_before, full_after));
    assert(!pwm_adc_phase_window_valid(0, -80, 400, full_before, full_after));
    assert(!pwm_adc_phase_window_valid(3, -80, 400, full_before, full_after));
    const uint16_t short_after[3] = {0, 400, 2000};
    assert(!pwm_adc_phase_window_valid(1, -80, 400, full_before, short_after));
    // No one-tick software cap at either endpoint: the TEZ shadow mode
    // explicitly produces constant low/high, and releases for interior duty.
    assert(pwm_force_shadow_bits(2000, 2000) == 0x41);
    assert(pwm_force_shadow_bits(0, 2000) == 0x81);
    assert(pwm_force_shadow_bits(1, 2000) == 1);
    assert(pwm_force_shadow_bits(1999, 2000) == 1);
    // 20 kHz/70%: -7.5 .. +7.5 us low window; six us CSA settling
    // and one us rising-edge guard leave -1.5 .. +6.5 us conversion time.
    assert(pwm_adc_window_valid(-80, 400, 600, 600));
    assert(!pwm_adc_window_valid(-121, 400, 600, 600));
    assert(!pwm_adc_window_valid(-80, 520, 600, 600));
    assert(!pwm_adc_window_valid(-80, 400, 600, 450));
    assert(!pwm_adc_window_valid(-80, -81, 600, 600));
    assert(!pwm_adc_window_valid(0, 1, 479, 600));
    // The real latched waveform on each side of TEZ controls acceptance.
    assert(pwm_adc_window_valid(-880, 2000, 2400, 2400));
    assert(!pwm_adc_window_valid(-880, 2000, 2400, 2000));
    assert(adc_stage_matches(1, 0) && adc_stage_matches(2, 1) && adc_stage_matches(3, 2));
    assert(!adc_stage_matches(2, 0) && !adc_stage_matches(3, 3) && !adc_stage_matches(4, 3));
    // A deferred request does not own the next START until the preceding
    // callback has completed its data and diagnostics, not just claimed a word.
    assert(!adc_next_stage_ready(1, 1, 2, 0));
    assert(adc_next_stage_ready(1, 1, 2, 1));
    assert(adc_next_stage_ready(1, 1, 3, 1));
    assert(!adc_next_stage_ready(2, 1, 3, 1));
    assert(!adc_next_stage_ready(2, 2, 3, 1));
    assert(adc_next_stage_ready(2, 2, 3, 3));
    assert(!adc_next_stage_ready(3, 3, 3, 7));
    assert(adc_staggered_timing_valid(50, 100, 107));
    assert(!adc_staggered_timing_valid(100, 150, 157));
    assert(!adc_staggered_timing_valid(50, 100, 120));
    uint16_t before[3][3] = {{600, 900, 1920}, {600, 900, 1920}, {700, 1000, 1920}};
    uint16_t after[3][3] = {{600, 900, 1920}, {600, 900, 1920}, {800, 1100, 1920}};
    assert(pwm_adc_pair_coherent(before, after));
    // A normal first-TEZ update does not switch the already-LOW sampled phase.
    // Its old falling edge and new rising edge still bound that ADC window.
    before[0][0] = 900; before[0][1] = 1000;
    assert(pwm_adc_pair_coherent(before, after));
    assert(pwm_adc_phase_window_valid(0, -80, 400, before[0], after[0]));
    assert(pwm_adc_phase_window_valid(1, -80, 400, before[1], after[1]));
    // Coherence alone must never authorize an insufficient physical window.
    before[0][0] = 500;
    assert(pwm_adc_pair_coherent(before, after));
    assert(!pwm_adc_phase_window_valid(0, -80, 400, before[0], after[0]));
    before[0][0] = 900;
    after[0][0] = before[1][0] = after[1][0] = 450;
    assert(pwm_adc_pair_coherent(before, after));
    assert(!pwm_adc_phase_window_valid(0, -80, 400, before[0], after[0]));
    after[0][0] = before[1][0] = after[1][0] = 600;
    // A command change between the measured phases or at the second TEZ is
    // still rejected, even when both individual phase windows are long enough.
    after[0][0] = 601;
    assert(!pwm_adc_pair_coherent(before, after));
    after[0][0] = 600; before[1][1] = 901;
    assert(!pwm_adc_pair_coherent(before, after));
    before[1][1] = 900; after[1][2] = 1919;
    assert(!pwm_adc_pair_coherent(before, after));
    assert(pwm_zero_command(0, 0, 0));
    assert(!pwm_zero_command(0.04f, 0.04f, 0.04f));
    assert(!pwm_zero_command(0, 0, 0.00001f));
    assert(zero_duty_mask(0, .04f, .7f) == 1);
    assert(zero_duty_mask(.04f, 0, .7f) == 2);
    assert(zero_duty_mask(.04f, .7f, 0) == 4);
    assert(zero_duty_mask(0, 0, 0) == 7);
    assert(zero_duty_mask(.04f, .04f, .04f) == 0);
    // Follow IDF's next-count correction with one coherent status read.
    constexpr auto up = decode_pwm_counter(321, 4000);
    constexpr auto down = decode_pwm_counter((1U << 16) | 319, 4000);
    static_assert(up.count == 320 && !up.down && up.signed_tick() == 320);
    static_assert(down.count == 320 && down.down && down.signed_tick() == -320);
    static_assert(up.phase_ticks(4000) == 320 && down.phase_ticks(4000) == 7680);
    // 10 kHz at +20 us: shadow write at +100 us, latch at +200 us.
    assert(pwm_latch_delay_ticks(1600, false, 4000) == 14400);
    // TEZ just occurred while interrupts were masked: its pending ISR writes
    // this cycle's shadow, so the command latches at the next physical TEZ.
    assert(pwm_latch_delay_ticks(160, true, 4000) == 7840);
    assert(pwm_latch_delay_ticks(1600, false, 2000) == 6400);
    // A direct shadow commit at +20 us in a 20 kHz period latches at +50 us.
    assert(pwm_direct_latch_delay_ticks(1600, 2000) == 2400);
    // Stage 1 protects the carrier between useful conversions. The TEP queue
    // skips +25 us, writes at +75 us, and latches at +100 us. Prediction must
    // include that extra 50 us even for a command arriving after the first TEP.
    assert(pwm_command_deferred(true, 1, false));
    assert(!pwm_command_deferred(false, 1, false));
    assert(!pwm_command_deferred(true, 0, false));
    assert(!pwm_command_deferred(true, 2, false));
    assert(!pwm_command_deferred(true, 3, false));
    assert(!pwm_command_deferred(true, 1, true));
    assert(pwm_staggered_latch_delay_ticks(960, 2000, true) == 7040); // +12 -> +100 us
    assert(pwm_staggered_latch_delay_ticks(3200, 2000, true) == 4800); // +40 -> +100 us
    assert(pwm_staggered_latch_delay_ticks(960, 2000, false) == 3040); // normal +12 -> +50 us
    // Stage 2's START is at most 2.25 us before TEZ. Although it releases pair
    // ownership, the 5 us commit guard permits the direct path only after TEZ.
    assert(!pwm_schedule_still_valid(5, 5, false, false, 3820, 3830, 4000, 400, 1, 50));
    assert(pwm_schedule_still_valid(6, 6, false, false, 80, 160, 4000, 400, 1, 50));
    assert(pwm_schedule_still_valid(5, 5, false, false, 960, 1040, 4000, 400, 1, 50, true, true));
    // Recompute the angle if ADC ownership changes while SVPWM is calculated,
    // even when no serviced/pending TEZ change has yet exposed the transition.
    assert(!pwm_schedule_still_valid(5, 5, false, false, 960, 1040, 4000, 400, 1, 50, true, false));
    assert(!pwm_schedule_still_valid(5, 5, false, false, 960, 1040, 4000, 400, 1, 50, false, true));
    // The queue owns one intended latch generation. A late ADC slot cannot
    // release an old angle prediction for application a carrier later.
    assert(pwm_queued_action(101, 102, true, false) == PwmQueuedAction::wait);
    assert(pwm_queued_action(101, 102, false, false) == PwmQueuedAction::wait);
    assert(pwm_queued_action(102, 102, false, false) == PwmQueuedAction::apply);
    assert(pwm_queued_action(102, 102, true, false) == PwmQueuedAction::discard);
    assert(pwm_queued_action(103, 102, false, false) == PwmQueuedAction::discard);
    assert(pwm_queued_action(110, 102, true, false) == PwmQueuedAction::discard);
    assert(pwm_queued_action(101, 102, true, true) == PwmQueuedAction::apply);
    assert(pwm_queued_action(102, 102, true, true) == PwmQueuedAction::apply);
    assert(pwm_queued_action(110, 102, true, true) == PwmQueuedAction::apply);
    assert(pwm_queued_action(0xffffffffU, 0, true, false) == PwmQueuedAction::wait);
    assert(pwm_queued_action(0, 0, false, false) == PwmQueuedAction::apply);
    assert(pwm_queued_action(1, 0, false, false) == PwmQueuedAction::discard);
    assert(pwm_queued_action(0, 0xffffffffU, false, false) == PwmQueuedAction::discard);
    assert(pwm_schedule_still_valid(5, 5, false, false, 1600, 2000, 8000, 160, 5, 100));
    assert(pwm_schedule_still_valid(5, 5, true, true, 160, 400, 8000, 160, 3, 100));
    // Serviced TEZ, newly pending TEZ, elapsed full cycle, wrap and guard.
    assert(!pwm_schedule_still_valid(5, 6, false, false, 1600, 2000, 8000, 160, 5, 100));
    assert(!pwm_schedule_still_valid(5, 5, false, true, 1600, 2000, 8000, 160, 5, 100));
    assert(!pwm_schedule_still_valid(5, 5, true, false, 1600, 2000, 8000, 160, 5, 100));
    assert(!pwm_schedule_still_valid(5, 5, false, false, 1600, 2000, 8000, 160, 105, 100));
    assert(!pwm_schedule_still_valid(5, 5, false, false, 7800, 80, 8000, 160, 5, 100));
    assert(!pwm_schedule_still_valid(5, 5, false, false, 7700, 7840, 8000, 160, 2, 100));
    // Unsigned generation wrap is harmless as long as no TEZ was serviced.
    assert(pwm_schedule_still_valid(0xffffffffU, 0xffffffffU, false, false, 1600, 2000, 8000, 160, 5, 100));
    assert(!pwm_schedule_still_valid(0xffffffffU, 0, false, false, 1600, 2000, 8000, 160, 5, 100));
}
