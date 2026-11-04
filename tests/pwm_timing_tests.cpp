#include "pwm_timing.hpp"
#include <cassert>
using namespace knob;
int main() {
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
    after[0][0] = 601;
    assert(!pwm_adc_pair_coherent(before, after));
    after[0][0] = 600; before[1][1] = 901;
    assert(!pwm_adc_pair_coherent(before, after));
    assert(pwm_zero_command(0, 0, 0));
    assert(!pwm_zero_command(0.04f, 0.04f, 0.04f));
    assert(!pwm_zero_command(0, 0, 0.00001f));
    assert(zero_duty_mask(0, .04f, .7f) == 1);
    assert(zero_duty_mask(.04f, 0, .7f) == 2);
    assert(zero_duty_mask(.04f, .7f, 0) == 4);
    assert(zero_duty_mask(0, 0, 0) == 7);
    assert(zero_duty_mask(.04f, .04f, .04f) == 0);
    PwmForceLowState force;
    assert(force.active == 7);
    force.queue(1);
    assert(force.active == 7 && force.shadow == 7);
    force.commit_shadow();
    force.queue(2); // A pending command must not replace the already staged U clamp.
    assert(force.shadow == 1 && force.active == 7);
    assert(force.latch() == 6 && force.active == 1);
    force.commit_shadow();
    force.queue(4); force.commit_shadow(); // Supersede V before hardware TEZ.
    assert(force.active == 1);
    assert(force.latch() == 5 && force.active == 4);
    assert(force.latch() == 0); // No force-register writes for an unchanged clamp.
    force.queue(0); force.commit_shadow();
    assert(force.latch() == 4 && force.active == 0); // Leave W clamp, enable PWM.
    force.queue(7); force.commit_shadow();
    assert(force.latch() == 7 && force.active == 7); // Stop forces all phases low.
    force = {}; // Reinitialize mirrors to hardware startup state.
    assert(force.pending == 7 && force.shadow == 7 && force.active == 7);
    const uint32_t release_compare[3] = {1920, 600, 1800};
    // 20k:4% pulse begins at24us. Releasing at24.8us must fail even
    // though the counter is still going up and below the25us peak.
    assert(!pwm_force_update_fits({1984, false}, 1, release_compare, 400));
    // A full5us operation margin must fit strictly before the changed edge.
    assert(pwm_force_update_fits({1519, false}, 1, release_compare, 400));
    assert(!pwm_force_update_fits({1520, false}, 1, release_compare, 400));
    assert(!pwm_force_update_fits({100, true}, 1, release_compare, 400));
    // An unchanged phase can already be high; only changed force bits need
    // guarding. Migrating the clamp must account for the newly released one.
    assert(pwm_force_update_fits({750, false}, 1, release_compare, 400));
    assert(!pwm_force_update_fits({750, false}, 3, release_compare, 400));
    const uint32_t migrate_compare[3] = {2000, 650, 1900};
    assert(pwm_force_update_fits({249, false}, 3, migrate_compare, 400));
    assert(!pwm_force_update_fits({250, false}, 3, migrate_compare, 400));
    // Postwrite check detects a missed edge even when entry was on time.
    assert(pwm_force_update_fits({649, false}, 3, migrate_compare, 0));
    assert(!pwm_force_update_fits({650, false}, 3, migrate_compare, 0));
    assert(pwm_force_update_fits({1984, false}, 0, release_compare, 400));
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
