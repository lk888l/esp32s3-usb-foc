#pragma once
#include <cstdint>

namespace knob::sampling_timing {
// All timing values use the same 80 MHz MCPWM clock as the HAL. The settling
// budget includes board propagation and CSA settling; it is not just the
// amplifier's typical data-sheet value. Completion is observed by the dedicated
// ADC1_DONE ISR, independently of later DMA delivery and task scheduling.
constexpr uint32_t resolution_hz = 80'000'000;
// ESP32-S3 TRM 39.3.7.1: 25 SAR clocks per conversion, SAR clock <= 5 MHz.
// The +1 encoding of both dividers is also documented in ESP-IDF's
// components/bootloader_support/src/bootloader_random_esp32s3.c.
// Default IDF CLKM=15/SAR=1 runs SAR at only 2.5 MHz. Our PWM-triggered adapter
// uses CLKM=7/SAR=1: 10 MHz digital control, 5 MHz analog conversion. ADC2's
// independent RTC oneshot clock is unaffected.
constexpr uint32_t adc_apb_hz = 80'000'000;
constexpr uint32_t adc_controller_divider = 7, adc_sar_divider = 1;
constexpr uint32_t adc_controller_hz = adc_apb_hz / (adc_controller_divider + 1);
constexpr uint32_t adc_sar_hz = adc_controller_hz / (adc_sar_divider + 1);
constexpr uint32_t adc_conversion_clocks = 25;
static_assert(adc_sar_divider + 1 >= 2 && adc_sar_hz <= 5'000'000);
static_assert(adc_apb_hz % (adc_controller_divider + 1) == 0 &&
    adc_controller_hz % (adc_sar_divider + 1) == 0);
constexpr uint32_t settling_ticks = 480;        // 6 us after the falling edge
constexpr uint32_t rising_guard_ticks = 80;    // 1 us before the rising edge
constexpr int start_tick = -180;              // START no earlier than -2.25 us
constexpr uint32_t completion_budget_tick = 560; // ADC1_DONE observation: +7 us
constexpr uint32_t reserve_ticks = 40;         // another 0.5 us on both bounds

// Around TEZ, the low-side interval is [-falling_compare, +rising_compare].
// Derive a modulation limit that can contain the entire acquisition envelope.
// The +1 preserves the runtime completion check's strict upper inequality.
constexpr uint32_t falling_requirement = settling_ticks + static_cast<uint32_t>(-start_tick);
constexpr uint32_t rising_requirement = completion_budget_tick + rising_guard_ticks + 1;
constexpr uint32_t minimum_compare_ticks =
    (falling_requirement > rising_requirement ? falling_requirement : rising_requirement) + reserve_ticks;
constexpr float duty_ceiling(uint32_t half_period_ticks) {
    return half_period_ticks > minimum_compare_ticks
        ? static_cast<float>(half_period_ticks - minimum_compare_ticks) / half_period_ticks : 0.0f;
}
static_assert(start_tick < 0 && minimum_compare_ticks == 700);
}
