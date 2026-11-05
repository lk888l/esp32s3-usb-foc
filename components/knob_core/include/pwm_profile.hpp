#pragma once
#include <cstdint>
#include "sampling_timing.hpp"
#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

namespace knob {
// The default voltage circle is bounded by the ADC acquisition envelope.
// Explicit high-duty profiles remain available for controlled comparisons;
// they do not guarantee a measurable low-side window for every phase pair.
// Pulse-floor policy remains independent of that voltage ceiling.
#if defined(CONFIG_KNOB_PWM_20K90) && CONFIG_KNOB_PWM_20K90
constexpr uint32_t pwm_profile_hz = 20000;
constexpr float pwm_profile_duty_ceiling = .90f;
constexpr bool pwm_sampling_window_constrained = false;
constexpr bool pwm_zero_clamp = true, pwm_zero_floor = true;
constexpr const char* pwm_profile_name = "20k90_svpwm";
#elif defined(CONFIG_KNOB_PWM_20K65) && CONFIG_KNOB_PWM_20K65
constexpr uint32_t pwm_profile_hz = 20000;
constexpr float pwm_profile_duty_ceiling = sampling_timing::duty_ceiling(sampling_timing::resolution_hz / (2 * pwm_profile_hz));
constexpr bool pwm_sampling_window_constrained = true;
constexpr bool pwm_zero_clamp = true, pwm_zero_floor = true;
constexpr const char* pwm_profile_name = "20k65_sampling";
#elif defined(CONFIG_KNOB_PWM_20K100) && CONFIG_KNOB_PWM_20K100
constexpr uint32_t pwm_profile_hz = 20000;
constexpr float pwm_profile_duty_ceiling = 1.0f;
constexpr bool pwm_sampling_window_constrained = false;
constexpr bool pwm_zero_clamp = true, pwm_zero_floor = true;
constexpr const char* pwm_profile_name = "20k100_svpwm";
#elif defined(CONFIG_KNOB_PWM_20K70) && CONFIG_KNOB_PWM_20K70
constexpr uint32_t pwm_profile_hz = 20000;
constexpr float pwm_profile_duty_ceiling = .70f;
constexpr bool pwm_sampling_window_constrained = false;
constexpr bool pwm_zero_clamp = true, pwm_zero_floor = false;
constexpr const char* pwm_profile_name = "20k70_experimental";
#elif defined(CONFIG_KNOB_PWM_5K70) && CONFIG_KNOB_PWM_5K70
constexpr uint32_t pwm_profile_hz = 5000;
constexpr float pwm_profile_duty_ceiling = .70f;
constexpr bool pwm_sampling_window_constrained = false;
constexpr bool pwm_zero_clamp = false, pwm_zero_floor = false;
constexpr const char* pwm_profile_name = "5k70";
#elif defined(CONFIG_KNOB_PWM_10K50) && CONFIG_KNOB_PWM_10K50
constexpr uint32_t pwm_profile_hz = 10000;
constexpr float pwm_profile_duty_ceiling = .50f;
constexpr bool pwm_sampling_window_constrained = false;
constexpr bool pwm_zero_clamp = false, pwm_zero_floor = false;
constexpr const char* pwm_profile_name = "10k50";
#else
constexpr uint32_t pwm_profile_hz = 20000;
constexpr float pwm_profile_duty_ceiling = sampling_timing::duty_ceiling(sampling_timing::resolution_hz / (2 * pwm_profile_hz));
constexpr bool pwm_sampling_window_constrained = true;
constexpr bool pwm_zero_clamp = true, pwm_zero_floor = true;
constexpr const char* pwm_profile_name = "20k65_sampling";
#endif
constexpr bool pwm_full_range = pwm_profile_duty_ceiling == 1.0f;
}
