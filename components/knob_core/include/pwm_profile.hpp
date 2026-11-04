#pragma once
#include <cstdint>
#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

namespace knob {
// Kconfig selects the hardware timing and voltage span together. Host tests
// use the same CONFIG_* defines to exercise every supported profile.
#if defined(CONFIG_KNOB_PWM_20K70) && CONFIG_KNOB_PWM_20K70
constexpr uint32_t pwm_profile_hz = 20000;
constexpr float pwm_profile_duty_ceiling = .70f;
constexpr bool pwm_zero_clamp = true;
constexpr const char* pwm_profile_name = "20k70_experimental";
#elif defined(CONFIG_KNOB_PWM_5K70) && CONFIG_KNOB_PWM_5K70
constexpr uint32_t pwm_profile_hz = 5000;
constexpr float pwm_profile_duty_ceiling = .70f;
constexpr bool pwm_zero_clamp = false;
constexpr const char* pwm_profile_name = "5k70";
#else
constexpr uint32_t pwm_profile_hz = 10000;
constexpr float pwm_profile_duty_ceiling = .50f;
constexpr bool pwm_zero_clamp = false;
constexpr const char* pwm_profile_name = "10k50";
#endif
}
