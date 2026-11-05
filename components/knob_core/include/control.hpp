#pragma once
#include <array>
#include <cstdint>
#include "pwm_profile.hpp"
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define KNOB_IRAM IRAM_ATTR
#else
#define KNOB_IRAM
#endif

namespace knob {
constexpr float pi = 3.14159265358979323846f;
constexpr float tau = 2 * pi;
constexpr float max_motor_current = 4.5f;
constexpr float peak_current_trip = 5.0f;
constexpr float absolute_current_trip = 6.0f;
constexpr float max_haptic_strength = 1000.0f;
constexpr unsigned current_overload_trip_us = 20000;
constexpr unsigned current_peak_trip_us = 1000;
constexpr unsigned current_trip_recovery_ratio = 4;
// The target stays within the configured limit, at most 4.5 A. Short overloads
// first fold back; these bands bound their duration independently of the PI.
KNOB_IRAM float overcurrent_threshold(float current_limit);
enum class CurrentTripLevel { normal, overload, peak, emergency, invalid };
class CurrentTripFilter {
public:
    void reset() { overload_ticks_ = peak_ticks_ = 0; }
    // Exposure builds at real elapsed time and recovers four times more slowly.
    // A brief dip cannot hide repeated excess. Invalid samples freeze evidence;
    // the sampling recovery path handles loss of trustworthy feedback.
    KNOB_IRAM bool step(CurrentTripLevel level, unsigned elapsed_us) {
        if (level == CurrentTripLevel::invalid) return false;
        if (level == CurrentTripLevel::emergency) return true;
        if (elapsed_us > current_overload_trip_us) elapsed_us = current_overload_trip_us;
        const bool peak = level == CurrentTripLevel::peak;
        update(overload_ticks_, peak || level == CurrentTripLevel::overload,
               elapsed_us, current_overload_trip_us);
        update(peak_ticks_, peak, elapsed_us, current_peak_trip_us);
        return overload_ticks_ >= current_overload_trip_us * current_trip_recovery_ratio ||
               peak_ticks_ >= current_peak_trip_us * current_trip_recovery_ratio;
    }
private:
    static KNOB_IRAM void update(unsigned& ticks, bool above, unsigned elapsed_us, unsigned trip_us) {
        const unsigned ceiling = trip_us * current_trip_recovery_ratio;
        if (above) {
            const unsigned increase = elapsed_us * current_trip_recovery_ratio;
            ticks += increase < ceiling - ticks ? increase : ceiling - ticks;
        } else {
            ticks = elapsed_us < ticks ? ticks - elapsed_us : 0;
        }
    }
    // Quarter-microsecond units retain recovery accuracy at any sample rate.
    unsigned overload_ticks_ = 0, peak_ticks_ = 0;
};
constexpr float duty_floor = pwm_zero_floor ? 0.0f : 0.04f;
constexpr float duty_ceiling = pwm_profile_duty_ceiling;
// Zero-clamp falls back to a common minimum pulse near a sector boundary.
// floor/ceiling <= (3-sqrt(6))/6 guarantees this fallback fits the full circle.
static_assert(!pwm_zero_clamp || duty_floor <= duty_ceiling * .09f);
// Maximum linear dq voltage circle. Full-range SVPWM gives Vbus / sqrt(3).
// Invalid bus readings provide no output capacity; callers must also stop on
// missing/stale measurements rather than substituting a configured voltage.
KNOB_IRAM float max_output_voltage(float bus_voltage);
enum class Mode : int { free, detents, bounded, spring };
struct Config {
    float bus_voltage = 19.4f;
    float sense_gain = 0.15f; // V/A; minimum supported gain gives conservative current estimates.
    float current_limit = 0.8f;
    // Retain this slot only to read existing NVS blobs without shifting gains
    // or calibration. It is never validated, exposed, or used by control.
    float legacy_voltage_limit = 2.35f;
    float current_kp = 0.4f;
    float current_ki = 200;
    float width = pi / 18;
    float strength = 20.0f; // A/rad outside the SmartKnob center dead zone; capped by current_limit.
    float endstop = 8.0f; // A/rad
    float damping = 0.003f; // A/(rad/s); velocity damping throughout each detent.
    float snap = 0.55f;
    int min_position = 0;
    int max_position = 11;
    Mode mode = Mode::detents;
};
struct Calibration {
    bool valid = false;
    int direction = 1;
    int pole_pairs = 7;
    float electrical_zero = 0;
};
const char* validate(const Config& config);
bool valid_calibration(const Calibration& calibration);
KNOB_IRAM float wrap(float angle);
KNOB_IRAM float wrap_signed(float angle);
struct Dq { float d = 0, q = 0; };
KNOB_IRAM Dq park(const std::array<float, 3>& current, float angle);
KNOB_IRAM float phase_current(float sense_volts, float offset_volts, float gain);
KNOB_IRAM std::array<float, 3> svpwm(Dq voltage, float angle, float bus_voltage);
class Pi {
public:
    KNOB_IRAM float step(float error, float dt, float kp, float ki, float limit, float feedforward = 0);
    void reset() { integral_ = 0; }
private:
    float integral_ = 0;
};
class CurrentLoop {
public:
    KNOB_IRAM Dq step(Dq error, float dt, float kp, float ki, float voltage_limit, Dq feedforward = {});
    void reset() { d_.reset(); q_.reset(); }
private:
    Pi d_, q_;
};
// Quiet only an exact zero control intent at rest, never a small-current band.
// Moving zero-current operation still needs PI to cancel back-EMF.
class QuietOutput {
public:
    void reset() { quiet_ = false; idle_s_ = 0; }
    KNOB_IRAM bool step(bool zero_intent, float velocity, float dt);
private:
    bool quiet_ = false;
    float idle_s_ = 0;
};
class CurrentLimiter {
public:
    void reset(float limit) { budget_ = capacity_ = limit; limited_ = false; }
    KNOB_IRAM void configure(float limit);
    KNOB_IRAM float step(float requested_q, Dq measured, float phase_peak, float limit, float dt);
    float capacity() const { return capacity_; }
    bool limited() const { return limited_; }
private:
    float budget_ = 0, capacity_ = 0;
    bool limited_ = false;
};
class AngleTracker {
public:
    void reset() { *this = {}; }
    KNOB_IRAM bool update(float wrapped_angle, float dt);
    float angle() const { return continuous_; }
    float velocity() const { return velocity_; }
    float haptic_velocity() const { return haptic_velocity_; }
    // Predict directly from the latest continuous angle and fast velocity.
    // No extra position smoothing or detent-specific velocity path.
    KNOB_IRAM float haptic_angle(float horizon = 0) const;
    float wrapped_angle() const { return previous_; }
private:
    bool ready_ = false;
    int32_t turns_ = 0;
    float previous_ = 0, continuous_ = 0, velocity_ = 0, haptic_velocity_ = 0;
};
class SettlingMonitor {
public:
    void reset() { *this = {}; }
    KNOB_IRAM void update(float angle, float dt);
    bool still() const { return quiet_windows_ >= 2 && maximum_ - minimum_ < 0.015f; }
private:
    bool ready_ = false;
    float minimum_ = 0, maximum_ = 0, elapsed_ = 0;
    unsigned quiet_windows_ = 0;
};
// Speed is an operating envelope for haptics, not a latched motor fault.
constexpr float haptic_fade_start = 25.0f; // rad/s
constexpr float haptic_speed_cutoff = 40.0f;
constexpr float haptic_fade_out_s = 0.010f;
constexpr float haptic_fade_in_s = 0.500f;
constexpr float haptic_resume_speed = 5.0f;
constexpr float haptic_resume_hold_s = 0.030f;
constexpr float detent_assist_fade_start = 10.0f;
constexpr float detent_assist_cutoff = 25.0f;
constexpr float elastic_min_damping = 0.04f; // Return damping for springs/endstops only.
class HapticSpeedLimiter {
public:
    void reset() { scale_ = 1; paused_ = false; quiet_s_ = 0; }
    // Apply after combining all effects (including clicks), before current PI.
    KNOB_IRAM float step(float requested, float velocity, float current_limit, float dt);
    float scale() const { return scale_; }
    bool paused() const { return paused_; }
    bool limited() const { return scale_ < 1; }
private:
    float scale_ = 1, quiet_s_ = 0;
    bool paused_ = false;
};
class Haptics {
public:
    void reset(float angle, int position = 0) { center_ = angle; origin_ = angle; position_ = position; }
    // Preserve the center while tuning strength; geometry changes start here.
    void reconfigure(const Config& previous, const Config& next, float angle);
    KNOB_IRAM float step(const Config& config, float angle, float velocity);
    int position() const { return position_; }
    float sub_position(const Config& config, float angle) const { return (angle - center_) / config.width; }
private:
    float center_ = 0, origin_ = 0;
    int position_ = 0;
};
} // namespace knob
