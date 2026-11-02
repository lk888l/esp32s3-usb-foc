#pragma once
#include <array>
#include <cstdint>

namespace knob {
constexpr float pi = 3.14159265358979323846f;
constexpr float tau = 2 * pi;
constexpr float max_motor_current = 1.0f;
constexpr float absolute_current_trip = 1.25f;
// Fault protection has transient headroom above the normal current regulator.
float overcurrent_threshold(float current_limit);
class CurrentTripFilter {
public:
    void reset() { consecutive_ = 0; }
    // 2 kHz valid samples: qualify ordinary excess for 2 ms. Severe excess
    // remains an immediate cutoff, independent of this qualification.
    bool step(bool over_limit, bool over_absolute, bool valid = true) {
        if (!valid) { reset(); return false; }
        if (over_absolute) return true;
        if (!over_limit) { reset(); return false; }
        consecutive_ = consecutive_ < 4 ? consecutive_ + 1 : 4;
        return consecutive_ >= 4;
    }
private:
    unsigned consecutive_ = 0;
};
constexpr float duty_floor = 0.04f;
constexpr float duty_ceiling = 0.40f; // 30 us all-low vector at 20 kHz.
enum class Mode : int { free, detents, bounded, spring };
struct Config {
    float bus_voltage = 11.62f;
    float sense_gain = 0.15f; // V/A; minimum supported gain gives conservative current estimates.
    float current_limit = 0.65f;
    float voltage_limit = 2.35f;
    float current_kp = 1.5f;
    float current_ki = 300;
    float width = pi / 6;
    float strength = 5.0f; // A/rad
    float endstop = 8.0f; // A/rad
    float damping = 0.015f; // A/(rad/s)
    float snap = 0.6f;
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
float wrap(float angle);
float wrap_signed(float angle);
struct Dq { float d = 0, q = 0; };
Dq park(const std::array<float, 3>& current, float angle);
float phase_current(float sense_volts, float offset_volts, float gain);
std::array<float, 3> svpwm(Dq voltage, float angle, float bus_voltage);
class Pi {
public:
    float step(float error, float dt, float kp, float ki, float limit);
    void reset() { integral_ = 0; }
private:
    float integral_ = 0;
};
class CurrentLoop {
public:
    Dq step(Dq error, float dt, float kp, float ki, float voltage_limit);
    void reset() { d_.reset(); q_.reset(); }
private:
    Pi d_, q_;
};
class CurrentLimiter {
public:
    void reset(float limit) { budget_ = limit; applied_ = 0; capacity_ = limit; limited_ = false; }
    void configure(float limit);
    float step(float requested_q, Dq measured, float phase_peak, float limit, float dt);
    float capacity() const { return capacity_; }
    bool limited() const { return limited_; }
private:
    float budget_ = 0, applied_ = 0, capacity_ = 0;
    bool limited_ = false;
};
class AngleTracker {
public:
    bool update(float wrapped_angle, float dt);
    float angle() const { return continuous_; }
    float velocity() const { return velocity_; }
    float haptic_velocity() const { return haptic_velocity_; }
private:
    bool ready_ = false;
    float previous_ = 0, continuous_ = 0, velocity_ = 0, haptic_velocity_ = 0;
};
class SettlingMonitor {
public:
    void reset() { *this = {}; }
    void update(float angle, float dt);
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
class HapticSpeedLimiter {
public:
    void reset() { scale_ = 1; paused_ = false; quiet_s_ = 0; }
    // Apply after combining all effects (including clicks), before current PI.
    float step(float requested, float velocity, float current_limit, float dt);
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
    float step(const Config& config, float angle, float velocity);
    int position() const { return position_; }
    float sub_position(const Config& config, float angle) const { return (angle - center_) / config.width; }
private:
    float center_ = 0, origin_ = 0;
    int position_ = 0;
};
} // namespace knob
