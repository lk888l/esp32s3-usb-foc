#include "control.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace knob {
float overcurrent_threshold(float limit) {
    return std::min(absolute_current_trip, limit + std::max(0.15f, limit * 0.25f));
}
const char* validate(const Config& c)
{
    const float values[] = {c.bus_voltage, c.sense_gain, c.current_limit, c.voltage_limit,
                           c.current_kp, c.current_ki, c.width, c.strength, c.endstop, c.damping, c.snap};
    for (float v : values) if (!std::isfinite(v)) return "non_finite";
    if (c.bus_voltage < 4.5f || c.bus_voltage > 30) return "bus_voltage_range";
    if (c.sense_gain != 0.15f && c.sense_gain != 0.3f && c.sense_gain != 0.6f && c.sense_gain != 1.2f) return "sense_gain_invalid";
    if (c.current_limit < 0.02f || c.current_limit > max_motor_current) return "current_limit_range";
    if (c.voltage_limit < 0.05f || c.voltage_limit > 5 || c.voltage_limit > c.bus_voltage * (duty_ceiling - duty_floor) / 1.7320508f) return "voltage_limit_range";
    if (c.current_kp < 0 || c.current_kp > 20 || c.current_ki < 0 || c.current_ki > 1000) return "pi_range";
    if (c.width < pi / 180 || c.width > pi || c.snap < 0.5f || c.snap > 1) return "detent_range";
    if (c.strength < 0 || c.strength > 10 || c.endstop < 0 || c.endstop > 20 || c.damping < 0 || c.damping > 0.5f) return "haptic_range";
    if (c.min_position < -10000 || c.max_position > 10000 || c.min_position > c.max_position) return "position_range";
    if (static_cast<int>(c.mode) < 0 || static_cast<int>(c.mode) > 3) return "mode_invalid";
    return nullptr;
}
bool valid_calibration(const Calibration& c) {
    return c.valid && (c.direction == -1 || c.direction == 1) && c.pole_pairs == 7 &&
           std::isfinite(c.electrical_zero) && c.electrical_zero >= 0 && c.electrical_zero < tau;
}
float wrap(float a) { a = std::fmod(a, tau); return a < 0 ? a + tau : a; }
float wrap_signed(float a) { return wrap(a + pi) - pi; }
float phase_current(float sense_volts, float offset_volts, float gain) {
    // TI DRV8316C Fig. 8-30: current flowing out of OUTx raises SOx.
    return (sense_volts - offset_volts) / gain;
}
Dq park(const std::array<float, 3>& i, float a) {
    // Remove common-mode sensor offset before Clarke transformation.
    const float alpha = (2 * i[0] - i[1] - i[2]) / 3;
    const float beta = (i[1] - i[2]) / 1.7320508f;
    const float s = std::sin(a), c = std::cos(a);
    return {alpha * c + beta * s, -alpha * s + beta * c};
}
std::array<float, 3> svpwm(Dq v, float a, float bus) {
    if (!std::isfinite(v.d) || !std::isfinite(v.q) || !std::isfinite(a) || !std::isfinite(bus) || bus < 4.5f)
        return {duty_floor, duty_floor, duty_floor};
    const float alpha = v.d * std::cos(a) - v.q * std::sin(a);
    const float beta = v.d * std::sin(a) + v.q * std::cos(a);
    std::array<float, 3> phases{alpha, -0.5f * alpha + 0.8660254f * beta, -0.5f * alpha - 0.8660254f * beta};
    const auto [lo, hi] = std::minmax_element(phases.begin(), phases.end());
    const float minimum = *lo, span = *hi - minimum;
    const float scale = span > bus * (duty_ceiling - duty_floor) ? (duty_ceiling - duty_floor) / span : 1 / bus;
    // Space vector common-mode injection: only line-to-line voltages matter.
    // Bias the zero vector toward 000 to retain the low-side sampling window.
    for (float& phase : phases) phase = std::clamp(duty_floor + (phase - minimum) * scale, duty_floor, duty_ceiling);
    return phases;
}
float Pi::step(float e, float dt, float kp, float ki, float limit) {
    if (!std::isfinite(e) || !std::isfinite(dt) || dt <= 0 || limit <= 0) { reset(); return 0; }
    const float candidate = std::clamp(integral_ + e * ki * dt, -limit, limit);
    const float raw = e * kp + candidate;
    if (std::abs(raw) <= limit || raw * e < 0) integral_ = candidate;
    return std::clamp(e * kp + integral_, -limit, limit);
}
Dq CurrentLoop::step(Dq error, float dt, float kp, float ki, float limit) {
    // Regulate flux first, then give torque the remaining voltage circle.
    // Each PI sees its actual available voltage, retaining anti-windup.
    const float d = d_.step(error.d, dt, kp, ki, limit);
    const float q_limit = std::sqrt(std::max(0.0f, limit * limit - d * d));
    return {d, q_.step(error.q, dt, kp, ki, q_limit)};
}
void CurrentLimiter::configure(float limit) {
    budget_ = std::clamp(budget_, 0.0f, limit);
    capacity_ = std::min(capacity_, budget_);
    applied_ = std::clamp(applied_, -capacity_, capacity_);
}
float CurrentLimiter::step(float requested, Dq measured, float phase_peak, float limit, float dt) {
    if (!std::isfinite(requested) || !std::isfinite(measured.d) || !std::isfinite(measured.q) ||
        !std::isfinite(phase_peak) || !std::isfinite(limit) || limit <= 0 || limit > max_motor_current ||
        !std::isfinite(dt) || dt <= 0 || dt > .002f) { reset(0); return 0; }
    configure(limit);
    const float peak = std::max(phase_peak, std::hypot(measured.d, measured.q));
    // Fast foldback on measured excess, slow recovery with hysteresis. The PI
    // continues regulating; crossing the operating limit does not disable PWM.
    if (peak > limit) budget_ = std::max(0.0f, budget_ - (peak - limit) * dt / .002f);
    else if (peak < limit * .9f) budget_ = std::min(limit, budget_ + limit * dt / .1f);
    // Reserve current for measured d-axis error, rather than clamping Iq alone.
    capacity_ = std::sqrt(std::max(0.0f, budget_ * budget_ - measured.d * measured.d));
    const float desired = std::clamp(requested, -capacity_, capacity_);
    // 0 -> 1 A takes at least 10 ms; lowering a limit still takes effect now.
    applied_ += std::clamp(desired - applied_, -100.0f * dt, 100.0f * dt);
    applied_ = std::clamp(applied_, -capacity_, capacity_);
    limited_ = std::abs(requested) >= limit - 1e-5f ||
        budget_ < limit - 1e-5f || std::abs(applied_ - requested) > 1e-5f;
    return applied_;
}
bool AngleTracker::update(float a, float dt) {
    if (!std::isfinite(a) || dt <= 0 || dt > 0.02f) return false;
    if (!ready_) { previous_ = a; continuous_ = a; ready_ = true; return true; }
    const float delta = wrap_signed(a - previous_);
    if (std::abs(delta) > 0.15f) return false;
    continuous_ += delta;
    const float measured_velocity = delta / dt;
    velocity_ += (dt / (0.01f + dt)) * (measured_velocity - velocity_);
    // Damping and positive-work limiting must follow a reversal promptly.
    // Keep the quieter 10 ms estimate for telemetry and speed-envelope gating.
    haptic_velocity_ += (dt / (0.002f + dt)) * (measured_velocity - haptic_velocity_);
    previous_ = a;
    return true;
}
void SettlingMonitor::update(float angle, float dt) {
    if (!std::isfinite(angle) || !std::isfinite(dt) || dt <= 0 || dt > .02f) { reset(); return; }
    if (!ready_) { minimum_ = maximum_ = angle; ready_ = true; }
    minimum_ = std::min(minimum_, angle); maximum_ = std::max(maximum_, angle);
    if (maximum_ - minimum_ >= .015f) quiet_windows_ = 0;
    elapsed_ += dt;
    if (elapsed_ >= .1f) {
        quiet_windows_ = maximum_ - minimum_ < .015f ? std::min(2U, quiet_windows_ + 1) : 0;
        elapsed_ = 0; minimum_ = maximum_ = angle;
    }
}
void Haptics::reconfigure(const Config& previous, const Config& next, float angle) {
    if (previous.mode != next.mode || previous.width != next.width ||
        previous.min_position != next.min_position || previous.max_position != next.max_position) {
        reset(angle, std::clamp(0, next.min_position, next.max_position));
    }
}
float HapticSpeedLimiter::step(float requested, float velocity, float limit, float dt) {
    if (!std::isfinite(requested) || !std::isfinite(velocity) ||
        !std::isfinite(limit) || limit <= 0 || limit > max_motor_current ||
        !std::isfinite(dt) || dt <= 0 || dt > .002f) {
        scale_ = 0; paused_ = true; quiet_s_ = 0; return 0;
    }
    const float speed = std::abs(velocity);
    if (speed >= haptic_speed_cutoff) { paused_ = true; quiet_s_ = 0; }
    if (paused_) {
        quiet_s_ = speed <= haptic_resume_speed ? quiet_s_ + dt : 0;
        if (quiet_s_ >= haptic_resume_hold_s) { paused_ = false; quiet_s_ = 0; }
    }
    // Smoothstep has zero slope at both ends of the fade band. Hysteresis
    // keeps a fast spin quiet. Require settled low speed, not a brief reversal,
    // before restoring detent/spring energy; recovering at 25 rad/s can kick.
    const float x = std::clamp((speed - haptic_fade_start) /
                              (haptic_speed_cutoff - haptic_fade_start), 0.0f, 1.0f);
    const float desired = paused_ ? 0.0f : 1.0f - x * x * (3.0f - 2.0f * x);
    scale_ = std::clamp(scale_ + std::clamp(desired - scale_,
                        -dt / haptic_fade_out_s, dt / haptic_fade_in_s), 0.0f, 1.0f);
    // Clamp before scaling so saturated endstops/springs also fade to zero.
    return std::clamp(requested, -limit, limit) * scale_;
}
float Haptics::step(const Config& c, float angle, float velocity) {
    if (!std::isfinite(angle) || !std::isfinite(velocity)) return 0;
    // Always track detents, even while the common speed limiter mutes torque.
    float error = angle - center_;
    // Catch up multiple detents without an unbounded loop or integer overflow.
    const float excess = std::abs(error) - c.width * c.snap;
    if (excess > 0 && c.mode != Mode::spring) {
        int steps = static_cast<int>(std::min(1000.0f, 1 + std::floor(excess / c.width)));
        if (error < 0) steps = -steps;
        int next = std::clamp(position_ + steps, -1000000, 1000000);
        if (c.mode == Mode::bounded) next = std::clamp(next, c.min_position, c.max_position);
        center_ += (next - position_) * c.width;
        position_ = next;
        error = angle - center_;
    }
    float iq = -c.damping * velocity;
    const bool endstop = c.mode == Mode::bounded && ((position_ == c.min_position && error < 0) || (position_ == c.max_position && error > 0));
    const float deadzone = std::min(c.width * 0.1f, pi / 180);
    if (c.mode == Mode::spring) iq -= c.strength * (angle - origin_);
    else if (endstop) iq -= c.endstop * error;
    else if (c.mode != Mode::free) iq -= c.strength * (error - std::clamp(error, -deadzone, deadzone));
    iq = std::clamp(iq, -c.current_limit, c.current_limit);
    // A wound-up spring/endstop must not accelerate back into the speed mute
    // on every recovery. Retain braking torque, taper only positive work.
    if ((c.mode == Mode::spring || endstop) && iq * velocity > 0) {
        iq *= std::clamp((25.0f - std::abs(velocity)) / 15.0f, 0.0f, 1.0f);
    }
    return iq;
}
} // namespace knob
