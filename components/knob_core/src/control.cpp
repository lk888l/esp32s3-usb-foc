#include "control.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace knob {
float overcurrent_threshold(float limit) {
    return std::min(peak_current_trip, limit + std::max(0.15f, limit * 0.25f));
}
float max_output_voltage(float bus_voltage) {
    if (!std::isfinite(bus_voltage) || bus_voltage < 4.5f || bus_voltage > 30.0f) return 0;
    const float span = pwm_zero_clamp ? duty_ceiling : duty_ceiling - duty_floor;
    return bus_voltage * span / 1.7320508f;
}
float effective_voltage_limit(float requested_limit, float bus_voltage) {
    if (!std::isfinite(requested_limit) || requested_limit <= 0) return 0;
    return std::min(requested_limit, max_output_voltage(bus_voltage));
}
const char* validate(const Config& c)
{
    const float values[] = {c.bus_voltage, c.sense_gain, c.current_limit, c.voltage_limit,
                           c.current_kp, c.current_ki, c.width, c.strength, c.endstop, c.damping, c.snap};
    for (float v : values) if (!std::isfinite(v)) return "non_finite";
    if (c.bus_voltage < 4.5f || c.bus_voltage > 30) return "bus_voltage_range";
    if (c.sense_gain != 0.15f && c.sense_gain != 0.3f && c.sense_gain != 0.6f && c.sense_gain != 1.2f) return "sense_gain_invalid";
    if (c.current_limit < 0.02f || c.current_limit > max_motor_current) return "current_limit_range";
    if (c.voltage_limit < 0.05f || c.voltage_limit > max_output_voltage(c.bus_voltage)) return "voltage_limit_range";
    if (c.current_kp < 0 || c.current_kp > 20 || c.current_ki < 0 || c.current_ki > 1000) return "pi_range";
    if (c.width < pi / 180 || c.width > pi || c.snap < 0.5f || c.snap > 1) return "detent_range";
    if (c.strength < 0 || c.strength > max_haptic_strength || c.endstop < 0 || c.endstop > 20 || c.damping < 0 || c.damping > 0.5f) return "haptic_range";
    if (c.min_position < -10000 || c.max_position > 10000 || c.min_position > c.max_position) return "position_range";
    if (static_cast<int>(c.mode) < 0 || static_cast<int>(c.mode) > 3) return "mode_invalid";
    return nullptr;
}
bool valid_calibration(const Calibration& c) {
    return c.valid && (c.direction == -1 || c.direction == 1) && c.pole_pairs == 7 &&
           std::isfinite(c.electrical_zero) && c.electrical_zero >= 0 && c.electrical_zero < tau;
}
float wrap(float a) {
    if (a >= 0 && a < tau) return a;
    if (a >= -tau && a < 0) return a + tau;
    if (a >= tau && a < 2*tau) return a - tau;
    return a - tau * std::floor(a / tau);
}
float wrap_signed(float a) { return wrap(a + pi) - pi; }
// Range-reduced ninth-order sine: <4e-6 absolute error over a turn.
// Avoid libm's flash-resident argument reduction on the 5 kHz path.
static KNOB_IRAM float fast_sin(float a) {
    float x = wrap_signed(a);
    if (x > pi*.5f) x = pi-x;
    else if (x < -pi*.5f) x = -pi-x;
    const float x2 = x*x;
    return x * (1 + x2 * (-1.0f/6 + x2 * (1.0f/120 + x2 * (-1.0f/5040 + x2/362880))));
}
static KNOB_IRAM float fast_cos(float a) { return fast_sin(a + pi*.5f); }
float phase_current(float sense_volts, float offset_volts, float gain) {
    // TI DRV8316C Fig. 8-30: current flowing out of OUTx raises SOx.
    return (sense_volts - offset_volts) / gain;
}
Dq park(const std::array<float, 3>& i, float a) {
    // Remove common-mode sensor offset before Clarke transformation.
    const float alpha = (2 * i[0] - i[1] - i[2]) / 3;
    const float beta = (i[1] - i[2]) / 1.7320508f;
    const float s = fast_sin(a), c = fast_cos(a);
    return {alpha * c + beta * s, -alpha * s + beta * c};
}
std::array<float, 3> svpwm(Dq v, float a, float bus) {
    if (!std::isfinite(v.d) || !std::isfinite(v.q) || !std::isfinite(a) || !std::isfinite(bus) || bus < 4.5f)
        return {duty_floor, duty_floor, duty_floor};
    const float alpha = v.d * fast_cos(a) - v.q * fast_sin(a);
    const float beta = v.d * fast_sin(a) + v.q * fast_cos(a);
    std::array<float, 3> phases{alpha, -0.5f * alpha + 0.8660254f * beta, -0.5f * alpha - 0.8660254f * beta};
    const auto [lo, hi] = std::minmax_element(phases.begin(), phases.end());
    const float minimum = *lo, span = *hi - minimum;
    if constexpr (pwm_zero_clamp) {
        // Clamp the minimum phase low when both remaining pulses are long
        // enough. Near a sector boundary, shift all three pulses up instead;
        // this avoids creating a nonzero pulse shorter than duty_floor.
        // Re-evaluate the middle pulse AFTER over-range scaling so a large,
        // narrow vector cannot accidentally create a sub-minimum pulse.
        const float middle = std::max(std::min(phases[0], phases[1]),
                                      std::min(std::max(phases[0], phases[1]), phases[2]));
        float scale = span > bus * duty_ceiling ? duty_ceiling / span : 1 / bus;
        const float offset = (middle - minimum) * scale >= duty_floor ? 0 : duty_floor;
        if (offset && span > bus * (duty_ceiling - duty_floor))
            scale = (duty_ceiling - duty_floor) / span;
        for (float& phase : phases) {
            const float pulse = offset + (phase - minimum) * scale;
            phase = pulse == 0 ? 0 : std::clamp(pulse, duty_floor, duty_ceiling);
        }
        return phases;
    }
    const float scale = span > bus * (duty_ceiling - duty_floor) ? (duty_ceiling - duty_floor) / span : 1 / bus;
    // Space vector common-mode injection: only line-to-line voltages matter.
    // Bias the zero vector toward 000 to retain the low-side sampling window.
    for (float& phase : phases) phase = std::clamp(duty_floor + (phase - minimum) * scale, duty_floor, duty_ceiling);
    return phases;
}
float Pi::step(float e, float dt, float kp, float ki, float limit, float feedforward) {
    if (!std::isfinite(e) || !std::isfinite(dt) || !std::isfinite(feedforward) || dt <= 0 || limit <= 0) { reset(); return 0; }
    const float candidate = std::clamp(integral_ + e * ki * dt, -limit, limit);
    const float raw = feedforward + e * kp + candidate;
    if (std::abs(raw) <= limit || raw * e < 0) integral_ = candidate;
    return std::clamp(feedforward + e * kp + integral_, -limit, limit);
}
Dq CurrentLoop::step(Dq error, float dt, float kp, float ki, float limit, Dq feedforward) {
    // Regulate flux first, then give torque the remaining voltage circle.
    // Each PI sees its actual available voltage, retaining anti-windup.
    const float d = d_.step(error.d, dt, kp, ki, limit, feedforward.d);
    const float q_limit = std::sqrt(std::max(0.0f, limit * limit - d * d));
    return {d, q_.step(error.q, dt, kp, ki, q_limit, feedforward.q)};
}
bool QuietOutput::step(bool zero_intent, float velocity, float dt) {
    if (!zero_intent || !std::isfinite(velocity) || !std::isfinite(dt) || dt <= 0 || dt > .002f) {
        reset(); return false;
    }
    const float speed = std::abs(velocity);
    if (quiet_) {
        if (speed > .7f) reset();
    } else {
        idle_s_ = speed < .25f ? idle_s_ + dt : 0;
        if (idle_s_ >= .02f) quiet_ = true;
    }
    return quiet_;
}
void CurrentLimiter::configure(float limit) {
    budget_ = std::clamp(budget_, 0.0f, limit);
    capacity_ = std::min(capacity_, budget_);
}
float CurrentLimiter::step(float requested, Dq measured, float phase_peak, float limit, float dt) {
    if (!std::isfinite(requested) || !std::isfinite(measured.d) || !std::isfinite(measured.q) ||
        !std::isfinite(phase_peak) || !std::isfinite(limit) || limit <= 0 || limit > max_motor_current ||
        !std::isfinite(dt) || dt <= 0 || dt > .002f) { reset(0); return 0; }
    configure(limit);
    const float peak = std::max(phase_peak, std::sqrt(measured.d * measured.d + measured.q * measured.q));
    // Fast foldback on measured excess, slow recovery with hysteresis. The PI
    // continues regulating; crossing the operating limit does not disable PWM.
    if (peak > limit) budget_ = std::max(0.0f, budget_ - (peak - limit) * dt / .002f);
    else if (peak < limit * .9f) budget_ = std::min(limit, budget_ + limit * dt / .1f);
    // Reserve current for measured d-axis error, rather than clamping Iq alone.
    capacity_ = std::sqrt(std::max(0.0f, budget_ * budget_ - measured.d * measured.d));
    const float desired = std::clamp(requested, -capacity_, capacity_);
    // Follow the available current immediately, including release/reversal.
    // Measured-current foldback still bounds the target; an extra slew would
    // flatten short detent pulses as the requested peak current increases.
    limited_ = std::abs(requested) >= limit - 1e-5f ||
        budget_ < limit - 1e-5f || std::abs(desired - requested) > 1e-5f;
    return desired;
}
bool AngleTracker::update(float a, float dt) {
    if (!std::isfinite(a) || !std::isfinite(dt) || dt <= 0 || dt > 0.02f) return false;
    a = wrap(a);
    if (!ready_) {
        previous_ = continuous_ = a;
        ready_ = true; return true;
    }
    const float delta = wrap_signed(a - previous_);
    if (std::abs(delta) > std::max(0.15f, 240.0f * dt)) return false;
    if (a - previous_ > pi) --turns_;
    else if (a - previous_ < -pi) ++turns_;
    continuous_ = turns_ * tau + a;
    const float measured_velocity = delta / dt;
    velocity_ += (dt / (0.01f + dt)) * (measured_velocity - velocity_);
    // Damping and positive-work limiting must follow a reversal promptly.
    // Keep the quieter 10 ms estimate for telemetry and speed-envelope gating.
    // A 2 ms filter delayed elastic damping enough to sustain a ~100 Hz
    // reversal on the unloaded rotor. At the 2.5 kHz encoder cadence, 0.5 ms
    // follows that reversal while still smoothing encoder quantization.
    haptic_velocity_ += (dt / (0.0005f + dt)) * (measured_velocity - haptic_velocity_);
    previous_ = a;
    return true;
}
float AngleTracker::haptic_angle(float horizon) const {
    return continuous_ + haptic_velocity_ * std::clamp(horizon, 0.0f, .002f);
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
        // Strictly crossing the threshold advances a detent. At an exact
        // multi-pitch threshold, floor()+1 would advance one extra detent.
        int steps = static_cast<int>(std::min(1000.0f, std::ceil(excess / c.width)));
        if (error < 0) steps = -steps;
        int next = std::clamp(position_ + steps, -1000000, 1000000);
        if (c.mode == Mode::bounded) next = std::clamp(next, c.min_position, c.max_position);
        center_ += (next - position_) * c.width;
        position_ = next;
        error = angle - center_;
    }
    const bool endstop = c.mode == Mode::bounded && ((position_ == c.min_position && error < 0) || (position_ == c.max_position && error > 0));
    // Springs/endstops retain their minimum return damping. Ordinary detents
    // use the configured D term across the entire pitch, as in SmartKnob.
    const float damping = c.mode == Mode::spring || endstop ? std::max(c.damping, elastic_min_damping) : c.damping;
    float iq = -damping * velocity;
    if (c.mode == Mode::spring) iq -= c.strength * (angle - origin_);
    else if (endstop) iq -= c.endstop * error;
    else if (c.mode != Mode::free) {
        // SmartKnob's restoring spring uses the SAME center as snap detection.
        // Crossing snap moves that center by one pitch and flips the spring
        // toward the next detent in this update. Do not wrap to the nearest
        // pitch: that would reverse torque before the logical transition.
        const float dead_zone = std::min(c.width * .2f, pi / 180);
        const float input = -error + std::clamp(error, -dead_zone, dead_zone);
        iq += c.strength * input;
        // Match the reference's geometric idle gate, never a current-amplitude
        // deadband. Even a tiny restoring request outside the zone survives.
        if (input == 0 && std::abs(velocity) <= .05f) iq = 0;
    }
    iq = std::clamp(iq, -c.current_limit, c.current_limit);
    // At fast crossings, current/encoder latency can turn a nominally
    // conservative detent into a motor. Retain the resisting half, but taper
    // positive work only above ordinary hand-turning speed. The same
    // rule prevents a wound-up spring/endstop from repeatedly reaccelerating.
    if (iq * velocity > 0) {
        const bool elastic = c.mode == Mode::spring || endstop;
        const float start = elastic ? 10.0f : detent_assist_fade_start;
        const float stop = elastic ? 25.0f : detent_assist_cutoff;
        iq *= std::clamp((stop - std::abs(velocity)) / (stop - start), 0.0f, 1.0f);
    }
    return iq;
}
} // namespace knob
