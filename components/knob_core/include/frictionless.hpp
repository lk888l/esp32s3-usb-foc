#pragma once
#include "control.hpp"

namespace knob {
// Adapted from m5-rollercan_smart-knob/MyFile/{inc,src}/frictionless_hold.*.
// The reference's motor-specific angle map and inertia estimate are omitted.
// Current is in amperes; viscous loss is A/(rad/s); scale is dimensionless.
struct FrictionlessConfig {
    float friction = 0.022f;
    float viscous = 0.000304f;
    float scale = 1.0f;
    float hold_kp = 0.016f;
    float hold_ki = 0.024f;
};
KNOB_IRAM const char* validate_frictionless(const FrictionlessConfig& config);
namespace frictionless_detail {
typedef enum {
    FRICTIONLESS_HOLD_REST = 0,
    FRICTIONLESS_HOLD_OBSERVE,
    FRICTIONLESS_HOLD_TRACK,
    FRICTIONLESS_HOLD_YIELD,
    FRICTIONLESS_HOLD_ACQUIRE
} FrictionlessHoldPhase;

typedef struct {
    float kp_a_per_rad_s;
    float ki_a_per_rad;
    /* Extra assistance above the loss feedforward, and opposing current.
       The ESP32 adaptation uses 60 mA, bounded by the total current budget.
       Negative feedback may always cancel the entire feedforward. */
    float extra_current_limit_a;
    float start_velocity_rad_s;
    float stop_velocity_rad_s;
    float acceleration_tau_s;
    float gesture_acceleration_rad_s2;
    float gesture_delta_rad_s;
    float gesture_confirm_s;
    float capture_quiet_acceleration_rad_s2;
    float capture_quiet_s;
    float capture_max_s;
    float gesture_arm_error_rad_s;
    float gesture_arm_s;
    float restart_delay_s;
    float startup_acquire_min_s;
    float startup_acquire_max_s;
    float startup_quiet_acceleration_rad_s2;
    float startup_quiet_s;
    float startup_trend_tau_s;
    float clutch_error_rad_s;
    float clutch_current_a;
    float clutch_confirm_s;
    float clutch_fast_current_a;
    float clutch_fast_confirm_s;
    float clutch_current_tau_s;
} FrictionlessHoldConfig;

/* Public state also supplies a debugger/telemetry snapshot. The reference is
   immutable in TRACK; it changes only on entry or a confirmed hand-motion gesture.
   A launch from rest gets one bounded, filtered acquisition opportunity;
   subsequent gestures require the preceding hold to have first settled.
   This is a motion heuristic, not a touch or external-torque measurement. */
typedef struct {
    FrictionlessHoldPhase phase;
    float reference_velocity_rad_s;
    float integral_current_a;
    float correction_current_a;
    float output_current_a;
    float acceleration_rad_s2;
    float filtered_velocity_rad_s;
    float phase_elapsed_s;
    float quiet_elapsed_s;
    float gesture_elapsed_s;
    float gesture_start_velocity_rad_s;
    float gesture_arm_elapsed_s;
    float restart_remaining_s;
    float capture_velocity_rad_s;
    float capture_acceleration_rad_s2;
    float settled_integral_current_a;
    float filtered_correction_current_a;
    float clutch_elapsed_s;
    float clutch_trend_elapsed_s;
    int8_t clutch_trend_direction;
    float yield_deadline_s;
    float yield_bias_current_a;
    int8_t direction;
    int8_t gesture_direction;
    bool velocity_initialized;
    bool control_active;
    bool gesture_ready;
} FrictionlessHoldState;

KNOB_IRAM FrictionlessHoldConfig frictionless_hold_default_config(void);
KNOB_IRAM void frictionless_hold_reset(FrictionlessHoldState *state);

/* Return the complete MEAN current, before the independent angle ripple term.
   velocity is the encoder's already-filtered rad/s measurement. feedforward
   is the signed nominal loss current. current_limit is the effective current
   budget after all external limits; zero budget clears the controller.

   enabled=false observes motion without accumulating an integral/reference.
   The first enabled sample captures the smoothed measured velocity, permitting
   a diagnostic speed-driver -> normal-controller handover without stale goals.
   Invalid input, a discontinuous dt, stop, and reversal clear the old goal. */
KNOB_IRAM float frictionless_hold_step(FrictionlessHoldState *state,
                            const FrictionlessHoldConfig *config,
                            float velocity_rad_s,
                            float feedforward_a,
                            float current_limit_a,
                            float dt_s,
                            bool enabled);


} // namespace frictionless_detail
class Frictionless {
public:
    KNOB_IRAM void reset();
    // Input is encoder velocity in sensor coordinates. Invalid input, >2 ms
    // time gaps clear every stored torque/goal. A zero available-current
    // budget mutes output and freezes the hold goal/integral for at most
    // 20 ms; longer interruptions clear the goal before reacquiring motion.
    // drive_override observes motion but returns zero; releasing it captures
    // the actual filtered speed instead of a diagnostic requested speed.
    KNOB_IRAM float step(const FrictionlessConfig& config, float velocity,
                         float dt, float current_limit, bool drive_override = false);
    int phase() const { return static_cast<int>(hold_.phase); }
    float reference() const { return hold_.reference_velocity_rad_s; }
    float correction() const { return hold_.correction_current_a; }
    float feedforward() const { return feedforward_; }
    float velocity() const { return velocity_; }
private:
    frictionless_detail::FrictionlessHoldState hold_{};
    float velocity_ = 0, feedforward_ = 0, arm_s_ = 0, restart_s_ = 0;
    float zero_budget_s_ = 0;
    int direction_ = 0;
    bool initialized_ = false, armed_ = false;
};
} // namespace knob
