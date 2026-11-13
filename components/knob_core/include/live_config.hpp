#pragma once
#include "reference_modes.hpp"
#include "frictionless.hpp"

namespace knob {
enum class ConfigState { idle, active, unavailable };
struct LiveConfigChange {
    bool electrical = false, mode = false, current = false;
    bool reference_feel = false, frictionless = false, output = false;
};
inline bool same_frictionless_config(const FrictionlessConfig& a, const FrictionlessConfig& b) {
    return a.friction == b.friction && a.viscous == b.viscous && a.scale == b.scale &&
        a.hold_kp == b.hold_kp && a.hold_ki == b.hold_ki;
}
// Pure preflight: the caller commits only after this validates the entire
// request. No controller, configuration or protection state is mutated here.
inline const char* plan_live_config(ConfigState state, const Config& old, const Config& next,
        const ReferenceModeBank& bank, const ReferenceModeBank& patch, uint16_t mask,
        const FrictionlessConfig& frictionless, const FrictionlessConfig* next_frictionless,
        LiveConfigChange& change) {
    change = {};
    if (state == ConfigState::unavailable) return "idle_or_active_required";
    if (const char* e = validate(next)) return e;
    if (mask & ~((1u << reference_config_count) - 1u)) return "invalid_reference_mode";
    if (next_frictionless)
        if (const char* e = validate_frictionless(*next_frictionless)) return e;
    for (int i = 0; i < reference_config_count; ++i) {
        if (!(mask & (1u << i))) continue;
        if (const char* e = validate_reference_config(patch[i], i)) return e;
    }
    change.electrical = old.bus_voltage != next.bus_voltage || old.sense_gain != next.sense_gain ||
        old.current_kp != next.current_kp || old.current_ki != next.current_ki;
    if (state == ConfigState::active && change.electrical) return "disable_before_electrical_config";
    change.mode = old.mode != next.mode;
    change.current = old.current_limit != next.current_limit;
    change.frictionless = next_frictionless && !same_frictionless_config(frictionless, *next_frictionless);
    const int preset = static_cast<int>(next.mode) - reference_mode_first;
    if (preset >= 0 && preset < reference_config_count && (mask & (1u << preset))) {
        auto previous = bank[preset];
        previous.position = patch[preset].position; // Initial count is a future reset preference.
        change.reference_feel = !same_reference_config(previous, patch[preset]);
    }
    const bool legacy = !reference_mode(next.mode) &&
        (old.width != next.width || old.strength != next.strength || old.damping != next.damping ||
         old.endstop != next.endstop || old.snap != next.snap ||
         old.min_position != next.min_position || old.max_position != next.max_position);
    change.output = change.mode || change.current || change.reference_feel || legacy ||
        (next.mode == Mode::frictionless && change.frictionless);
    return nullptr;
}
} // namespace knob
