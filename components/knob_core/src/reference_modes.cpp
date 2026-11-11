#include "reference_modes.hpp"
#include <algorithm>
#include <cmath>

namespace knob {
namespace {
// Derived from m5-rollercan_smart-knob/MyFile/src/smart_knob_modes.c and
// smart_knob.c. The reference credits scottbez1/smartknob for its detent
// geometry. These are software presets, not electrical/motor calibrations.
// p and d are the reference's pre-scale gains; multiplying by scale converts
// the PID result to amperes. detent_strength_unit is descriptive in that
// controller: its actual restoring gain is tuning.p_gain.
struct Preset {
    const char* name;
    int position, minimum, maximum;
    float width_degrees, endstop, snap, bias;
    float p, d, scale, friction, click;
};
constexpr Preset legacy_presets[] = {
    {"Custom",             0,  0, -1, 10, 1, .55f, 0,   0, 0,     .0875f, 0,     0},
    {"Unbounded smooth",   0,  0, -1, 10, 1, .75f, 0,   0, 0,     .0375f, .02f,  0},
    {"Bounded 0-10",       0,  0, 10, 10, 1, 1.1f, 0,   0, 0,     .0625f, 0,     0},
    {"Multi-rev smooth",   0,  0, 72, 10, 5, .75f, 0,   0, 0,     .0375f, 0,     0},
    {"On/off strong",      0,  0,  1, 60, 1, .55f, 0,  38, .55f,  .1f,    0,     0},
    {"Return to center",   0,  0,  0, 60, .6f,1.1f,0,  40, .1f,   .2f,    .0075f,0},
    {"Fine smooth",      127,  0,255,  1, 1, 1.1f, 0,   0, .1f,   .075f,  0,     0},
    {"Fine detents",     127,  0,255,  1, 1, .9f,  0,   0, .1f,   .0625f, 0,     .1f},
    {"Coarse strong",      0,  0, 31, 10, 1, .75f, 0,  28, .16f,  .2f,    0,     0},
    {"Coarse weak",        0,  0, 31, 10, 1, .9f,  0,   5, .16f,  .2f,    0,     .35f},
    {"Magnetic detents",   0,  0, 31,  7, 1, .7f,  0,  40, .2f,   .2f,    0,     0},
    {"Center detents",     0, -6,  6, 60, 1, .55f,.4f, 10, .1f,   .2f,    0,     0},
};
constexpr float degrees = pi / 180.0f;
constexpr float idle_speed = .05f;
// Convert the reference's legacy per-tick EWMAs to time constants. The
// continuous exponential gives the same decay at any ESP32 control cadence.
constexpr float idle_velocity_tau = .19633034f;
constexpr float idle_center_tau = .39275892f;
bool ordinary_preset(int preset) { return preset >= 0 && preset < reference_config_count; }
constexpr ReferenceModeBankV1 make_legacy_defaults() {
    ReferenceModeBankV1 result{};
    for (int i = 0; i < reference_config_count; ++i) {
        const auto& p = legacy_presets[i];
        auto& c = result[i];
        c.position = p.position; c.min_position = p.minimum; c.max_position = p.maximum;
        c.width = p.width_degrees * degrees; c.endstop = p.endstop; c.snap = p.snap; c.bias = p.bias;
        c.strength = p.p; c.damping = p.d; c.scale = p.scale; c.friction = p.friction; c.click = p.click;
        c.bounded = p.minimum <= p.maximum;
    }
    result[10].magnetic_positions = {2, 10, 21, 22};
    result[10].magnetic_count = 4;
    return result;
}
constexpr ReferenceModeBankV1 old_defaults = make_legacy_defaults();
constexpr ReferenceModeBank make_defaults() {
    ReferenceModeBank result{};
    // Preserve mode identities/geometry; use physical gains with scale=1.
    constexpr float strength[] = {20, 0, 0, 0, 5, 0, 0, 60, 20, 5, 20, 2};
    constexpr float damping[] = {.003f, 0, 0, 0, .003f, .04f, 0, .003f, .003f, .003f, .003f, .003f};
    for (int i = 0; i < reference_config_count; ++i) {
        const auto& old = old_defaults[i]; auto& c = result[i];
        c.position = old.position; c.min_position = old.min_position; c.max_position = old.max_position;
        c.width = old.width; c.bounded = old.bounded; c.bias = old.bias;
        c.snap = strength[i] > 0 ? .55f : old.snap;
        c.strength = strength[i]; c.damping = damping[i]; c.scale = 1;
        c.endstop = i == 5 ? .5f : 8.0f; c.endstop_damping = .04f;
        c.friction = 0; c.click = .08f; c.click_enabled = false;
        c.magnetic_positions = old.magnetic_positions; c.magnetic_count = old.magnetic_count;
    }
    return result;
}
constexpr ReferenceModeBank defaults = make_defaults();
bool in_range(float value, float minimum, float maximum) {
    return std::isfinite(value) && value >= minimum && value <= maximum;
}
bool valid_position(int value) { return value >= -10000 && value <= 10000; }
bool magnetic_position(const ReferenceModeConfig& c, int position) {
    for (unsigned i = 0; i < c.magnetic_count && i < c.magnetic_positions.size(); ++i)
        if (position == c.magnetic_positions[i]) return true;
    return false;
}
} // namespace

const ReferenceModeConfig& default_reference_config(int preset) {
    return defaults[ordinary_preset(preset) ? preset : 0];
}
ReferenceModeBank default_reference_modes() { return defaults; }

const char* validate_reference_config(const ReferenceModeConfig& c, int preset) {
    if (!ordinary_preset(preset)) return "invalid_reference_mode";
    if (!valid_position(c.position) || !valid_position(c.min_position) ||
        !valid_position(c.max_position) ||
        (c.bounded && (c.min_position > c.max_position || c.position < c.min_position || c.position > c.max_position)))
        return "invalid_reference_positions";
    if (!in_range(c.width, degrees, pi)) return "invalid_reference_width";
    if (!in_range(c.endstop, 0, 20)) return "invalid_reference_endstop";
    if (!in_range(c.snap, .5f, 2) || !in_range(c.bias, 0, 1.5f) || c.bias > c.snap - .05f)
        return "invalid_reference_snap";
    if (!in_range(c.strength, 0, max_haptic_strength)) return "invalid_reference_strength";
    if (!in_range(c.damping, 0, .5f)) return "invalid_reference_damping";
    if (!in_range(c.endstop_damping, 0, .5f)) return "invalid_reference_endstop_damping";
    if (!in_range(c.scale, 0, 1)) return "invalid_reference_scale";
    if (!in_range(c.friction, 0, .45f)) return "invalid_reference_friction";
    if (!in_range(c.click, 0, .45f)) return "invalid_reference_click";
    if (c.magnetic_count > c.magnetic_positions.size() || (preset != 10 && c.magnetic_count != 0))
        return "invalid_reference_magnets";
    for (unsigned i = 0; i < c.magnetic_count; ++i) {
        const int position = c.magnetic_positions[i];
        if (!valid_position(position) || (c.bounded && (position < c.min_position || position > c.max_position)))
            return "invalid_reference_magnets";
        for (unsigned j = 0; j < i; ++j)
            if (position == c.magnetic_positions[j]) return "invalid_reference_magnets";
    }
    return nullptr;
}

bool same_reference_config(const ReferenceModeConfig& a, const ReferenceModeConfig& b) {
    if (a.position != b.position || a.min_position != b.min_position || a.max_position != b.max_position ||
        a.width != b.width || a.endstop != b.endstop || a.snap != b.snap || a.bias != b.bias ||
        a.strength != b.strength || a.damping != b.damping || a.scale != b.scale ||
        a.friction != b.friction || a.click != b.click || a.bounded != b.bounded ||
        a.endstop_damping != b.endstop_damping || a.click_enabled != b.click_enabled ||
        a.magnetic_count != b.magnetic_count || a.magnetic_count > a.magnetic_positions.size()) return false;
    for (unsigned i = 0; i < a.magnetic_count; ++i)
        if (a.magnetic_positions[i] != b.magnetic_positions[i]) return false;
    return true;
}

ReferenceModeBankV1 legacy_reference_modes() { return old_defaults; }
bool migrate_reference_modes(const ReferenceModeBankV1& old, ReferenceModeBank& migrated) {
    ReferenceModeBank candidate = defaults;
    for (int i = 0; i < reference_config_count; ++i) {
        const auto& v = old[i]; const auto& d = old_defaults[i];
        if (!in_range(v.strength, 0, 100) || !in_range(v.damping, 0, 5) ||
            !in_range(v.endstop, 0, 20) || !in_range(v.scale, 0, 1)) return false;
        auto copy = [](const ReferenceModeConfigV1& from) {
            ReferenceModeConfig c{};
            c.position=from.position; c.min_position=from.min_position; c.max_position=from.max_position;
            c.width=from.width; c.endstop=from.endstop; c.snap=from.snap; c.bias=from.bias;
            c.strength=from.strength; c.damping=std::min(from.damping, .5f); c.scale=from.scale;
            c.friction=from.friction; c.click=from.click; c.bounded=from.bounded;
            c.magnetic_positions=from.magnetic_positions; c.magnetic_count=from.magnetic_count;
            return c;
        };
        auto c = copy(v);
        if (validate_reference_config(c, i)) return false;
        // Untouched v1 presets adopt the new feel. Customized modes retain
        // their geometry and converted physical gains; clicks start disabled.
        if (same_reference_config(c, copy(d)) && v.damping == d.damping) continue;
        c.strength = v.strength * v.scale;
        c.endstop = std::min(20.0f, 4 * v.endstop * v.scale);
        c.damping = std::min(.5f, v.damping * v.scale);
        c.endstop_damping = c.damping;
        c.scale = 1; c.click_enabled = false;
        if (validate_reference_config(c, i)) return false;
        candidate[i] = c;
    }
    migrated = candidate;
    return true;
}

const char* reference_mode_name(int preset) {
    if (ordinary_preset(preset)) return legacy_presets[preset].name;
    return preset == 12 ? "Frictionless" : "Unknown";
}

void ReferenceHaptics::reset(float angle, int preset) {
    reset(angle, preset, default_reference_config(preset));
}
void ReferenceHaptics::reset(float angle, int preset, const ReferenceModeConfig& config) {
    *this = {};
    preset_ = preset;
    center_ = std::isfinite(angle) ? angle : 0;
    config_valid_ = !validate_reference_config(config, preset);
    position_ = config_valid_ ? config.position : 0;
}

float ReferenceHaptics::step(int preset, float angle, float velocity,
                              float dt, float current_limit) {
    return step(preset, default_reference_config(preset), angle, velocity, dt, current_limit);
}
float ReferenceHaptics::step(int preset, const ReferenceModeConfig& p, float angle, float velocity,
                              float dt, float current_limit) {
    if (!ordinary_preset(preset) || !std::isfinite(angle) ||
        !std::isfinite(velocity) || !std::isfinite(dt) || dt <= 0 || dt > .02f ||
        !std::isfinite(current_limit) || current_limit <= 0) {
        click_active_ = false;
        return 0;
    }
    if (preset != preset_) reset(angle, preset, p);
    if (!config_valid_ || !std::isfinite(p.width) || p.width <= 0) return 0;
    const float width = p.width;
    const bool bounded = p.bounded;
    if (click_active_) click_elapsed_ += dt;

    // Preserve the reference's absolute bounded lattice; only its unbounded
    // presets slowly re-center at rest. ESP32 owns encoder timing/filtering.
    if (!bounded) {
        idle_velocity_ += (1.0f - std::exp(-dt / idle_velocity_tau)) *
                          (std::abs(velocity) - idle_velocity_);
        idle_elapsed_ = idle_velocity_ > idle_speed ? 0 : idle_elapsed_ + dt;
        if (idle_elapsed_ > .5f && std::abs(angle - center_) < 5 * degrees)
            center_ += (1.0f - std::exp(-dt / idle_center_tau)) * (angle - center_);
    }

    float error = angle - center_;
    const int previous_position = position_;
    // The reference processes at most 16 transitions per step. This remains
    // bounded for malformed input, with position saturation for unbounded modes.
    for (int transition = 0; transition < 16; ++transition) {
        const float decrease = width * (p.snap + (position_ <= 0 ? p.bias : -p.bias));
        const float increase = -width * (p.snap + (position_ >= 0 ? p.bias : -p.bias));
        if (error > decrease && (bounded ? position_ > p.min_position : position_ > -1000000)) {
            center_ += width; error -= width; --position_;
        } else if (error < increase && (bounded ? position_ < p.max_position : position_ < 1000000)) {
            center_ -= width; error += width; ++position_;
        } else break;
    }
    sub_position_ = -error / width;
    const bool endstop = bounded &&
        ((error > 0 && position_ == p.min_position) || (error < 0 && position_ == p.max_position));
    const bool magnetic_gap = !endstop && preset == 10 && !magnetic_position(p, position_);
    // Original ESP32 force law. Endpoints have no center dead zone and use
    // this mode's explicit endpoint damping (no hidden shared D term).
    float requested = p.scale * (endstop
        ? -p.endstop * error - p.endstop_damping * velocity
        : detent_current(magnetic_gap ? 0 : error, velocity, width, p.strength, p.damping));
    if (std::abs(velocity) > idle_speed)
        requested += p.friction * std::copysign(std::atan(std::abs(velocity) / .5f) / (pi * .5f), velocity);

    const bool click_allowed = p.click_enabled && !endstop && (preset != 10 || !magnetic_gap);
    if (!click_allowed || click_elapsed_ >= .004f) click_active_ = false;
    // Complete both lobes before accepting another crossing. Retriggering
    // the first lobe at high detent rates would create sustained braking.
    if (position_ != previous_position && click_allowed && p.click > 0 && !click_active_) {
        // Oppose the crossing first, then release; deterministic in both directions.
        click_direction_ = position_ > previous_position ? 1.0f : -1.0f;
        click_elapsed_ = 0;
        click_active_ = true;
    }
    const float click = click_active_
        ? (click_elapsed_ < .002f ? click_direction_ : -click_direction_) * p.click : 0;
    requested += click;

    if (std::abs(velocity) >= 180.0f) high_speed_ = true;
    else if (std::abs(velocity) < 120.0f) high_speed_ = false;
    if (high_speed_) return 0;
    return limit_haptic_current(requested, velocity,
                                std::min(current_limit, reference_current_limit), endstop);
}
} // namespace knob
