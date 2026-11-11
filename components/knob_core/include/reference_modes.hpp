#pragma once
#include "control.hpp"

namespace knob {
// Existing ESP32 mode IDs 0..3 are unchanged. New IDs are first + preset.
constexpr int reference_mode_first = 4;
constexpr int reference_mode_count = 13;
constexpr int reference_config_count = 12;
constexpr unsigned reference_magnetic_capacity = 8;
constexpr float reference_current_limit = max_motor_current;
constexpr int reference_config_version = 2;
const char* reference_mode_name(int preset);

// Kept outside Config so existing electrical settings and calibration blobs
// retain their layout. Each ordinary reference mode owns one configuration.
// Gains use the original ESP32 detent units: A/rad and A/(rad/s).
// Each mode owns its interior and endpoint damping. Clicks are opt-in.
struct ReferenceModeConfig {
    int position = 0, min_position = 0, max_position = -1;
    float width = pi / 18, endstop = 1, snap = .55f, bias = 0;
    float strength = 0, damping = 0, scale = .0875f, friction = 0, click = 0;
    bool bounded = false;
    std::array<int, reference_magnetic_capacity> magnetic_positions{};
    unsigned magnetic_count = 0;
    float endstop_damping = .04f;
    bool click_enabled = false;
};
// Exact v21 on-flash layout. Never reinterpret its scaled gains as v2 units.
struct ReferenceModeConfigV1 {
    int position = 0, min_position = 0, max_position = -1;
    float width = pi / 18, endstop = 1, snap = .55f, bias = 0;
    float strength = 0, damping = 0, scale = .0875f, friction = 0, click = 0;
    bool bounded = false;
    std::array<int, reference_magnetic_capacity> magnetic_positions{};
    unsigned magnetic_count = 0;
};
using ReferenceModeBankV1 = std::array<ReferenceModeConfigV1, reference_config_count>;
static_assert(sizeof(ReferenceModeConfigV1) == 88);
ReferenceModeBankV1 legacy_reference_modes();
using ReferenceModeBank = std::array<ReferenceModeConfig, reference_config_count>;
bool migrate_reference_modes(const ReferenceModeBankV1& old, ReferenceModeBank& migrated);
// Invalid indices return the harmless Custom default; validation rejects them.
const ReferenceModeConfig& default_reference_config(int preset);
ReferenceModeBank default_reference_modes();
const char* validate_reference_config(const ReferenceModeConfig& config, int preset);
bool same_reference_config(const ReferenceModeConfig& left, const ReferenceModeConfig& right);

// Default RollerCAN SmartKnob presets, in sensor mechanical coordinates.
// Positive mechanical rotation decreases the reference's logical position.
// Preset 12 is implemented by the separate frictionless controller.
class ReferenceHaptics {
public:
    // Configuration is validated on reset, outside the repeated control step.
    // Call reset after applying changed parameters, including for the same mode.
    void reset(float angle, int preset, const ReferenceModeConfig& config);
    KNOB_IRAM float step(int preset, const ReferenceModeConfig& config,
                         float angle, float velocity, float dt, float current_limit);
    // Default wrappers retain source compatibility for old tests and callers.
    void reset(float angle, int preset);
    KNOB_IRAM float step(int preset, float angle, float velocity,
                         float dt, float current_limit);
    int position() const { return position_; }
    float sub_position() const { return sub_position_; }
private:
    int preset_ = -1;
    int position_ = 0;
    float center_ = 0;
    float sub_position_ = 0;
    float idle_velocity_ = 0;
    float idle_elapsed_ = 0;
    float click_elapsed_ = 0;
    float click_direction_ = 1;
    bool click_active_ = false;
    bool high_speed_ = false;
    bool config_valid_ = false;
};
} // namespace knob
