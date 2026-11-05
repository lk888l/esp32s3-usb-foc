#pragma once
#include "control.hpp"
#include "encoder_types.hpp"
#include <cstddef>
#include <cstdint>

namespace knob {
// Keep the existing blob layout/key so configuration survives sensor changes.
// Version 1 predates selectable encoders and its calibration is AS5600-only.
// The former user voltage limit occupies a reserved float. Its bytes remain
// part of the legacy CRC, but it is neither validated nor used by control.
static_assert(sizeof(Config) == 56);
static_assert(offsetof(Config, legacy_voltage_limit) == 12);
static_assert(offsetof(Config, current_kp) == 16);
constexpr uint32_t encoder_settings_version(uint32_t encoder_id) {
    return 0x10000U | encoder_id;
}
constexpr bool known_settings_version(uint32_t version) {
    return version == 1 ||
           version == encoder_settings_version(static_cast<uint32_t>(EncoderKind::as5600)) ||
           version == encoder_settings_version(static_cast<uint32_t>(EncoderKind::mt6701));
}
constexpr bool settings_calibration_matches(uint32_t version, uint32_t encoder_id) {
    if (version == 1) return encoder_id == static_cast<uint32_t>(EncoderKind::as5600);
    return known_settings_version(version) && version == encoder_settings_version(encoder_id);
}
inline bool restore_encoder_settings(uint32_t version, const Config& saved_config,
                                     const Calibration& saved_calibration, uint32_t encoder_id,
                                     Config& config, Calibration& calibration) {
    if (!known_settings_version(version) || validate(saved_config) ||
        (saved_calibration.valid && !valid_calibration(saved_calibration))) return false;
    config = saved_config;
    calibration = settings_calibration_matches(version, encoder_id) ? saved_calibration : Calibration{};
    return true;
}
} // namespace knob
