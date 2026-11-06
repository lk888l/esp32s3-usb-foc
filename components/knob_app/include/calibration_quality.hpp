#pragma once
#include "calibration_fit.hpp"
#include "current_offsets.hpp"
#include "encoder_types.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <type_traits>

namespace knob {

// Versioned evidence for a completed calibration, separate from the existing
// Calibration/NVS layout. Method zero means that no such evidence is present.
// This record does not claim external calibration of the current-sense gain.
struct CalibrationQuality {
    uint32_t method = 0;
    float current = 0, voltage_limit = 0;
    float measured_pole_pairs = 0, forward_pole_pairs = 0, reverse_pole_pairs = 0;
    float direction_difference = 0, hysteresis_max = 0;
    float residual_rms = 0, residual_max = 0, verification_max = 0;
    uint32_t forward_points = 0, reverse_points = 0, verification_points = 0;
    bool phase_check_passed = false;
    std::array<float, 3> noise_a{}, sem_a{}, drift_a{};
};

// CRC, storage length and atomic NVS operations are the protocol's concern.
// These identity fields prevent an old sidecar from certifying a different
// calibration after settings were replaced but metadata was not.
struct SavedCalibrationQuality {
    Calibration calibration{};
    uint32_t encoder_id = 0;
    float sense_gain = 0;
    CalibrationQuality quality{};
};

static_assert(std::is_trivially_copyable_v<CalibrationQuality>);
static_assert(std::is_standard_layout_v<CalibrationQuality>);
static_assert(std::is_trivially_copyable_v<SavedCalibrationQuality>);
static_assert(std::is_standard_layout_v<SavedCalibrationQuality>);

constexpr uint32_t calibration_quality_method = 1;
constexpr uint32_t calibration_verification_points = 6;
constexpr float calibration_verification_maximum = .20f;
constexpr float calibration_minimum_current = .02f;
constexpr float calibration_maximum_current = .25f;
constexpr float calibration_minimum_voltage = .05f;
constexpr float calibration_maximum_voltage = 3.0f;

inline bool valid_calibration_quality(const CalibrationQuality& quality) {
    const auto within = [](float value, float minimum, float maximum) {
        return std::isfinite(value) && value >= minimum && value <= maximum;
    };
    const auto valid_poles = [&](float poles) {
        return std::isfinite(poles) &&
            std::abs(poles / CalibrationFit::expected_pole_pairs - 1) <= CalibrationFit::maximum_pole_pair_relative_error;
    };
    if (quality.method != calibration_quality_method || !quality.phase_check_passed ||
        quality.forward_points != CalibrationFit::point_count || quality.reverse_points != CalibrationFit::point_count ||
        quality.verification_points != calibration_verification_points ||
        !within(quality.current, calibration_minimum_current, calibration_maximum_current) ||
        !within(quality.voltage_limit, calibration_minimum_voltage, calibration_maximum_voltage) ||
        !valid_poles(quality.measured_pole_pairs) || !valid_poles(quality.forward_pole_pairs) ||
        !valid_poles(quality.reverse_pole_pairs) ||
        std::abs(quality.measured_pole_pairs - .5f * (quality.forward_pole_pairs + quality.reverse_pole_pairs)) > 1e-4f ||
        !within(quality.direction_difference, 0, CalibrationFit::maximum_direction_difference) ||
        !within(quality.hysteresis_max, 0, CalibrationFit::maximum_hysteresis) ||
        !within(quality.residual_rms, 0, CalibrationFit::maximum_residual_rms) ||
        !within(quality.residual_max, 0, CalibrationFit::maximum_residual) ||
        quality.residual_rms > quality.residual_max + 1e-5f ||
        !within(quality.verification_max, 0, calibration_verification_maximum)) return false;
    constexpr CurrentOffsetLimits offsets{};
    for (unsigned phase = 0; phase < 3; ++phase) {
        if (!within(quality.noise_a[phase], 0, offsets.maximum_noise_a) ||
            !within(quality.sem_a[phase], 0, offsets.maximum_standard_error_a) ||
            !within(quality.drift_a[phase], 0, offsets.maximum_drift_a)) return false;
    }
    return true;
}

inline bool quality_matches(const SavedCalibrationQuality& saved, const Calibration& calibration,
                            uint32_t encoder_id, float sense_gain) {
    const bool known_encoder = encoder_id == static_cast<uint32_t>(EncoderKind::as5600) ||
                               encoder_id == static_cast<uint32_t>(EncoderKind::mt6701);
    const bool known_gain = sense_gain == .15f || sense_gain == .3f || sense_gain == .6f || sense_gain == 1.2f;
    return known_encoder && known_gain && valid_calibration(calibration) && valid_calibration(saved.calibration) &&
        saved.encoder_id == encoder_id && saved.sense_gain == sense_gain &&
        saved.calibration.valid == calibration.valid && saved.calibration.direction == calibration.direction &&
        saved.calibration.pole_pairs == calibration.pole_pairs &&
        saved.calibration.electrical_zero == calibration.electrical_zero && valid_calibration_quality(saved.quality);
}

// Restore an explicit RAM backup only after the complete replacement and its
// optional evidence pass validation. The motor task owns both destinations,
// so no status reply can expose a new calibration with the old quality record.
inline bool restore_calibration_with_quality(const Calibration& restored,
        const SavedCalibrationQuality* evidence, uint32_t encoder_id, float sense_gain,
        Calibration& calibration, CalibrationQuality& quality) {
    if (!valid_calibration(restored) ||
        (evidence && !quality_matches(*evidence, restored, encoder_id, sense_gain))) return false;
    calibration = restored;
    quality = evidence ? evidence->quality : CalibrationQuality{};
    return true;
}

} // namespace knob
