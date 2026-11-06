#include "calibration_quality.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace knob;

static SavedCalibrationQuality verified() {
    SavedCalibrationQuality saved;
    saved.calibration = {true, -1, 7, 6.03f};
    saved.encoder_id = static_cast<uint32_t>(EncoderKind::mt6701);
    saved.sense_gain = .15f;
    auto& q = saved.quality;
    q.method = calibration_quality_method;
    q.current = .08f;
    q.voltage_limit = 1.2f;
    q.measured_pole_pairs = q.forward_pole_pairs = q.reverse_pole_pairs = 7;
    q.direction_difference = .02f;
    q.hysteresis_max = .05f;
    q.residual_rms = .03f;
    q.residual_max = .08f;
    q.verification_max = .07f;
    q.forward_points = q.reverse_points = CalibrationFit::point_count;
    q.verification_points = calibration_verification_points;
    q.phase_check_passed = true;
    q.noise_a = {.003f, .004f, .005f};
    q.sem_a = {.0003f, .0004f, .0005f};
    q.drift_a = {.001f, .0015f, .002f};
    return saved;
}

static bool matches(const SavedCalibrationQuality& saved) {
    const auto reference = verified();
    return quality_matches(saved, reference.calibration, reference.encoder_id, reference.sense_gain);
}

static void test_identity_binding() {
    const auto reference = verified();
    assert(matches(reference));
    assert(valid_calibration_quality(reference.quality));
    auto other = reference;
    other.calibration.electrical_zero = std::nextafter(other.calibration.electrical_zero, tau);
    assert(!matches(other)); // Even one float step must not inherit old evidence.
    other = reference; other.calibration.direction = 1; assert(!matches(other));
    other = reference; other.calibration.pole_pairs = 8; assert(!matches(other));
    other = reference; other.calibration.valid = false; assert(!matches(other));
    other = reference; other.encoder_id = static_cast<uint32_t>(EncoderKind::as5600); assert(!matches(other));
    other = reference; other.sense_gain = .3f; assert(!matches(other));
    assert(!quality_matches(reference, other.calibration, reference.encoder_id, .3f));
    Calibration invalid = reference.calibration;
    invalid.valid = false;
    assert(!quality_matches(reference, invalid, reference.encoder_id, reference.sense_gain));
    for (float zero : {-1.0f, tau, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
        other = reference;
        other.calibration.electrical_zero = zero;
        assert(!quality_matches(other, other.calibration, other.encoder_id, other.sense_gain));
    }
    for (int direction : {0, 2, -2}) {
        other = reference; other.calibration.direction = direction;
        assert(!quality_matches(other, other.calibration, other.encoder_id, other.sense_gain));
    }
    for (uint32_t encoder : {0U, 3U, 0xffffffffU}) {
        other = reference; other.encoder_id = encoder;
        assert(!quality_matches(other, other.calibration, encoder, other.sense_gain));
    }
    for (float gain : {0.0f, .2f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
        other = reference; other.sense_gain = gain;
        assert(!quality_matches(other, other.calibration, other.encoder_id, gain));
    }
    for (auto encoder : {EncoderKind::as5600, EncoderKind::mt6701}) for (float gain : {.15f, .3f, .6f, 1.2f}) {
        other = reference; other.encoder_id = static_cast<uint32_t>(encoder); other.sense_gain = gain;
        assert(quality_matches(other, other.calibration, other.encoder_id, gain));
    }
}

static void test_method_and_complete_evidence() {
    auto saved = verified();
    for (uint32_t method : {0U, 2U, 0xffffffffU}) {
        saved = verified(); saved.quality.method = method; assert(!matches(saved));
    }
    saved = verified(); saved.quality = {}; assert(!matches(saved));
    assert(saved.calibration.valid); // Legacy calibration can remain usable but unverified.
    for (auto member : {&CalibrationQuality::forward_points, &CalibrationQuality::reverse_points}) {
        for (uint32_t count : {0U, 28U, 30U, 0xffffffffU}) {
            saved = verified(); saved.quality.*member = count; assert(!matches(saved));
        }
    }
    for (uint32_t count : {0U, 5U, 7U, 0xffffffffU}) {
        saved = verified(); saved.quality.verification_points = count; assert(!matches(saved));
    }
    saved = verified(); saved.quality.phase_check_passed = false; assert(!matches(saved));
}

static void test_numerical_validation() {
    const float bad_values[] = {-1.0f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
                                -std::numeric_limits<float>::infinity()};
    for (auto member : {&CalibrationQuality::current, &CalibrationQuality::voltage_limit,
                       &CalibrationQuality::measured_pole_pairs, &CalibrationQuality::forward_pole_pairs,
                       &CalibrationQuality::reverse_pole_pairs, &CalibrationQuality::direction_difference,
                       &CalibrationQuality::hysteresis_max, &CalibrationQuality::residual_rms,
                       &CalibrationQuality::residual_max, &CalibrationQuality::verification_max}) {
        for (float bad : bad_values) {
            auto saved = verified(); saved.quality.*member = bad; assert(!matches(saved));
        }
    }
    for (auto member : {&CalibrationQuality::noise_a, &CalibrationQuality::sem_a, &CalibrationQuality::drift_a}) {
        for (unsigned phase = 0; phase < 3; ++phase) for (float bad : bad_values) {
            auto saved = verified(); (saved.quality.*member)[phase] = bad; assert(!matches(saved));
        }
    }
    for (unsigned phase = 0; phase < 3; ++phase) {
        auto saved = verified();
        saved.quality.noise_a[phase] = CurrentOffsetLimits{}.maximum_noise_a + .001f; assert(!matches(saved));
        saved = verified(); saved.quality.sem_a[phase] = CurrentOffsetLimits{}.maximum_standard_error_a + .001f; assert(!matches(saved));
        saved = verified(); saved.quality.drift_a[phase] = CurrentOffsetLimits{}.maximum_drift_a + .001f; assert(!matches(saved));
        saved = verified();
        saved.quality.noise_a[phase] = CurrentOffsetLimits{}.maximum_noise_a;
        saved.quality.sem_a[phase] = CurrentOffsetLimits{}.maximum_standard_error_a;
        saved.quality.drift_a[phase] = CurrentOffsetLimits{}.maximum_drift_a;
        assert(matches(saved));
    }
    for (float current : {.019f, .251f}) {
        auto saved = verified(); saved.quality.current = current; assert(!matches(saved));
    }
    for (float voltage : {.049f, 3.001f}) {
        auto saved = verified(); saved.quality.voltage_limit = voltage; assert(!matches(saved));
    }
    for (float current : {.02f, .25f}) for (float voltage : {.05f, 3.0f}) {
        auto saved = verified(); saved.quality.current = current; saved.quality.voltage_limit = voltage; assert(matches(saved));
    }
    for (auto member : {&CalibrationQuality::measured_pole_pairs, &CalibrationQuality::forward_pole_pairs,
                       &CalibrationQuality::reverse_pole_pairs}) {
        for (float poles : {6.0f, 8.0f}) {
            auto saved = verified(); saved.quality.*member = poles; assert(!matches(saved));
        }
    }
    auto saved = verified(); saved.quality.measured_pole_pairs = 7.01f; assert(!matches(saved));
    saved = verified(); saved.quality.direction_difference = CalibrationFit::maximum_direction_difference + .001f; assert(!matches(saved));
    saved = verified(); saved.quality.hysteresis_max = CalibrationFit::maximum_hysteresis + .001f; assert(!matches(saved));
    saved = verified(); saved.quality.residual_rms = CalibrationFit::maximum_residual_rms + .001f; assert(!matches(saved));
    saved = verified(); saved.quality.residual_max = CalibrationFit::maximum_residual + .001f; assert(!matches(saved));
    saved = verified(); saved.quality.verification_max = calibration_verification_maximum + .001f; assert(!matches(saved));
    saved = verified(); saved.quality.residual_max = .01f; assert(!matches(saved)); // RMS cannot exceed maximum.
}

int main() {
    test_identity_binding();
    test_method_and_complete_evidence();
    test_numerical_validation();
    std::cout << "Calibration quality identity and validation tests passed\n";
}
