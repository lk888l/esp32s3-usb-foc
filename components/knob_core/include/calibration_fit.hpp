#pragma once
#include "control.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace knob {

struct CalibrationFitResult {
    Calibration calibration{};
    const char* error = "calibration_incomplete";
    float forward_pole_pairs = 0, reverse_pole_pairs = 0, pole_pairs = 0;
    float forward_zero = 0, reverse_zero = 0;
    // All errors below are electrical radians; coverage is mechanical radians.
    float direction_difference = 0, hysteresis_max = 0;
    float residual_rms = 0, residual_max = 0, coverage = 0;
    float concentration = 0;
    bool valid() const { return error == nullptr && calibration.valid; }
};

// A complete forward and reverse scan of one mechanical revolution. Each
// point is a settled angle mean at the same commanded electrical grid, with
// the continuous encoder coordinate preserved across both passes. Collection
// does not infer or repair missing samples, reorder the motor, or unwrap data.
// evaluate() is a one-time fit, not work for every current-control tick.
class CalibrationFit {
public:
    static constexpr int expected_pole_pairs = 7;
    static constexpr unsigned points_per_electrical_turn = 4;
    static constexpr unsigned point_count = expected_pole_pairs * points_per_electrical_turn + 1;
    static constexpr float maximum_pole_pair_relative_error = .03f;
    static constexpr float maximum_residual_rms = .12f;
    static constexpr float maximum_residual = .35f;
    static constexpr float maximum_direction_difference = .12f;
    static constexpr float maximum_hysteresis = .30f;
    static constexpr float minimum_coverage = .90f * tau;
    static constexpr float maximum_coverage = 1.10f * tau;
    static constexpr float minimum_concentration = .90f;

    static constexpr float command_angle(unsigned index) {
        return index * (tau / points_per_electrical_turn);
    }

    void reset() { *this = {}; }

    bool add(bool reverse, unsigned index, float unwrapped_mechanical) {
        if (collection_error_) return false;
        if (index >= point_count || !std::isfinite(unwrapped_mechanical)) {
            collection_error_ = "calibration_point_invalid";
            return false;
        }
        const unsigned pass = reverse ? 1U : 0U;
        if (present_[pass][index]) {
            collection_error_ = "calibration_point_duplicate";
            return false;
        }
        angles_[pass][index] = unwrapped_mechanical;
        present_[pass][index] = true;
        ++counts_[pass];
        return true;
    }

    unsigned forward_count() const { return counts_[0]; }
    unsigned reverse_count() const { return counts_[1]; }
    const char* collection_error() const { return collection_error_; }

    // Residual for a NEW held-out alignment point, in electrical radians.
    // This checks position agreement, not current calibration or torque.
    static float verify_point(const Calibration& calibration, float electrical_command,
                              float mechanical_angle) {
        if (!calibration.valid || (calibration.direction != -1 && calibration.direction != 1) ||
            calibration.pole_pairs != static_cast<int>(expected_pole_pairs) ||
            !std::isfinite(calibration.electrical_zero) || calibration.electrical_zero < 0 ||
            calibration.electrical_zero >= tau || !std::isfinite(electrical_command) ||
            !std::isfinite(mechanical_angle)) return std::numeric_limits<float>::quiet_NaN();
        return signed_angle(calibration.direction * expected_pole_pairs * mechanical_angle -
                            electrical_command - calibration.electrical_zero);
    }

    CalibrationFitResult evaluate() const {
        CalibrationFitResult result;
        if (collection_error_) { result.error = collection_error_; return result; }
        if (counts_[0] != point_count || counts_[1] != point_count) return result;

        const float forward_slope = mechanical_slope(angles_[0]);
        const float reverse_slope = mechanical_slope(angles_[1]);
        const float forward_span = angles_[0].back() - angles_[0].front();
        const float reverse_span = angles_[1].back() - angles_[1].front();
        result.coverage = std::min(std::abs(forward_span), std::abs(reverse_span));
        if (!std::isfinite(forward_slope) || !std::isfinite(reverse_slope) ||
            std::abs(forward_slope) < 1e-5f || std::abs(reverse_slope) < 1e-5f) {
            result.error = "calibration_no_motion";
            return result;
        }
        const int direction = forward_slope > 0 ? 1 : -1;
        result.calibration.direction = direction;
        result.forward_pole_pairs = 1 / std::abs(forward_slope);
        result.reverse_pole_pairs = 1 / std::abs(reverse_slope);
        result.pole_pairs = .5f * (result.forward_pole_pairs + result.reverse_pole_pairs);
        if (direction * reverse_slope <= 0 || direction * forward_span <= 0 || direction * reverse_span <= 0) {
            result.error = "calibration_direction_mismatch";
            return result;
        }

        // Reference subtraction keeps the fit accurate after many accumulated
        // turns. It also retains pole slips: only the final zero is circular;
        // residuals and paired hysteresis NEVER wrap away an extra 2*pi.
        const float anchor = angles_[0][0];
        std::array<std::array<float, point_count>, 2> phases{};
        std::array<float, 2> sine{}, cosine{};
        for (unsigned pass = 0; pass < 2; ++pass) {
            for (unsigned index = 0; index < point_count; ++index) {
                const float phase = direction * expected_pole_pairs * (angles_[pass][index] - anchor) -
                                    command_angle(index);
                if (!std::isfinite(phase)) { result.error = "calibration_point_invalid"; return result; }
                phases[pass][index] = phase;
                sine[pass] += std::sin(phase);
                cosine[pass] += std::cos(phase);
            }
        }
        const float forward_mean = std::atan2(sine[0], cosine[0]);
        const float reverse_mean = std::atan2(sine[1], cosine[1]);
        const float mean = std::atan2(sine[0] + sine[1], cosine[0] + cosine[1]);
        const float base_zero = positive_angle(direction * expected_pole_pairs * anchor);
        result.forward_zero = positive_angle(base_zero + forward_mean);
        result.reverse_zero = positive_angle(base_zero + reverse_mean);
        result.calibration.electrical_zero = positive_angle(base_zero + mean);
        result.direction_difference = std::abs(signed_angle(forward_mean - reverse_mean));
        result.concentration = std::hypot(sine[0] + sine[1], cosine[0] + cosine[1]) / (2 * point_count);
        float squared_residual = 0;
        for (unsigned index = 0; index < point_count; ++index) {
            result.hysteresis_max = std::max(result.hysteresis_max,
                std::abs(direction * expected_pole_pairs * (angles_[0][index] - angles_[1][index])));
            for (unsigned pass = 0; pass < 2; ++pass) {
                const float residual = phases[pass][index] - mean;
                squared_residual += residual * residual;
                result.residual_max = std::max(result.residual_max, std::abs(residual));
            }
        }
        result.residual_rms = std::sqrt(squared_residual / (2 * point_count));

        // Fixed commissioning limits. A failed run cannot widen its own
        // acceptance bounds. Both passes must individually agree with 7 pp.
        if (std::abs(result.forward_pole_pairs / expected_pole_pairs - 1) > maximum_pole_pair_relative_error ||
            std::abs(result.reverse_pole_pairs / expected_pole_pairs - 1) > maximum_pole_pair_relative_error)
            result.error = "calibration_pole_pairs_mismatch";
        else if (result.coverage < minimum_coverage || std::abs(forward_span) > maximum_coverage ||
                 std::abs(reverse_span) > maximum_coverage)
            result.error = "calibration_coverage";
        else if (!std::isfinite(result.concentration) || result.concentration < minimum_concentration)
            result.error = "calibration_zero_ambiguous";
        else if (result.hysteresis_max > maximum_hysteresis)
            result.error = "calibration_hysteresis";
        else if (result.direction_difference > maximum_direction_difference)
            result.error = "calibration_direction_bias";
        else if (!std::isfinite(result.residual_rms) || result.residual_rms > maximum_residual_rms ||
                 result.residual_max > maximum_residual)
            result.error = "calibration_residual";
        else {
            result.error = nullptr;
            result.calibration.valid = true;
        }
        return result;
    }

private:
    std::array<std::array<float, point_count>, 2> angles_{};
    std::array<std::array<bool, point_count>, 2> present_{};
    std::array<unsigned, 2> counts_{};
    const char* collection_error_ = nullptr;

    static float positive_angle(float angle) {
        angle = std::fmod(angle, tau);
        if (angle < 0) angle += tau;
        // Rounding a tiny negative angle up to exactly tau must still produce
        // the half-open interval required by the persisted Calibration type.
        return angle < tau ? angle : 0;
    }
    static float signed_angle(float angle) { return std::remainder(angle, tau); }

    static float mechanical_slope(const std::array<float, point_count>& angles) {
        const float command_mean = command_angle((point_count - 1) / 2);
        const float anchor = angles[0];
        float mechanical_mean = 0;
        for (float angle : angles) mechanical_mean += angle - anchor;
        mechanical_mean /= point_count;
        float covariance = 0, command_variance = 0;
        for (unsigned index = 0; index < point_count; ++index) {
            const float command_delta = command_angle(index) - command_mean;
            covariance += command_delta * (angles[index] - anchor - mechanical_mean);
            command_variance += command_delta * command_delta;
        }
        return covariance / command_variance;
    }
};

} // namespace knob
