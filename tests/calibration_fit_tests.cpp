#include "calibration_fit.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

using namespace knob;

static float angular_error(float a, float b) { return std::abs(std::remainder(a - b, tau)); }
static bool near(float a, float b, float tolerance = 1e-4f) { return std::abs(a - b) < tolerance; }
static void expect_error(const CalibrationFitResult& result, const char* error) {
    assert(!result.valid() && !result.calibration.valid);
    if (!result.error || std::strcmp(result.error, error)) {
        std::cerr << "Expected " << error << ", got " << (result.error ? result.error : "success") << '\n';
        assert(false);
    }
}

template<typename Error>
static CalibrationFit make_scan(int direction, float poles, float zero, Error error, float turns = 0) {
    CalibrationFit fit;
    for (unsigned pass = 0; pass < 2; ++pass) {
        // Hardware can naturally collect the reverse pass in descending order.
        for (unsigned j = 0; j < CalibrationFit::point_count; ++j) {
            const unsigned index = pass ? CalibrationFit::point_count - 1 - j : j;
            const float electrical = CalibrationFit::command_angle(index);
            const float mechanical = (electrical + zero + error(pass, index)) / (direction * poles) + turns * tau;
            assert(fit.add(pass != 0, index, mechanical));
        }
    }
    return fit;
}
static CalibrationFit exact_scan(int direction = 1, float poles = 7, float zero = 1.3f, float turns = 0) {
    return make_scan(direction, poles, zero, [](unsigned, unsigned) { return 0.0f; }, turns);
}

static void test_correct_mapping() {
    for (int direction : {-1, 1}) for (float zero : {0.0f, .003f, 1.3f, tau - .003f}) {
        for (float turns : {-5.0f, 0.0f, 9.0f}) {
            const auto result = exact_scan(direction, 7, zero, turns).evaluate();
            assert(result.valid());
            assert(result.calibration.direction == direction && result.calibration.pole_pairs == 7);
            assert(angular_error(result.calibration.electrical_zero, zero) < .00015f);
            assert(near(result.forward_pole_pairs, 7) && near(result.reverse_pole_pairs, 7));
            assert(near(result.coverage, tau));
            assert(result.residual_rms < .0001f && result.residual_max < .0001f);
        }
    }
}

static void test_bidirectional_average_and_small_encoder_error() {
    const auto scan = make_scan(-1, 7, tau - .003f, [](unsigned pass, unsigned index) {
        // Repeatable full-turn encoder distortion plus opposite friction bias.
        return .07f * std::sin(CalibrationFit::command_angle(index) / 7) + (pass ? -.025f : .025f);
    });
    const auto result = scan.evaluate();
    assert(result.valid());
    assert(angular_error(result.calibration.electrical_zero, tau - .003f) < .001f);
    assert(near(result.direction_difference, .05f));
    assert(near(result.hysteresis_max, .05f));
    assert(result.residual_rms > .04f && result.residual_rms < .07f);
    // Points either side of 0/2pi must average near zero, not near pi.
    const auto wrap_result = make_scan(1, 7, .001f, [](unsigned pass, unsigned) {
        return pass ? -.02f : .02f;
    }).evaluate();
    assert(wrap_result.valid() && angular_error(wrap_result.calibration.electrical_zero, .001f) < .0001f);
}

static void test_wrong_motor_and_missing_motion() {
    for (int direction : {-1, 1}) for (float poles : {6.0f, 8.0f})
        expect_error(exact_scan(direction, poles).evaluate(), "calibration_pole_pairs_mismatch");
    CalibrationFit no_motion;
    for (unsigned pass = 0; pass < 2; ++pass) for (unsigned index = 0; index < CalibrationFit::point_count; ++index)
        assert(no_motion.add(pass != 0, index, 2.0f));
    expect_error(no_motion.evaluate(), "calibration_no_motion");
    CalibrationFit opposite_directions;
    for (unsigned pass = 0; pass < 2; ++pass) for (unsigned index = 0; index < CalibrationFit::point_count; ++index)
        assert(opposite_directions.add(pass != 0, index, (pass ? -1 : 1) * CalibrationFit::command_angle(index) / 7));
    expect_error(opposite_directions.evaluate(), "calibration_direction_mismatch");
    // Insufficient endpoint travel can hide behind an acceptable global slope.
    auto insufficient = make_scan(1, 7, 0, [](unsigned, unsigned index) {
        return index == 0 ? .055f * tau * 7 : index == CalibrationFit::point_count - 1 ? -.055f * tau * 7 : 0.0f;
    });
    expect_error(insufficient.evaluate(), "calibration_coverage");
}

static void test_collection_contract() {
    CalibrationFit fit;
    expect_error(fit.evaluate(), "calibration_incomplete");
    for (unsigned index = 0; index < CalibrationFit::point_count; ++index) {
        assert(fit.add(false, index, CalibrationFit::command_angle(index) / 7));
        if (index != 12) assert(fit.add(true, index, CalibrationFit::command_angle(index) / 7));
    }
    assert(fit.forward_count() == 29 && fit.reverse_count() == 28);
    expect_error(fit.evaluate(), "calibration_incomplete");
    assert(fit.add(true, 12, CalibrationFit::command_angle(12) / 7));
    assert(fit.evaluate().valid());
    assert(!fit.add(true, 12, 99));
    expect_error(fit.evaluate(), "calibration_point_duplicate");
    assert(!fit.add(false, 0, 0)); // Failure remains latched until explicit reset.
    fit.reset();
    assert(!fit.collection_error() && fit.forward_count() == 0 && fit.reverse_count() == 0);
    for (float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
                      -std::numeric_limits<float>::infinity()}) {
        fit.reset();
        assert(!fit.add(false, 0, bad));
        expect_error(fit.evaluate(), "calibration_point_invalid");
    }
    fit.reset();
    assert(!fit.add(false, CalibrationFit::point_count, 0));
    expect_error(fit.evaluate(), "calibration_point_invalid");
}

static void test_quality_rejections() {
    auto directional_bias = make_scan(1, 7, .2f, [](unsigned pass, unsigned) { return pass ? -.08f : .08f; });
    expect_error(directional_bias.evaluate(), "calibration_direction_bias");
    auto local_hysteresis = make_scan(1, 7, .2f, [](unsigned pass, unsigned index) {
        return !pass && index == 14 ? .32f : 0.0f;
    });
    expect_error(local_hysteresis.evaluate(), "calibration_hysteresis");
    auto outlier = make_scan(1, 7, .2f, [](unsigned, unsigned index) { return index == 14 ? .5f : 0.0f; });
    expect_error(outlier.evaluate(), "calibration_residual");
    auto periodic_error = make_scan(1, 7, .2f, [](unsigned, unsigned index) {
        return .22f * std::cos(CalibrationFit::command_angle(index) / 7);
    });
    expect_error(periodic_error.evaluate(), "calibration_residual");
    // A whole electrical-turn slip has a perfect circular zero and would be
    // missed by a modulo-only residual check. Midpoint keeps the slope ideal.
    auto pole_slip = make_scan(1, 7, .2f, [](unsigned, unsigned index) { return index == 14 ? tau : 0.0f; });
    const auto slipped = pole_slip.evaluate();
    assert(near(slipped.pole_pairs, 7) && slipped.residual_max > 6);
    expect_error(slipped, "calibration_residual");
    auto ambiguous = make_scan(1, 7, .2f, [](unsigned, unsigned index) { return index % 2 ? -pi / 2 : pi / 2; });
    expect_error(ambiguous.evaluate(), "calibration_zero_ambiguous");
}

static void test_heldout_points() {
    for (int direction : {-1, 1}) {
        const auto calibration = exact_scan(direction, 7, 6.27f).evaluate().calibration;
        for (float command : {-.41f, .71f, 9.83f, 43.57f}) {
            const float mechanical = (command + calibration.electrical_zero) / (direction * 7);
            assert(near(CalibrationFit::verify_point(calibration, command, mechanical), 0));
            assert(near(CalibrationFit::verify_point(calibration, command, mechanical + .1f / (direction * 7)), .1f));
        }
    }
    assert(std::isnan(CalibrationFit::verify_point({}, 0, 0)));
    const auto calibrated = exact_scan().evaluate().calibration;
    assert(std::isnan(CalibrationFit::verify_point(calibrated, std::numeric_limits<float>::quiet_NaN(), 0)));
}

int main() {
    test_correct_mapping();
    test_bidirectional_average_and_small_encoder_error();
    test_wrong_motor_and_missing_motion();
    test_collection_contract();
    test_quality_rejections();
    test_heldout_points();
    std::cout << "Bidirectional calibration fit tests passed\n";
}
