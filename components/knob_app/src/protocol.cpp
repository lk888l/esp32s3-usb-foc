#include "protocol.hpp"
#include "board.hpp"
#include "encoder_config.hpp"
#include "settings_identity.hpp"
#include "nvs.h"
#include "esp_crc.h"
#include "esp_system.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <cstring>
#include <cstddef>
#include <algorithm>
namespace knob {
const char* state_name(State s) {
    constexpr const char* names[] = {"idle", "offsets", "align", "sweep", "settle", "active", "fault"};
    return names[static_cast<int>(s)];
}
namespace {
struct Field { const char* name; float Config::*member; };
constexpr Field fields[] = {{"bus_voltage", &Config::bus_voltage}, {"sense_gain", &Config::sense_gain},
    {"current_limit", &Config::current_limit},
    {"current_kp", &Config::current_kp}, {"current_ki", &Config::current_ki}, {"width", &Config::width},
    {"strength", &Config::strength}, {"endstop", &Config::endstop}, {"damping", &Config::damping}, {"snap", &Config::snap}};
void array(cJSON* obj, const char* name, const std::array<float, 3>& values) {
    cJSON_AddItemToObject(obj, name, cJSON_CreateFloatArray(values.data(), values.size()));
}
template <typename T, size_t N>
void numeric_array(cJSON* obj, const char* name, const std::array<T, N>& values) {
    std::array<double, N> numbers{};
    for (size_t i = 0; i < N; ++i) numbers[i] = values[i];
    cJSON_AddItemToObject(obj, name, cJSON_CreateDoubleArray(numbers.data(), N));
}
struct Stored { uint32_t version; Config config; Calibration calibration; uint32_t crc; };
struct StoredQuality { SavedCalibrationQuality record; uint32_t crc; };
void quality_fields(cJSON* object, const CalibrationQuality& q) {
    cJSON_AddNumberToObject(object, "current", q.current);
    cJSON_AddNumberToObject(object, "voltage_limit", q.voltage_limit);
    cJSON_AddNumberToObject(object, "forward_points", q.forward_points);
    cJSON_AddNumberToObject(object, "reverse_points", q.reverse_points);
    cJSON_AddNumberToObject(object, "verification_points", q.verification_points);
    cJSON_AddNumberToObject(object, "pole_pairs", q.measured_pole_pairs);
    cJSON_AddNumberToObject(object, "forward_pole_pairs", q.forward_pole_pairs);
    cJSON_AddNumberToObject(object, "reverse_pole_pairs", q.reverse_pole_pairs);
    cJSON_AddNumberToObject(object, "direction_difference", q.direction_difference);
    cJSON_AddNumberToObject(object, "hysteresis_max", q.hysteresis_max);
    cJSON_AddNumberToObject(object, "residual_rms", q.residual_rms);
    cJSON_AddNumberToObject(object, "residual_max", q.residual_max);
    cJSON_AddNumberToObject(object, "verification_max", q.verification_max);
    cJSON_AddBoolToObject(object, "phase_check_passed", q.phase_check_passed);
    array(object, "offset_noise_a", q.noise_a);
    array(object, "offset_sem_a", q.sem_a);
    array(object, "offset_drift_a", q.drift_a);
}
}
const char* decode_request(const char* line, Request& r) {
    const char* end = nullptr;
    cJSON* root = cJSON_ParseWithOpts(line, &end, true);
    if (!root) return "invalid_json";
    const auto fail = [&](const char* reason) { cJSON_Delete(root); return reason; };
    if (!cJSON_IsObject(root)) return fail("object_required");
    const auto* id = cJSON_GetObjectItemCaseSensitive(root, "id");
    if (!cJSON_IsNumber(id) || !std::isfinite(id->valuedouble) || id->valuedouble < 0 || id->valuedouble > 2147483647 || std::floor(id->valuedouble) != id->valuedouble) return fail("invalid_id");
    r.id = id->valueint;
    const auto* op = cJSON_GetObjectItemCaseSensitive(root, "op");
    if (!cJSON_IsString(op)) return fail("invalid_op");
    struct NamedOp { const char* name; Op op; };
    constexpr NamedOp names[] = {{"status", Op::status}, {"configure", Op::configure}, {"calibrate", Op::calibrate},
        {"enable", Op::enable}, {"disable", Op::disable}, {"clear", Op::clear}, {"zero", Op::zero}, {"pulse", Op::pulse}, {"save", Op::save}
#if CONFIG_KNOB_DIAGNOSTICS
        , {"diagnose", Op::diagnose}, {"diagnostic_read", Op::diagnostic_read}, {"diagnostic_hold", Op::diagnostic_hold},
        {"diagnostic_restore_calibration", Op::diagnostic_restore_calibration}
#endif
    };
    bool found = false;
    for (auto n : names) if (!std::strcmp(op->valuestring, n.name)) { r.op = n.op; found = true; }
    if (!found) return fail("unknown_op");
    if (r.op == Op::calibrate) {
        struct Parameter { const char* name; float CalibrationPlan::*member; };
        constexpr Parameter parameters[] = {{"current", &CalibrationPlan::current},
            {"max_current", &CalibrationPlan::max_current}, {"voltage", &CalibrationPlan::voltage_limit}};
        for (const auto parameter : parameters) {
            const auto* item = cJSON_GetObjectItemCaseSensitive(root, parameter.name);
            if (!item) continue;
            if (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble)) return fail("calibration_parameters");
            r.calibration_plan.*(parameter.member) = static_cast<float>(item->valuedouble);
        }
        if (const char* error = validate_calibration_plan(r.calibration_plan)) return fail(error);
    }
#if CONFIG_KNOB_DIAGNOSTICS
    if (r.op == Op::diagnostic_restore_calibration) {
        const auto* encoder = cJSON_GetObjectItemCaseSensitive(root, "encoder");
        const auto* cal = cJSON_GetObjectItemCaseSensitive(root, "calibration");
        if (!cJSON_IsString(encoder) || std::strcmp(encoder->valuestring, encoder_config::name) ||
            !cJSON_IsObject(cal)) return fail("calibration_identity");
        const auto* valid = cJSON_GetObjectItemCaseSensitive(cal, "valid");
        const auto* direction = cJSON_GetObjectItemCaseSensitive(cal, "direction");
        const auto* poles = cJSON_GetObjectItemCaseSensitive(cal, "pole_pairs");
        const auto* zero = cJSON_GetObjectItemCaseSensitive(cal, "electrical_zero");
        if (!cJSON_IsTrue(valid) || !cJSON_IsNumber(direction) || !cJSON_IsNumber(poles) ||
            !cJSON_IsNumber(zero) || !std::isfinite(zero->valuedouble) ||
            (direction->valuedouble != -1 && direction->valuedouble != 1) || poles->valuedouble != 7)
            return fail("invalid_calibration");
        r.diagnostic_calibration = {true, direction->valueint, poles->valueint, static_cast<float>(zero->valuedouble)};
        if (!valid_calibration(r.diagnostic_calibration)) return fail("invalid_calibration");
        r.diagnostic_quality_present = false;
        r.diagnostic_quality = {};
        if (const auto* quality = cJSON_GetObjectItemCaseSensitive(root, "calibration_quality")) {
            if (!cJSON_IsObject(quality)) return fail("invalid_calibration_quality");
            const auto* method = cJSON_GetObjectItemCaseSensitive(quality, "method");
            const auto* verified = cJSON_GetObjectItemCaseSensitive(quality, "verified");
            const auto* phase_check = cJSON_GetObjectItemCaseSensitive(quality, "phase_check_passed");
            const auto* gain = cJSON_GetObjectItemCaseSensitive(root, "sense_gain");
            if (!cJSON_IsString(method) || std::strcmp(method->valuestring, "bidirectional_v1") ||
                !cJSON_IsTrue(verified) || !cJSON_IsTrue(phase_check) ||
                !cJSON_IsNumber(gain) || !std::isfinite(gain->valuedouble))
                return fail("invalid_calibration_quality");
            auto& saved = r.diagnostic_quality;
            saved.calibration = r.diagnostic_calibration;
            saved.encoder_id = encoder_config::calibration_id;
            saved.sense_gain = static_cast<float>(gain->valuedouble);
            auto& q = saved.quality;
            q.method = calibration_quality_method;
            q.phase_check_passed = true;
            struct QualityNumber { const char* name; float CalibrationQuality::*member; };
            constexpr QualityNumber numbers[] = {{"current", &CalibrationQuality::current},
                {"voltage_limit", &CalibrationQuality::voltage_limit}, {"pole_pairs", &CalibrationQuality::measured_pole_pairs},
                {"forward_pole_pairs", &CalibrationQuality::forward_pole_pairs}, {"reverse_pole_pairs", &CalibrationQuality::reverse_pole_pairs},
                {"direction_difference", &CalibrationQuality::direction_difference}, {"hysteresis_max", &CalibrationQuality::hysteresis_max},
                {"residual_rms", &CalibrationQuality::residual_rms}, {"residual_max", &CalibrationQuality::residual_max},
                {"verification_max", &CalibrationQuality::verification_max}};
            for (const auto field : numbers) {
                const auto* item = cJSON_GetObjectItemCaseSensitive(quality, field.name);
                if (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble)) return fail("invalid_calibration_quality");
                q.*(field.member) = static_cast<float>(item->valuedouble);
            }
            struct QualityCount { const char* name; uint32_t CalibrationQuality::*member; };
            constexpr QualityCount counts[] = {{"forward_points", &CalibrationQuality::forward_points},
                {"reverse_points", &CalibrationQuality::reverse_points}, {"verification_points", &CalibrationQuality::verification_points}};
            for (const auto field : counts) {
                const auto* item = cJSON_GetObjectItemCaseSensitive(quality, field.name);
                if (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble) ||
                    item->valuedouble < 0 || item->valuedouble > UINT32_MAX ||
                    std::floor(item->valuedouble) != item->valuedouble) return fail("invalid_calibration_quality");
                q.*(field.member) = static_cast<uint32_t>(item->valuedouble);
            }
            struct QualityArray { const char* name; std::array<float, 3> CalibrationQuality::*member; };
            constexpr QualityArray arrays[] = {{"offset_noise_a", &CalibrationQuality::noise_a},
                {"offset_sem_a", &CalibrationQuality::sem_a}, {"offset_drift_a", &CalibrationQuality::drift_a}};
            for (const auto field : arrays) {
                const auto* values = cJSON_GetObjectItemCaseSensitive(quality, field.name);
                if (!cJSON_IsArray(values) || cJSON_GetArraySize(values) != 3) return fail("invalid_calibration_quality");
                for (int phase = 0; phase < 3; ++phase) {
                    const auto* item = cJSON_GetArrayItem(values, phase);
                    if (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble)) return fail("invalid_calibration_quality");
                    (q.*(field.member))[phase] = static_cast<float>(item->valuedouble);
                }
            }
            if (!quality_matches(saved, r.diagnostic_calibration, encoder_config::calibration_id, saved.sense_gain))
                return fail("invalid_calibration_quality");
            r.diagnostic_quality_present = true;
        }
    }
    if (r.op == Op::diagnostic_hold) {
        const auto* current = cJSON_GetObjectItemCaseSensitive(root, "current");
        if (!cJSON_IsNumber(current) || !std::isfinite(current->valuedouble) ||
            std::abs(current->valuedouble) < .04 || std::abs(current->valuedouble) > .25)
            return fail("diagnostic_hold_current_range");
        r.diagnostic.current = static_cast<float>(current->valuedouble);
    }
    if (r.op == Op::diagnose) {
        const auto* axis = cJSON_GetObjectItemCaseSensitive(root, "axis");
        if (axis && (!cJSON_IsString(axis) ||
            (std::strcmp(axis->valuestring, "d") && std::strcmp(axis->valuestring, "q"))))
            return fail("diagnostic_axis");
        const bool d_axis = axis && !std::strcmp(axis->valuestring, "d");
        float electrical_offset = 0;
        if (const auto* offset = cJSON_GetObjectItemCaseSensitive(root, "electrical_offset")) {
            if (!cJSON_IsNumber(offset) || !std::isfinite(offset->valuedouble) ||
                std::abs(offset->valuedouble) > pi / 6 || (!d_axis && offset->valuedouble != 0))
                return fail("diagnostic_electrical_offset");
            electrical_offset = static_cast<float>(offset->valuedouble);
        }
        const char* keys[] = {"mode", "velocity", "current", "drive_ms", "observe_ms"};
        double values[5]{};
        for (int i = 0; i < 5; ++i) {
            const auto* item = cJSON_GetObjectItemCaseSensitive(root, keys[i]);
            // Positive d injection needs no velocity stimulus or haptic mode.
            if (d_axis && !item && i < 2) continue;
            if (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble)) return fail("diagnostic_fields_required");
            values[i] = item->valuedouble;
        }
        if (values[0] < 0 || values[0] > 3 || std::floor(values[0]) != values[0] ||
            (d_axis ? values[1] != 0 : std::abs(values[1]) < 10 || std::abs(values[1]) > 60) ||
            values[2] < (d_axis ? .05 : .02) || values[2] > (d_axis ? 3 : .25) ||
            values[3] < 50 || values[3] > (d_axis ? 30000 : 500) ||
            values[4] < (d_axis ? 100 : 500) || values[4] > (d_axis ? 1000 : 3000))
            return fail("diagnostic_range");
        r.diagnostic = {static_cast<Mode>(static_cast<int>(values[0])), static_cast<float>(values[1]),
            static_cast<float>(values[2]), static_cast<int>(values[3] * 1000), static_cast<int>(values[4] * 1000),
            d_axis ? DiagnosticAxis::d : DiagnosticAxis::q, electrical_offset};
    }
    if (r.op == Op::diagnostic_read) {
        const auto* item = cJSON_GetObjectItemCaseSensitive(root, "offset");
        if (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble) || item->valuedouble < 0 ||
            item->valuedouble > diagnostic_capacity || std::floor(item->valuedouble) != item->valuedouble)
            return fail("diagnostic_offset");
        r.diagnostic_offset = item->valueint;
    }
#endif
    if (r.op == Op::configure) {
        const auto* c = cJSON_GetObjectItemCaseSensitive(root, "config");
        if (!cJSON_IsObject(c)) return fail("config_required");
        // Voltage capacity follows the live bus. Reject an old client's knob
        // explicitly, instead of silently pretending that its limit applies.
        if (cJSON_GetObjectItemCaseSensitive(c, "voltage_limit")) return fail("voltage_limit_automatic");
        // Complete replacements avoid lost partial updates across reconnects.
        for (auto f : fields) {
            const auto* item = cJSON_GetObjectItemCaseSensitive(c, f.name);
            if (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble)) return fail("config_field_missing");
            r.config.*(f.member) = static_cast<float>(item->valuedouble);
        }
        for (auto key : {"min_position", "max_position", "mode"}) {
            const auto* item = cJSON_GetObjectItemCaseSensitive(c, key);
            if (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble) || std::abs(item->valuedouble) > 10000 || std::floor(item->valuedouble) != item->valuedouble) return fail("integer_required");
            if (!std::strcmp(key, "min_position")) r.config.min_position = item->valueint;
            else if (!std::strcmp(key, "max_position")) r.config.max_position = item->valueint;
            else r.config.mode = static_cast<Mode>(item->valueint);
        }
        if (const char* error = validate(r.config)) return fail(error);
    }
    cJSON_Delete(root); return nullptr;
}
cJSON* encode_reply(int id, const Reply& r) {
    cJSON* root = cJSON_CreateObject(); if (!root) return nullptr;
    const auto& s = r.snapshot;
    cJSON_AddNumberToObject(root, "id", id);
    cJSON_AddStringToObject(root, "device", "smartknob-s3");
    cJSON_AddNumberToObject(root, "protocol", 1);
    cJSON_AddNumberToObject(root, "control_stack_free", s.control_stack_free);
    cJSON_AddNumberToObject(root, "console_stack_free", uxTaskGetStackHighWaterMark(nullptr));
    cJSON_AddStringToObject(root, "firmware", "dma-v20-15k-sampling");
    cJSON_AddStringToObject(root, "pwm_profile", pwm_profile_name);
    cJSON_AddNumberToObject(root, "adc_controller_hz", sampling_timing::adc_controller_hz);
    cJSON_AddNumberToObject(root, "adc_sar_hz", sampling_timing::adc_sar_hz);
    cJSON_AddNumberToObject(root, "adc_start_target_tick", sampling_timing::start_tick);
    cJSON_AddNumberToObject(root, "adc_minimum_compare_ticks", sampling_timing::minimum_compare_ticks);
    const auto* app = esp_app_get_description();
    char app_sha256[65]{};
    constexpr char hex[] = "0123456789abcdef";
    for (size_t i = 0; i < sizeof(app->app_elf_sha256); ++i) {
        app_sha256[2 * i] = hex[app->app_elf_sha256[i] >> 4];
        app_sha256[2 * i + 1] = hex[app->app_elf_sha256[i] & 15];
    }
    cJSON_AddStringToObject(root, "app_elf_sha256", app_sha256);
    cJSON_AddStringToObject(root, "current_sampling", Board::staggered_sampling ?
        "pwm_staggered_adc_done_experimental" : "pwm_triggered_adc_gdma");
    cJSON_AddNumberToObject(root, "current_phase_skew_us", s.phase_skew_us);
    cJSON_AddBoolToObject(root, "adc_sample_valid", s.adc_sample_valid);
    cJSON_AddNumberToObject(root, "adc_reject_reasons", s.adc_reject_reasons);
    cJSON* reject_counts = cJSON_AddObjectToObject(root, "adc_reject_counts");
    constexpr const char* reject_names[] = {
        "transport", "trigger", "pwm_tracking", "burst_timing", "phase_window", "pair_change"};
    for (size_t i = 0; i < s.adc_reject_counts.size(); ++i)
        cJSON_AddNumberToObject(reject_counts, reject_names[i], s.adc_reject_counts[i]);
    cJSON_AddBoolToObject(root, "sampling_restart_required", s.sampling_restart_required);
    cJSON_AddNumberToObject(root, "adc_completed_mask", s.adc_completed_mask);
    cJSON_AddNumberToObject(root, "adc_done_mask", s.adc_done_mask);
    cJSON_AddNumberToObject(root, "adc_done_raw_mask", s.adc_done_raw_mask);
    cJSON_AddNumberToObject(root, "adc_done_down_mask", s.adc_done_down_mask);
    numeric_array(root, "adc_done_ticks", s.adc_done_ticks);
    numeric_array(root, "adc_done_words", s.adc_done_words);
    numeric_array(root, "adc_armed_words", s.adc_armed_words);
    numeric_array(root, "adc_current_words", s.adc_current_words);
    numeric_array(root, "adc_done_elapsed_us", s.adc_done_elapsed_us);
    cJSON_AddNumberToObject(root, "adc_window_after_mask", s.adc_window_after_mask);
    numeric_array(root, "adc_trigger_elapsed_us", s.adc_trigger_elapsed_us);
    auto* before = cJSON_AddArrayToObject(root, "adc_pwm_compare_before");
    auto* after = cJSON_AddArrayToObject(root, "adc_pwm_compare_after");
    for (int slot = 0; slot < 3; ++slot) {
        auto* before_row = cJSON_CreateArray(); auto* after_row = cJSON_CreateArray();
        for (int p = 0; p < 3; ++p) {
            cJSON_AddItemToArray(before_row, cJSON_CreateNumber(s.adc_pwm_compare_before[slot][p]));
            cJSON_AddItemToArray(after_row, cJSON_CreateNumber(s.adc_pwm_compare_after[slot][p]));
        }
        cJSON_AddItemToArray(before, before_row); cJSON_AddItemToArray(after, after_row);
    }
    cJSON_AddNumberToObject(root, "control_nominal_hz", Board::control_hz);
    cJSON_AddNumberToObject(root, "encoder_nominal_hz", Board::encoder_hz);
    cJSON_AddNumberToObject(root, "pwm_hz", Board::pwm_hz);
    cJSON_AddNumberToObject(root, "pwm_actual_hz", Board::pwm_actual_hz);
    cJSON_AddNumberToObject(root, "control_carrier_divider", Board::control_divider);
    cJSON_AddNumberToObject(root, "encoder_carrier_divider", Board::encoder_divider);
    cJSON_AddNumberToObject(root, "pwm_half_period_ticks", Board::half_period);
    cJSON_AddNumberToObject(root, "pwm_resolution_hz", Board::resolution);
    cJSON_AddNumberToObject(root, "pwm_duty_floor", pwm_zero_clamp ? 0 : duty_floor);
    cJSON_AddNumberToObject(root, "pwm_min_active_duty", duty_floor);
    cJSON_AddBoolToObject(root, "pwm_zero_clamp", pwm_zero_clamp);
    cJSON_AddNumberToObject(root, "pwm_duty_ceiling", duty_ceiling);
    cJSON_AddBoolToObject(root, "voltage_limit_automatic", true);
    cJSON_AddNumberToObject(root, "max_voltage_limit", max_output_voltage(s.bus_voltage_used));
    cJSON_AddNumberToObject(root, "available_voltage_limit", max_output_voltage(s.bus_voltage_used));
    cJSON_AddNumberToObject(root, "configured_max_voltage_limit", max_output_voltage(s.config.bus_voltage));
    cJSON_AddNumberToObject(root, "effective_voltage_limit", s.effective_voltage_limit);
    cJSON_AddBoolToObject(root, "bus_voltage_sensing", BusVoltageMonitor::enabled);
    cJSON_AddStringToObject(root, "bus_voltage_source", BusVoltageMonitor::enabled ? "adc2_gpio11" : "configured");
    cJSON_AddNumberToObject(root, "bus_voltage", s.bus_voltage_used);
    cJSON_AddBoolToObject(root, "bus_voltage_valid", s.bus_voltage_valid);
    cJSON_AddNumberToObject(root, "bus_voltage_age_us", s.bus_voltage_age_us);
    cJSON_AddNumberToObject(root, "bus_voltage_raw", s.bus_sample.raw);
    cJSON_AddNumberToObject(root, "bus_voltage_adc_mv", s.bus_sample.millivolts);
    cJSON_AddNumberToObject(root, "bus_voltage_measured", s.bus_sample.volts);
    cJSON_AddNumberToObject(root, "bus_voltage_error", s.bus_sample.error);
    cJSON_AddNumberToObject(root, "bus_voltage_divider", BusVoltageMonitor::divider_ratio);
    cJSON_AddStringToObject(root, "detent_profile", "smart_knob");
    cJSON_AddBoolToObject(root, "detent_quiet_velocity", false);
    cJSON_AddBoolToObject(root, "haptic_position_filter", false);
    cJSON_AddBoolToObject(root, "current_target_slew", false);
    cJSON_AddNumberToObject(root, "host_lease_ms", CONFIG_KNOB_HOST_LEASE_MS);
    cJSON_AddNumberToObject(root, "current_feedforward_ohms", CONFIG_KNOB_PHASE_RESISTANCE_MILLIOHM * .001);
    cJSON_AddNumberToObject(root, "reset_reason", esp_reset_reason());
    cJSON_AddNumberToObject(root, "uptime_ms", esp_timer_get_time() / 1000);
    cJSON_AddNumberToObject(root, "minimum_free_heap", esp_get_minimum_free_heap_size());
    cJSON_AddNumberToObject(root, "current_limit_max", max_motor_current);
    cJSON_AddNumberToObject(root, "strength_max", max_haptic_strength);
    cJSON_AddNumberToObject(root, "current_absolute_trip", absolute_current_trip);
    cJSON_AddNumberToObject(root, "current_peak_trip", peak_current_trip);
    cJSON_AddNumberToObject(root, "current_peak_trip_ms", current_peak_trip_us / 1000.0);
    cJSON_AddNumberToObject(root, "current_overload_trip_ms", current_overload_trip_us / 1000.0);
    cJSON_AddNumberToObject(root, "current_trip_recovery_ratio", current_trip_recovery_ratio);
    cJSON_AddNumberToObject(root, "current_trip_limit", s.current_trip_limit);
    cJSON_AddNumberToObject(root, "current_target_limit", s.current_target_limit);
    cJSON_AddBoolToObject(root, "current_limited", s.current_limited);
    cJSON_AddBoolToObject(root, "quiet_output", s.quiet_output);
    cJSON_AddNumberToObject(root, "haptic_scale", s.haptic_scale);
    cJSON_AddBoolToObject(root, "speed_limited", s.speed_limited);
    cJSON_AddBoolToObject(root, "haptic_paused", s.haptic_paused);
    cJSON_AddNumberToObject(root, "haptic_fade_start", haptic_fade_start);
    cJSON_AddNumberToObject(root, "haptic_speed_cutoff", haptic_speed_cutoff);
    cJSON_AddNumberToObject(root, "haptic_resume_speed", haptic_resume_speed);
    cJSON_AddNumberToObject(root, "haptic_resume_hold_ms", haptic_resume_hold_s * 1000);
    cJSON_AddNumberToObject(root, "haptic_fade_in_ms", haptic_fade_in_s * 1000);
    cJSON_AddNumberToObject(root, "detent_assist_fade_start", detent_assist_fade_start);
    cJSON_AddNumberToObject(root, "detent_assist_cutoff", detent_assist_cutoff);
    cJSON_AddNumberToObject(root, "elastic_min_damping", elastic_min_damping);
    cJSON_AddBoolToObject(root, "fault_current_valid", s.fault_current_valid);
    cJSON_AddNumberToObject(root, "fault_current_trip_limit", s.fault_current_trip_limit);
    array(root, "fault_currents", s.fault_currents);
    if (s.fault_current_valid) {
        cJSON* motion = cJSON_AddObjectToObject(root, "fault_motion");
        cJSON_AddNumberToObject(motion, "angle", s.fault_angle);
        cJSON_AddNumberToObject(motion, "velocity", s.fault_velocity);
        cJSON_AddNumberToObject(motion, "velocity_fast", s.fault_velocity_fast);
    }
#if CONFIG_KNOB_CONSOLE_UART
    cJSON_AddStringToObject(root, "transport", "uart0");
#else
    cJSON_AddStringToObject(root, "transport", "usb_serial_jtag");
#endif
    cJSON_AddStringToObject(root, "encoder", encoder_config::name);
    cJSON_AddStringToObject(root, "encoder_interface", encoder_config::transport);
    cJSON_AddNumberToObject(root, "encoder_resolution_bits", encoder_config::resolution_bits);
    cJSON_AddBoolToObject(root, "encoder_dma", encoder_config::uses_dma);
#if CONFIG_KNOB_ALLOW_WEAK_MAGNET
    cJSON_AddBoolToObject(root, "weak_magnet_allowed", encoder_config::kind == EncoderKind::as5600);
#else
    cJSON_AddBoolToObject(root, "weak_magnet_allowed", false);
#endif
    cJSON_AddBoolToObject(root, "ok", !r.error);
    if (r.error) cJSON_AddStringToObject(root, "error", r.error);
    cJSON_AddStringToObject(root, "state", state_name(s.state));
    cJSON_AddStringToObject(root, "fault", s.fault);
#if CONFIG_KNOB_DIAGNOSTICS
    cJSON* diagnostic = cJSON_AddObjectToObject(root, "diagnostic");
    cJSON_AddStringToObject(diagnostic, "axis", s.diagnostic.axis == DiagnosticAxis::d ? "d" : "q");
    cJSON_AddStringToObject(diagnostic, "reference", s.diagnostic.axis == DiagnosticAxis::d ? "fixed_start_angle" : "rotor");
    cJSON_AddBoolToObject(diagnostic, "busy", s.diagnostic.busy);
    cJSON_AddBoolToObject(diagnostic, "cancelled", s.diagnostic.cancelled);
    cJSON_AddNumberToObject(diagnostic, "stage", s.diagnostic.stage);
    cJSON_AddNumberToObject(diagnostic, "count", s.diagnostic.count);
    cJSON_AddNumberToObject(diagnostic, "peak_speed", s.diagnostic.peak_speed);
    cJSON_AddNumberToObject(diagnostic, "peak_current", s.diagnostic.peak_current);
    cJSON_AddNumberToObject(diagnostic, "electrical_offset", s.diagnostic.electrical_offset);
    cJSON_AddNumberToObject(diagnostic, "electrical_reference", s.diagnostic.electrical_reference);
    cJSON_AddNumberToObject(diagnostic, "muted_samples", s.diagnostic.muted_samples);
    cJSON_AddBoolToObject(diagnostic, "recovered", s.diagnostic.recovered);
    if (r.diagnostic_count) {
        cJSON_AddNumberToObject(diagnostic, "offset", r.diagnostic_offset);
        cJSON* samples = cJSON_AddArrayToObject(diagnostic, "samples");
        for (int i = 0; i < r.diagnostic_count; ++i) {
            const auto& v = r.diagnostic_samples[i];
            const double row[] = {double(v.elapsed_us), v.angle, v.velocity, v.target, v.iq, v.id,
                                  v.scale, v.voltage_q, v.phase_peak, double(v.flags), v.velocity_fast, v.voltage_d,
                                  double(v.adc_raw_a), double(v.adc_raw_b),
                                  double(v.sample_start_tick), double(v.sample_end_tick), v.target_d};
            // Keep the legacy q trace layout; d tests append their actual Id target.
            cJSON_AddItemToArray(samples, cJSON_CreateDoubleArray(row,
                s.diagnostic.axis == DiagnosticAxis::d ? 17 : 16));
        }
    }
#endif
    cJSON_AddStringToObject(root, "hardware_detail", s.hardware_detail);
    cJSON_AddNumberToObject(root, "hardware_error", s.hardware_error);
    cJSON_AddNumberToObject(root, "encoder_error", s.encoder_error);
    cJSON_AddNumberToObject(root, "fault_magnet", s.fault_magnet);
    cJSON_AddNumberToObject(root, "encoder_failures", s.encoder_failures);
    cJSON_AddNumberToObject(root, "encoder_retries", s.encoder_retries);
    cJSON_AddNumberToObject(root, "voltage_d", s.voltage_d);
    cJSON_AddNumberToObject(root, "voltage_q", s.voltage_q);
    cJSON_AddNumberToObject(root, "angle", s.angle); cJSON_AddNumberToObject(root, "velocity", s.velocity);
    cJSON_AddNumberToObject(root, "velocity_fast", s.velocity_fast);
    cJSON_AddNumberToObject(root, "position", s.position); cJSON_AddNumberToObject(root, "sub_position", s.sub_position);
    cJSON_AddNumberToObject(root, "iq", s.iq); cJSON_AddNumberToObject(root, "id_current", s.id);
    cJSON_AddNumberToObject(root, "target", s.target); cJSON_AddNumberToObject(root, "magnet", s.magnet);
    cJSON_AddNumberToObject(root, "target_d", s.target_d);
    cJSON_AddBoolToObject(root, "current_valid", s.current_valid);
    cJSON_AddNumberToObject(root, "rejected_samples", s.rejected);
    cJSON_AddNumberToObject(root, "reused_samples", s.reused_samples);
    cJSON_AddNumberToObject(root, "missed_samples", s.missed_samples);
    cJSON_AddNumberToObject(root, "timing_skips", s.timing_skips);
    cJSON_AddNumberToObject(root, "adc_timing_skips", s.adc_timing_skips);
    cJSON_AddNumberToObject(root, "encoder_timing_skips", s.encoder_timing_skips);
    cJSON_AddNumberToObject(root, "sampling_recoveries", s.sampling_recoveries);
    cJSON_AddNumberToObject(root, "pwm_deferred", s.pwm_deferred);
    cJSON_AddNumberToObject(root, "pwm_peak_duty", s.pwm_peak_duty);
    cJSON_AddNumberToObject(root, "pwm_full_duty_cycles", s.pwm_full_duty_cycles);
    cJSON_AddNumberToObject(root, "control_cycles", s.control_cycles);
    cJSON_AddNumberToObject(root, "control_hz", s.control_hz);
    cJSON_AddNumberToObject(root, "control_interval_us", s.control_interval_us);
    cJSON_AddNumberToObject(root, "control_max_us", s.control_max_us);
    cJSON_AddNumberToObject(root, "encoder_age_us", s.encoder_age_us);
    cJSON_AddNumberToObject(root, "encoder_age_max_us", s.encoder_age_max_us);
    const double dma_words[] = {double(s.adc_dma_words[0]), double(s.adc_dma_words[1])};
    cJSON_AddItemToObject(root, "adc_dma_words", cJSON_CreateDoubleArray(dma_words, 2));
    numeric_array(root, "adc_slot_words", s.adc_slot_words);
    numeric_array(root, "adc_trigger_ticks", s.adc_trigger_ticks);
    numeric_array(root, "adc_callback_ticks", s.adc_callback_ticks);
    numeric_array(root, "adc_trigger_sar", s.adc_trigger_sar);
    numeric_array(root, "adc_callback_sar", s.adc_callback_sar);
    numeric_array(root, "adc_callback_elapsed_us", s.adc_callback_elapsed_us);
    cJSON_AddNumberToObject(root, "adc_slot_mask", s.adc_slot_mask);
    cJSON_AddNumberToObject(root, "adc_trigger_down_mask", s.adc_trigger_down_mask);
    cJSON_AddNumberToObject(root, "adc_callback_down_mask", s.adc_callback_down_mask);
    numeric_array(root, "pwm_compare_before", s.pwm_compare_before);
    numeric_array(root, "pwm_compare_after", s.pwm_compare_after);
    cJSON_AddNumberToObject(root, "pwm_latch_sequence", s.pwm_latch_sequence);
    cJSON_AddBoolToObject(root, "pwm_compare_valid", s.pwm_compare_valid);
    cJSON_AddNumberToObject(root, "pwm_apply_delay_us", s.pwm_apply_delay_us);
    cJSON_AddNumberToObject(root, "sample_us", s.sample_us); cJSON_AddNumberToObject(root, "loop_us", s.loop_us);
    cJSON_AddNumberToObject(root, "encoder_us", s.encoder_us);
    cJSON_AddNumberToObject(root, "control_us", s.control_us);
    if (s.fault_sample_interval_us) {
        cJSON* sample = cJSON_AddObjectToObject(root, "fault_sample");
        cJSON_AddNumberToObject(sample, "interval_us", s.fault_sample_interval_us);
        cJSON_AddNumberToObject(sample, "age_us", s.fault_sample_age_us);
        cJSON_AddNumberToObject(sample, "start_tick", s.fault_sample_start_tick);
        cJSON_AddNumberToObject(sample, "end_tick", s.fault_sample_end_tick);
        cJSON_AddNumberToObject(sample, "duration_us", s.fault_sample_us);
        cJSON_AddBoolToObject(sample, "valid", s.fault_sample_valid);
        if (s.fault_adc_captured) {
            const double fault_words[] = {double(s.fault_adc_dma_words[0]), double(s.fault_adc_dma_words[1])};
            cJSON_AddItemToObject(sample, "adc_dma_words", cJSON_CreateDoubleArray(fault_words, 2));
            cJSON_AddItemToObject(sample, "adc_raw", cJSON_CreateIntArray(s.fault_adc_raw.data(), 2));
            cJSON_AddItemToObject(sample, "adc_phase", cJSON_CreateIntArray(s.fault_adc_phase.data(), 2));
        }
    }
    cJSON_AddNumberToObject(root, "sample_start_tick", s.sample_start_tick); cJSON_AddNumberToObject(root, "sample_end_tick", s.sample_end_tick);
    array(root, "currents", s.currents); array(root, "sense_volts", s.volts); array(root, "offsets", s.offsets); array(root, "duty", s.duty);
    cJSON* config = cJSON_AddObjectToObject(root, "config");
    for (auto f : fields) cJSON_AddNumberToObject(config, f.name, s.config.*(f.member));
    cJSON_AddNumberToObject(config, "mode", static_cast<int>(s.config.mode));
    cJSON_AddNumberToObject(config, "min_position", s.config.min_position); cJSON_AddNumberToObject(config, "max_position", s.config.max_position);
    cJSON* cal = cJSON_AddObjectToObject(root, "calibration");
    cJSON_AddBoolToObject(cal, "valid", s.calibration.valid); cJSON_AddNumberToObject(cal, "direction", s.calibration.direction);
    cJSON_AddNumberToObject(cal, "pole_pairs", s.calibration.pole_pairs); cJSON_AddNumberToObject(cal, "electrical_zero", s.calibration.electrical_zero);
    cJSON* quality = cJSON_AddObjectToObject(root, "calibration_quality");
    cJSON_AddStringToObject(quality, "method", s.calibration_quality.method == 1 ? "bidirectional_v1" : "legacy_or_unverified");
    cJSON_AddBoolToObject(quality, "verified", s.calibration_quality.method == 1);
    quality_fields(quality, s.calibration_quality);
    cJSON* report = cJSON_AddObjectToObject(root, "calibration_report");
    cJSON_AddStringToObject(report, "method", "bidirectional_v1");
    cJSON_AddStringToObject(report, "stage", s.calibration_report.stage);
    cJSON_AddStringToObject(report, "error", s.calibration_report.error);
    cJSON_AddBoolToObject(report, "passed", s.calibration_report.passed);
    cJSON_AddBoolToObject(report, "cancelled", s.calibration_report.cancelled);
    cJSON_AddNumberToObject(report, "attempts", s.calibration_report.attempts);
    cJSON_AddNumberToObject(report, "point", s.calibration_report.point);
    cJSON_AddNumberToObject(report, "elapsed_s", s.calibration_report.elapsed_s);
    cJSON_AddNumberToObject(report, "zero_forward", s.calibration_report.zero_forward);
    cJSON_AddNumberToObject(report, "zero_reverse", s.calibration_report.zero_reverse);
    cJSON_AddNumberToObject(report, "observed_d", s.calibration_report.observed_d);
    cJSON_AddNumberToObject(report, "observed_q", s.calibration_report.observed_q);
    cJSON_AddNumberToObject(report, "tracking_error", s.calibration_report.tracking_error);
    cJSON_AddNumberToObject(report, "saturation_fraction", s.calibration_report.saturation_fraction);
    quality_fields(report, s.calibration_report.quality);
    for (unsigned phase = 0; phase < 3; ++phase) {
        const char* key = phase == 0 ? "phase_a_response" : phase == 1 ? "phase_b_response" : "phase_c_response";
        array(report, key, s.calibration_report.phase_response[phase]);
    }
    array(root, "offset_noise_a", s.offset_noise_a);
    array(root, "offset_sem_a", s.offset_sem_a);
    array(root, "offset_drift_a", s.offset_drift_a);
    return root;
}
bool load_settings(Config& c, Calibration& cal, CalibrationQuality& quality) {
    nvs_handle_t handle;
    if (nvs_open("smartknob", NVS_READONLY, &handle) != ESP_OK) return false;
    Stored data{}; size_t length = sizeof(data);
    const auto err = nvs_get_blob(handle, "settings_v1", &data, &length);
    StoredQuality saved_quality{}; size_t quality_length = sizeof(saved_quality);
    const auto quality_error = nvs_get_blob(handle, "cal_quality", &saved_quality, &quality_length);
    nvs_close(handle);
    if (err != ESP_OK || length != sizeof(data) ||
        data.crc != esp_crc32_le(0, reinterpret_cast<const uint8_t*>(&data), offsetof(Stored, crc))) return false;
    if (!restore_encoder_settings(data.version, data.config, data.calibration,
                                  encoder_config::calibration_id, c, cal)) return false;
    quality = {};
    if (quality_error == ESP_OK && quality_length == sizeof(saved_quality) &&
        saved_quality.crc == esp_crc32_le(0, reinterpret_cast<const uint8_t*>(&saved_quality), offsetof(StoredQuality, crc)) &&
        quality_matches(saved_quality.record, cal, encoder_config::calibration_id, c.sense_gain))
        quality = saved_quality.record.quality;
    return true;
}
bool save_settings(const Config& c, const Calibration& cal, const CalibrationQuality& quality) {
    if (validate(c) || (cal.valid && !valid_calibration(cal))) return false;
    StoredQuality saved_quality{};
    saved_quality.record.calibration = cal;
    saved_quality.record.encoder_id = encoder_config::calibration_id;
    saved_quality.record.sense_gain = c.sense_gain;
    saved_quality.record.quality = quality;
    if (quality.method != 0 && !quality_matches(saved_quality.record, cal, encoder_config::calibration_id, c.sense_gain)) return false;
    saved_quality.crc = esp_crc32_le(0, reinterpret_cast<const uint8_t*>(&saved_quality), offsetof(StoredQuality, crc));
    nvs_handle_t handle;
    if (nvs_open("smartknob", NVS_READWRITE, &handle) != ESP_OK) return false;
    Stored data{}; data.version = encoder_settings_version(encoder_config::calibration_id);
    data.config = c; data.calibration = cal;
    data.crc = esp_crc32_le(0, reinterpret_cast<const uint8_t*>(&data), offsetof(Stored, crc));
    // A stale/partially saved sidecar is never trusted: it is bound to the exact
    // encoder, gain and calibration above and independently CRC-protected.
    const bool ok = nvs_set_blob(handle, "cal_quality", &saved_quality, sizeof(saved_quality)) == ESP_OK &&
        nvs_set_blob(handle, "settings_v1", &data, sizeof(data)) == ESP_OK && nvs_commit(handle) == ESP_OK;
    nvs_close(handle); return ok;
}
}
