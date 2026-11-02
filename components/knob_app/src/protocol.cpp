#include "protocol.hpp"
#include "nvs.h"
#include "esp_crc.h"
#include <cmath>
#include <cstring>
#include <cstddef>
namespace knob {
const char* state_name(State s) {
    constexpr const char* names[] = {"idle", "offsets", "align", "sweep", "settle", "active", "fault"};
    return names[static_cast<int>(s)];
}
namespace {
struct Field { const char* name; float Config::*member; };
constexpr Field fields[] = {{"bus_voltage", &Config::bus_voltage}, {"sense_gain", &Config::sense_gain},
    {"current_limit", &Config::current_limit}, {"voltage_limit", &Config::voltage_limit},
    {"current_kp", &Config::current_kp}, {"current_ki", &Config::current_ki}, {"width", &Config::width},
    {"strength", &Config::strength}, {"endstop", &Config::endstop}, {"damping", &Config::damping}, {"snap", &Config::snap}};
void array(cJSON* obj, const char* name, const std::array<float, 3>& values) {
    cJSON_AddItemToObject(obj, name, cJSON_CreateFloatArray(values.data(), values.size()));
}
struct Stored { uint32_t version; Config config; Calibration calibration; uint32_t crc; };
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
        , {"diagnose", Op::diagnose}, {"diagnostic_read", Op::diagnostic_read}
#endif
    };
    bool found = false;
    for (auto n : names) if (!std::strcmp(op->valuestring, n.name)) { r.op = n.op; found = true; }
    if (!found) return fail("unknown_op");
#if CONFIG_KNOB_DIAGNOSTICS
    if (r.op == Op::diagnose) {
        const char* keys[] = {"mode", "velocity", "current", "drive_ms", "observe_ms"};
        double values[5]{};
        for (int i = 0; i < 5; ++i) {
            const auto* item = cJSON_GetObjectItemCaseSensitive(root, keys[i]);
            if (!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble)) return fail("diagnostic_fields_required");
            values[i] = item->valuedouble;
        }
        if (values[0] < 0 || values[0] > 3 || std::floor(values[0]) != values[0] ||
            std::abs(values[1]) < 10 || std::abs(values[1]) > 60 ||
            values[2] < .02 || values[2] > .25 ||
            values[3] < 50 || values[3] > 500 || values[4] < 500 || values[4] > 3000)
            return fail("diagnostic_range");
        r.diagnostic = {static_cast<Mode>(static_cast<int>(values[0])), static_cast<float>(values[1]),
            static_cast<float>(values[2]), static_cast<int>(values[3] * 1000), static_cast<int>(values[4] * 1000)};
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
    cJSON_AddNumberToObject(root, "current_limit_max", max_motor_current);
    cJSON_AddNumberToObject(root, "current_absolute_trip", absolute_current_trip);
    cJSON_AddNumberToObject(root, "current_trip_limit", s.current_trip_limit);
    cJSON_AddNumberToObject(root, "current_target_limit", s.current_target_limit);
    cJSON_AddBoolToObject(root, "current_limited", s.current_limited);
    cJSON_AddNumberToObject(root, "haptic_scale", s.haptic_scale);
    cJSON_AddBoolToObject(root, "speed_limited", s.speed_limited);
    cJSON_AddBoolToObject(root, "haptic_paused", s.haptic_paused);
    cJSON_AddNumberToObject(root, "haptic_fade_start", haptic_fade_start);
    cJSON_AddNumberToObject(root, "haptic_speed_cutoff", haptic_speed_cutoff);
    cJSON_AddNumberToObject(root, "haptic_resume_speed", haptic_resume_speed);
    cJSON_AddNumberToObject(root, "haptic_resume_hold_ms", haptic_resume_hold_s * 1000);
    cJSON_AddNumberToObject(root, "haptic_fade_in_ms", haptic_fade_in_s * 1000);
    cJSON_AddBoolToObject(root, "fault_current_valid", s.fault_current_valid);
    cJSON_AddNumberToObject(root, "fault_current_trip_limit", s.fault_current_trip_limit);
    array(root, "fault_currents", s.fault_currents);
#if CONFIG_KNOB_CONSOLE_UART
    cJSON_AddStringToObject(root, "transport", "uart0");
#else
    cJSON_AddStringToObject(root, "transport", "usb_serial_jtag");
#endif
#if CONFIG_KNOB_ALLOW_WEAK_MAGNET
    cJSON_AddBoolToObject(root, "weak_magnet_allowed", true);
#else
    cJSON_AddBoolToObject(root, "weak_magnet_allowed", false);
#endif
    cJSON_AddBoolToObject(root, "ok", !r.error);
    if (r.error) cJSON_AddStringToObject(root, "error", r.error);
    cJSON_AddStringToObject(root, "state", state_name(s.state));
    cJSON_AddStringToObject(root, "fault", s.fault);
#if CONFIG_KNOB_DIAGNOSTICS
    cJSON* diagnostic = cJSON_AddObjectToObject(root, "diagnostic");
    cJSON_AddBoolToObject(diagnostic, "busy", s.diagnostic.busy);
    cJSON_AddBoolToObject(diagnostic, "cancelled", s.diagnostic.cancelled);
    cJSON_AddNumberToObject(diagnostic, "stage", s.diagnostic.stage);
    cJSON_AddNumberToObject(diagnostic, "count", s.diagnostic.count);
    cJSON_AddNumberToObject(diagnostic, "peak_speed", s.diagnostic.peak_speed);
    cJSON_AddNumberToObject(diagnostic, "peak_current", s.diagnostic.peak_current);
    cJSON_AddNumberToObject(diagnostic, "muted_samples", s.diagnostic.muted_samples);
    cJSON_AddBoolToObject(diagnostic, "recovered", s.diagnostic.recovered);
    if (r.diagnostic_count) {
        cJSON_AddNumberToObject(diagnostic, "offset", r.diagnostic_offset);
        cJSON* samples = cJSON_AddArrayToObject(diagnostic, "samples");
        for (int i = 0; i < r.diagnostic_count; ++i) {
            const auto& v = r.diagnostic_samples[i];
            const double row[] = {double(v.elapsed_us), v.angle, v.velocity, v.target, v.iq, v.id,
                                  v.scale, v.voltage_q, v.phase_peak, double(v.flags), v.velocity_fast};
            cJSON_AddItemToArray(samples, cJSON_CreateDoubleArray(row, 11));
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
    cJSON_AddBoolToObject(root, "current_valid", s.current_valid);
    cJSON_AddNumberToObject(root, "rejected_samples", s.rejected);
    cJSON_AddNumberToObject(root, "reused_samples", s.reused_samples);
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
    return root;
}
bool load_settings(Config& c, Calibration& cal) {
    nvs_handle_t handle;
    if (nvs_open("smartknob", NVS_READONLY, &handle) != ESP_OK) return false;
    Stored data{}; size_t length = sizeof(data);
    const auto err = nvs_get_blob(handle, "settings_v1", &data, &length); nvs_close(handle);
    if (err != ESP_OK || length != sizeof(data) || data.version != 1 ||
        data.crc != esp_crc32_le(0, reinterpret_cast<const uint8_t*>(&data), offsetof(Stored, crc)) ||
        validate(data.config) || (data.calibration.valid && !valid_calibration(data.calibration))) return false;
    c = data.config; cal = data.calibration; return true;
}
bool save_settings(const Config& c, const Calibration& cal) {
    if (validate(c) || (cal.valid && !valid_calibration(cal))) return false;
    nvs_handle_t handle;
    if (nvs_open("smartknob", NVS_READWRITE, &handle) != ESP_OK) return false;
    Stored data{}; data.version = 1; data.config = c; data.calibration = cal;
    data.crc = esp_crc32_le(0, reinterpret_cast<const uint8_t*>(&data), offsetof(Stored, crc));
    const bool ok = nvs_set_blob(handle, "settings_v1", &data, sizeof(data)) == ESP_OK && nvs_commit(handle) == ESP_OK;
    nvs_close(handle); return ok;
}
}
