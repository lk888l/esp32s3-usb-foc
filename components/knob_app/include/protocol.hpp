#pragma once
#include "control.hpp"
#include "cJSON.h"
#include <array>
#include "sdkconfig.h"
#if CONFIG_KNOB_DIAGNOSTICS
#include "diagnostics.hpp"
#endif
namespace knob {
enum class Op { status, configure, calibrate, enable, disable, clear, zero, pulse, save
#if CONFIG_KNOB_DIAGNOSTICS
    , diagnose, diagnostic_read, test_begin, test_end
#endif
};
enum class State { idle, offsets, align, sweep, settle, active, fault };
const char* state_name(State state);
struct Request {
    int id = 0; Op op = Op::status; Config config{}; bool renew_lease = true;
#if CONFIG_KNOB_DIAGNOSTICS
    DiagnosticPlan diagnostic{};
    int diagnostic_offset = 0;
    bool test_command = false;
#endif
};
struct Snapshot {
    Config config{};
    Calibration calibration{};
    State state = State::idle;
    const char* fault = "none";
    const char* hardware_detail = "none";
    int hardware_error = 0;
    int encoder_error = 0, fault_magnet = 0;
    uint32_t encoder_failures = 0, encoder_retries = 0;
    float angle = 0, velocity = 0, velocity_fast = 0, id = 0, iq = 0, target = 0, sub_position = 0;
    std::array<float, 3> currents{}, volts{}, offsets{}, duty{};
    int position = 0, magnet = 0;
    bool current_valid = false;
    uint32_t rejected = 0, reused_samples = 0;
    float sample_us = 0, loop_us = 0, encoder_us = 0, control_us = 0;
    int fault_sample_interval_us = 0, fault_sample_age_us = 0;
    int fault_sample_start_tick = 0, fault_sample_end_tick = 0;
    float fault_sample_us = 0;
    bool fault_sample_valid = false;
    float voltage_d = 0, voltage_q = 0;
    float current_target_limit = 0;
    float current_trip_limit = overcurrent_threshold(Config{}.current_limit);
    bool current_limited = false;
    float haptic_scale = 0;
    bool speed_limited = false, haptic_paused = false;
    std::array<float, 3> fault_currents{};
    bool fault_current_valid = false;
    float fault_current_trip_limit = 0;
    int sample_start_tick = 0, sample_end_tick = 0;
#if CONFIG_KNOB_DIAGNOSTICS
    DiagnosticSummary diagnostic{};
#endif
};
struct Reply {
    const char* error = nullptr; Snapshot snapshot{}; int request_id = -1;
#if CONFIG_KNOB_DIAGNOSTICS
    std::array<DiagnosticSample, 4> diagnostic_samples{};
    int diagnostic_offset = 0, diagnostic_count = 0;
#endif
};
const char* decode_request(const char* line, Request& request);
cJSON* encode_reply(int id, const Reply& reply);
bool load_settings(Config& config, Calibration& calibration);
bool save_settings(const Config& config, const Calibration& calibration);
}
