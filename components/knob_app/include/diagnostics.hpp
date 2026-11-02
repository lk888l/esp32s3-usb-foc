#pragma once
#include "control.hpp"
#include <cstdint>
namespace knob {
constexpr int diagnostic_capacity = 2048;
struct DiagnosticPlan {
    Mode mode = Mode::detents;
    float velocity = 50, current = .15f;
    int drive_us = 300000, observe_us = 2000000;
};
struct DiagnosticSample {
    uint32_t elapsed_us = 0;
    float angle = 0, velocity = 0, target = 0, iq = 0, id = 0;
    float scale = 0, voltage_q = 0, phase_peak = 0;
    uint32_t flags = 0; // low byte: stage, bit 8: paused, bit 9: limited
    float velocity_fast = 0;
};
struct DiagnosticSummary {
    bool busy = false, cancelled = false, recovered = false;
    int stage = 0; // 0 idle, 1 preparing, 2 drive, 3 release, 4 complete, 5 aborted
    int count = 0;
    uint32_t muted_samples = 0;
    float peak_speed = 0, peak_current = 0;
};
}
