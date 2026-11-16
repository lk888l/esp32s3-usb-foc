#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace knob::usb {
// Only an atomic flag is touched by the real-time controller. USB work stays
// on CPU0, below the encoder worker; it never waits on or calls into FOC.
inline std::atomic<bool> input_allowed{false};
struct WheelStatus {
    bool enabled = false, mounted = false, suspended = false, ready = false;
    uint32_t reports = 0, steps = 0;
};
bool initialize();
bool shutdown();
int read(uint8_t* bytes, size_t capacity);
bool write(const char* bytes, size_t size);
void publish_encoder(float angle, int64_t timestamp_us, bool valid);
void set_wheel_enabled(bool enabled);
WheelStatus wheel_status();
bool bootloader_requested();
[[noreturn]] void enter_bootloader(); // Caller must stop the motor first.
}
