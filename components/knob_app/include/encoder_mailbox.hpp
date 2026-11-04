#pragma once
#include <cstdint>
#include "encoder_types.hpp"
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#endif

namespace knob {
struct EncoderFrame : EncoderSample {
    int error = 0, transport_error = 0;
    uint32_t duration_us = 0, retries = 0, failures = 0;
    int64_t timestamp_us = 0;
};

// Caller serializes every operation with the encoder's existing spinlock.
// Healthy frames may replace one another; the first unread failure survives.
class EncoderMailbox {
public:
    void reset() { latest_ = {}; pending_error_ = {}; error_pending_ = false; }
    void publish(const EncoderFrame& frame) {
        latest_ = frame;
        if (frame.error && !error_pending_) {
            pending_error_ = frame;
            error_pending_ = true;
        }
    }
#ifdef ESP_PLATFORM
    IRAM_ATTR
#endif
    EncoderFrame consume() {
        auto result = error_pending_ ? pending_error_ : latest_;
        error_pending_ = false;
        // Preserve the first failure's cause/time, but report current lifetime
        // counters even when more failures/retries occurred before consumption.
        result.failures = latest_.failures;
        result.retries = latest_.retries;
        return result;
    }
    // Clear acknowledges events already published. A still-failed latest frame
    // remains failed, and a failure published after this call latches anew.
    void acknowledge_errors() { error_pending_ = false; }
private:
    EncoderFrame latest_{}, pending_error_{};
    bool error_pending_ = false;
};
} // namespace knob
