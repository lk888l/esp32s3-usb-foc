#pragma once
#include "encoder_types.hpp"

namespace knob::mt6701 {
// MT6701 Rev.1.8 section 7.8.2: D13..D0, Mg3..Mg0, CRC5..CRC0.
// CRC-6 x^6+x+1, initial value 0, MSB first, no reflection/final XOR.
constexpr uint8_t crc6(uint32_t payload) {
    uint8_t crc = 0;
    for (int bit = 17; bit >= 0; --bit) {
        const bool feedback = ((crc >> 5) ^ (payload >> bit)) & 1U;
        crc = (crc << 1) & 0x3f;
        if (feedback) crc ^= 0x03;
    }
    return crc;
}
constexpr EncoderSample decode(const uint8_t* bytes) {
    const uint32_t frame = (uint32_t{bytes[0]} << 16) | (uint32_t{bytes[1]} << 8) | bytes[2];
    EncoderSample result{};
    if (crc6(frame >> 6) != (frame & 0x3f)) {
        result.fault = EncoderFault::crc;
        return result; // Corrupt angle AND status are untrusted.
    }
    result.magnet = (frame >> 6) & 0x0f;
    result.angle = (frame >> 10) * (6.28318530717958647692f / 16384.0f);
    if (result.magnet & 0x08) result.fault = EncoderFault::loss_of_track;
    else if (result.magnet & 0x03) result.fault = EncoderFault::magnet_strength;
    // Mg2 is the push indication, not a magnetic/angle error. Reserved Mg1:0=3
    // is rejected along with weak/strong fields. AS5600's ML exception is local
    // to its backend and does not relax MT6701 validation.
    return result;
}
} // namespace knob::mt6701
