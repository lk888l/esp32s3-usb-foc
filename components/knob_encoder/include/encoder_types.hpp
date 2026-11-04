#pragma once
#include <cstdint>

namespace knob {
enum class EncoderKind : uint8_t { as5600 = 1, mt6701 = 2 };
enum class EncoderFault : uint8_t { none, transport, crc, missing_magnet, magnet_strength, loss_of_track };
struct EncoderSample {
    float angle = 0;
    uint8_t magnet = 0; // Raw sensor status; interpret using the selected encoder.
    EncoderFault fault = EncoderFault::none;
};
constexpr const char* encoder_fault_name(EncoderFault fault) {
    switch (fault) {
    case EncoderFault::none: return "none";
    case EncoderFault::transport: return "encoder_transport";
    case EncoderFault::crc: return "encoder_crc";
    case EncoderFault::missing_magnet: return "encoder_missing_magnet";
    case EncoderFault::magnet_strength: return "encoder_magnet_strength";
    case EncoderFault::loss_of_track: return "encoder_loss_of_track";
    }
    return "encoder_transport";
}
} // namespace knob
