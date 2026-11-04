#include "mt6701_frame.hpp"
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>

using namespace knob;

// Independent oracle: divide the complete polynomial (payload * x^6) by
// x^6+x+1. This does not use the production shift-register implementation.
static uint8_t reference_crc(uint32_t payload) {
    uint32_t remainder = payload << 6;
    for (int degree = 23; degree >= 6; --degree)
        if (remainder & (1U << degree)) remainder ^= 0x43U << (degree - 6);
    return static_cast<uint8_t>(remainder);
}

static std::array<uint8_t, 3> bytes(uint32_t frame) {
    return {static_cast<uint8_t>(frame >> 16), static_cast<uint8_t>(frame >> 8), static_cast<uint8_t>(frame)};
}

int main() {
    struct Vector { uint32_t payload, frame; };
    // Fixed polynomial-division vectors, including zero and both range ends.
    constexpr Vector vectors[] = {
        {0x00000, 0x000000}, {0x00001, 0x000043}, {0x00010, 0x000430},
        {0x10000, 0x400035}, {0x20000, 0x800029}, {0x12340, 0x48d012},
        {0x3fff0, 0xfffc1f}, {0x3ffff, 0xffffce},
    };
    for (const auto& v : vectors) {
        assert(reference_crc(v.payload) == (v.frame & 0x3f));
        assert(mt6701::crc6(v.payload) == (v.frame & 0x3f));
        const auto good = bytes(v.frame);
        assert(mt6701::decode(good.data()).fault != EncoderFault::crc);
        for (int bit = 0; bit < 24; ++bit) {
            const auto corrupt = bytes(v.frame ^ (1U << bit));
            const auto result = mt6701::decode(corrupt.data());
            assert(result.fault == EncoderFault::crc);
            assert(result.angle == 0 && result.magnet == 0); // Untrusted status is not published.
        }
    }

    // Exercise every angle, including the quadrant and wrap boundaries, and
    // every status combination. In particular Mg2 alone is a healthy push.
    constexpr float tau = 6.28318530717958647692f;
    for (uint32_t raw = 0; raw < 16384; ++raw) {
        for (uint32_t status = 0; status < 16; ++status) {
            const uint32_t payload = (raw << 4) | status;
            const auto data = bytes((payload << 6) | reference_crc(payload));
            const auto result = mt6701::decode(data.data());
            assert(result.magnet == status);
            assert(std::abs(result.angle - static_cast<float>(raw) / 16384 * tau) < 0.000001f);
            assert(result.angle >= 0 && result.angle < tau);
            const auto expected = (status & 8) ? EncoderFault::loss_of_track :
                                  (status & 3) ? EncoderFault::magnet_strength : EncoderFault::none;
            assert(result.fault == expected);
        }
    }
    const auto disconnected = bytes(0xffffff);
    assert(mt6701::decode(disconnected.data()).fault == EncoderFault::crc);
    // Zero angle/status/CRC is valid and must not be confused with unplugged DO.
    const auto zero = bytes(0);
    assert(mt6701::decode(zero.data()).fault == EncoderFault::none);
    std::cout << "PASS: MT6701 CRC vectors, 262144 angle/status combinations, all single-bit errors, disconnect.\n";
}
