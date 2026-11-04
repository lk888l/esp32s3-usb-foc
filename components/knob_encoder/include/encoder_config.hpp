#pragma once
#include "encoder_types.hpp"

namespace knob::encoder_config {
// Change this one line to select a backend; no build flags or menuconfig choice.
inline constexpr EncoderKind kind = EncoderKind::mt6701;
static_assert(kind == EncoderKind::mt6701 || kind == EncoderKind::as5600);

inline constexpr int ssi_clk_gpio = 12;
inline constexpr int ssi_do_gpio = 13;
inline constexpr int ssi_cs_gpio = 10;
inline constexpr int ssi_clock_hz = 2'000'000;
inline constexpr int as5600_sda_gpio = 8;
inline constexpr int as5600_scl_gpio = 9;
inline constexpr int as5600_clock_hz = 400'000;

inline constexpr const char* name = kind == EncoderKind::mt6701 ? "MT6701" : "AS5600";
inline constexpr const char* transport = kind == EncoderKind::mt6701 ? "ssi_spi_dma" : "i2c";
inline constexpr bool uses_dma = kind == EncoderKind::mt6701;
inline constexpr unsigned resolution_bits = kind == EncoderKind::mt6701 ? 14 : 12;
inline constexpr unsigned counts_per_turn = 1U << resolution_bits;
inline constexpr float sensor_latency_s = kind == EncoderKind::mt6701 ? 0.000005f : 0.000150f;
inline constexpr uint32_t calibration_id = static_cast<uint32_t>(kind);
} // namespace knob::encoder_config
