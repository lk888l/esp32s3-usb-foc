#pragma once
#include <cstdint>
using TickType_t = uint32_t;
inline constexpr TickType_t portMAX_DELAY = UINT32_MAX;
inline constexpr TickType_t pdMS_TO_TICKS(uint32_t ms) { return ms; }
