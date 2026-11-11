#pragma once
#include "reference_modes.hpp"
#include "cJSON.h"
#include <cstdint>

namespace knob {
// A configure patch replaces only explicitly listed mode records. Every listed
// record is complete; omitted modes retain their current device-owned values.
const char* decode_reference_modes(const cJSON* object, ReferenceModeBank& modes, uint16_t& mask);
cJSON* encode_reference_modes(const ReferenceModeBank& modes);
}
