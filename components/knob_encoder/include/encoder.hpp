#pragma once
#include "encoder_config.hpp"
#include "as5600.hpp"
#include "mt6701.hpp"
#include <type_traits>

namespace knob {
// Both modules compile; only the selected module owns hardware at runtime.
// The business layer consumes EncoderSample and never decodes chip registers.
using Encoder = std::conditional_t<encoder_config::kind == EncoderKind::mt6701, Mt6701, As5600>;
} // namespace knob
