#pragma once
#include "app_module.hpp"
#include <memory>
namespace knob { std::unique_ptr<AppModule> create_module(); }
