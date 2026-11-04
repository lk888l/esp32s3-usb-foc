#pragma once
#include "spi_master.h"
using gpio_num_t = int;
inline constexpr int GPIO_PULLUP_ONLY = 0;
esp_err_t gpio_set_pull_mode(gpio_num_t, int);
