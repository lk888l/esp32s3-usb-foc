#pragma once
#include "esp_err.h"
using adc_oneshot_unit_handle_t = void*;
enum adc_unit_t { ADC_UNIT_1, ADC_UNIT_2 };
enum adc_channel_t { ADC_CHANNEL_0 };
enum adc_atten_t { ADC_ATTEN_DB_12 };
enum adc_bitwidth_t { ADC_BITWIDTH_12 };
enum adc_ulp_mode_t { ADC_ULP_MODE_DISABLE };
struct adc_oneshot_unit_init_cfg_t {
    adc_unit_t unit_id;
    int clk_src;
    adc_ulp_mode_t ulp_mode;
};
struct adc_oneshot_chan_cfg_t { adc_atten_t atten; adc_bitwidth_t bitwidth; };
esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t*, adc_oneshot_unit_handle_t*);
esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t, adc_channel_t, const adc_oneshot_chan_cfg_t*);
esp_err_t adc_oneshot_del_unit(adc_oneshot_unit_handle_t);
esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t, adc_channel_t, int*);
