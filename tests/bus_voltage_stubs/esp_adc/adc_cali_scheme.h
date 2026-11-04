#pragma once
#include "adc_cali.h"
#include "adc_oneshot.h"
struct adc_cali_curve_fitting_config_t {
    adc_unit_t unit_id;
    adc_channel_t chan;
    adc_atten_t atten;
    adc_bitwidth_t bitwidth;
};
esp_err_t adc_cali_create_scheme_curve_fitting(const adc_cali_curve_fitting_config_t*, adc_cali_handle_t*);
esp_err_t adc_cali_delete_scheme_curve_fitting(adc_cali_handle_t);
