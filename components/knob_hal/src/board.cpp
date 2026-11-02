#include "board.hpp"
#include "driver/gpio.h"
#include "esp_adc/adc_cali_scheme.h"
#include "hal/adc_ll.h"
#include "soc/sens_struct.h"
#include "esp_cpu.h"
#include "esp_idf_version.h"
#include "esp_timer.h"
#include "hal/mcpwm_ll.h"
#include "hal/gpio_ll.h"
#include "soc/mcpwm_struct.h"
#include "soc/gpio_struct.h"
#include <algorithm>
#include <cmath>
extern "C" portMUX_TYPE rtc_spinlock;
#if !CONFIG_IDF_TARGET_ESP32S3 || ESP_IDF_VERSION_MAJOR != 5 || ESP_IDF_VERSION_MINOR != 5
#error "Revalidate the exclusive ADC1 fast adapter before changing chip or IDF series"
#endif
namespace knob {
#define TRY(call) do { const esp_err_t e = (call); if (e != ESP_OK) { failed_operation_ = #call; initialization_error_ = e; return e; } } while (false)
int64_t Board::esp_time() { return esp_timer_get_time(); }
esp_err_t Board::initialize() {
    failed_operation_ = "none"; initialization_error_ = 0;
    gpio_config_t gpio{};
    gpio.pin_bit_mask = 1ULL << enable_gpio; gpio.mode = GPIO_MODE_OUTPUT;
    TRY(gpio_config(&gpio)); enable(false);
    i2c_master_bus_config_t bc{};
    bc.i2c_port = I2C_NUM_0; bc.sda_io_num = GPIO_NUM_8; bc.scl_io_num = GPIO_NUM_9;
    bc.clk_source = I2C_CLK_SRC_DEFAULT; bc.glitch_ignore_cnt = 7; bc.flags.enable_internal_pullup = true;
    bc.intr_priority = 2; // Service completion above ordinary peripheral interrupts.
    TRY(i2c_new_master_bus(&bc, &bus_));
    i2c_device_config_t dc{};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7; dc.device_address = 0x36; dc.scl_speed_hz = 300000;
    TRY(i2c_master_bus_add_device(bus_, &dc, &encoder_));
    uint8_t reg = 0x07, conf[2]{};
    // A USB reset can interrupt an I2C transfer while the sensor stays powered.
    // IDF resets its bus FSM on failure. Retry this startup-only read while the
    // bridge is asleep; runtime reads retain their strict freshness budget.
    esp_err_t config_error = ESP_FAIL;
    for (int attempt = 0; attempt < 3; ++attempt) {
        config_error = i2c_master_transmit_receive(encoder_, &reg, 1, conf, 2, 10);
        if (config_error == ESP_OK) break;
        if (attempt < 2) vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (config_error != ESP_OK) {
        failed_operation_ = "AS5600 startup config read"; initialization_error_ = config_error;
        return config_error;
    }
    // Volatile SF=3, FTH=1, PM=0, HYST=0. Never write/burn OTP.
    const uint8_t settings[] = {0x07, static_cast<uint8_t>((conf[0] & 0xe0) | 0x07), static_cast<uint8_t>(conf[1] & 0xf0)};
    TRY(i2c_master_transmit(encoder_, settings, sizeof(settings), 10));
    adc_oneshot_unit_init_cfg_t ac{}; ac.unit_id = ADC_UNIT_1;
    TRY(adc_oneshot_new_unit(&ac, &adc_));
    adc_oneshot_chan_cfg_t channel{};
    channel.atten = ADC_ATTEN_DB_12; channel.bitwidth = ADC_BITWIDTH_12;
    for (int p = 0; p < 3; ++p) {
        TRY(adc_oneshot_config_channel(adc_, static_cast<adc_channel_t>(3 + p), &channel));
        adc_cali_curve_fitting_config_t calibration{};
        calibration.unit_id = ADC_UNIT_1; calibration.chan = static_cast<adc_channel_t>(3 + p);
        calibration.atten = ADC_ATTEN_DB_12; calibration.bitwidth = ADC_BITWIDTH_12;
        TRY(adc_cali_create_scheme_curve_fitting(&calibration, &cali_[p]));
        for (int raw = 0; raw < 4096; ++raw) {
            int mv = 0; TRY(adc_cali_raw_to_voltage(cali_[p], raw, &mv));
            millivolts_[p][raw] = std::clamp(mv, 0, 5000);
        }
    }
    // The driver claims ADC1, powers it, sets per-channel attenuation, and loads
    // eFuse calibration once. Repeating analog REGI2C setup in every sample costs
    // ~82 us per pair on IDF 5.5.4. The exclusive fast path only changes the mux
    // and triggers conversions; no other SAR ADC/ULP user is permitted.
    for (int p = 0; p < 3; ++p) {
        int warmup = 0;
        TRY(adc_oneshot_read(adc_, static_cast<adc_channel_t>(3 + p), &warmup));
    }
    mcpwm_timer_config_t tc{};
    tc.group_id = 0; tc.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT; tc.resolution_hz = resolution;
    // IDF 5.5 defines period_ticks as the full up/down period, not the peak.
    tc.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN; tc.period_ticks = 2 * half_period; tc.intr_priority = 3;
    // Board exclusively owns MCPWM0; its first allocated timer is hardware timer 0.
    TRY(mcpwm_new_timer(&tc, &timer_));
    for (int p = 0; p < 3; ++p) {
        mcpwm_operator_config_t oc{}; oc.group_id = 0;
        TRY(mcpwm_new_operator(&oc, &operators_[p]));
        TRY(mcpwm_operator_connect_timer(operators_[p], timer_));
        mcpwm_comparator_config_t cc{}; cc.flags.update_cmp_on_tez = true;
        TRY(mcpwm_new_comparator(operators_[p], &cc, &comparators_[p]));
        TRY(mcpwm_comparator_set_compare_value(comparators_[p], half_period - 1));
        mcpwm_generator_config_t gc{}; gc.gen_gpio_num = pwm_gpio[p];
        TRY(mcpwm_new_generator(operators_[p], &gc, &generators_[p]));
        TRY(mcpwm_generator_set_action_on_timer_event(generators_[p],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
        TRY(mcpwm_generator_set_action_on_compare_event(generators_[p],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators_[p], MCPWM_GEN_ACTION_HIGH)));
        TRY(mcpwm_generator_set_action_on_compare_event(generators_[p],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, comparators_[p], MCPWM_GEN_ACTION_LOW)));
    }
    mcpwm_comparator_config_t trigger{};
    trigger.intr_priority = 3;
    TRY(mcpwm_new_comparator(operators_[0], &trigger, &sample_comparator_));
    // Trigger 10 us before TEZ. The ISR explicitly verifies that ADC starts
    // at least 6 us after the last falling PWM edge (propagation + CSA settling).
    TRY(mcpwm_comparator_set_compare_value(sample_comparator_, 800));
    return ESP_OK;
}
esp_err_t Board::start_sampling(TaskHandle_t task) {
    task_ = task; mcpwm_timer_event_callbacks_t cb{}; cb.on_empty = on_empty;
    TRY(mcpwm_timer_register_event_callbacks(timer_, &cb, this));
    mcpwm_comparator_event_callbacks_t sample_cb{}; sample_cb.on_reach = on_sample;
    TRY(mcpwm_comparator_register_event_callbacks(sample_comparator_, &sample_cb, this));
    TRY(mcpwm_timer_enable(timer_)); timer_enabled_ = true;
    TRY(mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_NO_STOP)); timer_running_ = true;
    return ESP_OK;
}
bool IRAM_ATTR Board::on_empty(mcpwm_timer_handle_t, const mcpwm_timer_event_data_t*, void* context) {
    auto& b = *static_cast<Board*>(context);
    // Update the shadow registers just after TEZ; all three latch at the next TEZ.
    portENTER_CRITICAL_ISR(&b.mux_);
    if (b.duty_pending_) {
        for (int p = 0; p < 3; ++p) mcpwm_comparator_set_compare_value(b.comparators_[p], b.pending_[p]);
        b.duty_pending_ = false;
    }
    portEXIT_CRITICAL_ISR(&b.mux_);
    return false;
}
bool IRAM_ATTR Board::on_sample(mcpwm_cmpr_handle_t, const mcpwm_compare_event_data_t* event, void* context) {
    if (event->direction != MCPWM_TIMER_DIRECTION_DOWN) return false;
    auto& b = *static_cast<Board*>(context);
    if (++b.divider_ < 10) return false;
    b.divider_ = 0; // 2 kHz control/ADC, synchronized with the 20 kHz carrier.
    RawSample s{}; s.timestamp_us = esp_timer_get_time();
    const int start_count = mcpwm_ll_timer_get_count_value(&MCPWM0, 0);
    const int start = mcpwm_ll_timer_get_count_direction(&MCPWM0, 0) == MCPWM_TIMER_DIRECTION_DOWN ? -start_count : start_count;
    constexpr int low_ticks = half_period - static_cast<int>(half_period * duty_ceiling);
    constexpr int earliest = -low_ticks + 480; // 6 us settling after the last falling edge.
    constexpr int deadline = low_ticks - 80;  // Finish 1 us before the first rising edge.
    s.phase[0] = b.pair_ % 3; s.phase[1] = (b.pair_ + 1) % 3; ++b.pair_;
    bool good = start >= earliest && start < deadline;
    if (good) {
        portENTER_CRITICAL_ISR(&rtc_spinlock);
        for (int k = 0; k < 2; ++k) good &= read_current_fast(3 + s.phase[k], s.raw[k]);
        portEXIT_CRITICAL_ISR(&rtc_spinlock);
    }
    const int end_count = mcpwm_ll_timer_get_count_value(&MCPWM0, 0);
    const int end = mcpwm_ll_timer_get_count_direction(&MCPWM0, 0) == MCPWM_TIMER_DIRECTION_DOWN ? -end_count : end_count;
    // Signed positions around TEZ allow crossing zero, but never the high vector
    // or a complete carrier cycle. No floating-point operations in the ISR.
    const uint32_t elapsed_us = esp_timer_get_time() - s.timestamp_us;
    good &= end >= start && end < deadline && elapsed_us < 28;
    s.duration_ticks = elapsed_us * 80;
    s.start_tick = start; s.end_tick = end;
    s.valid = good; if (!good) ++b.rejected_; s.rejected = b.rejected_;
    if (b.enabled_.load(std::memory_order_relaxed)) {
        int cause = 0;
        portENTER_CRITICAL_ISR(&b.mux_);
        if (good && b.trip_ready_.load(std::memory_order_acquire)) {
            // Integer microvolts, using the same calibrated LUT as the task.
            // Include the reconstructed phase so rotation of the ADC pair
            // cannot conceal a sustained excess in the unmeasured phase.
            const int32_t a = b.millivolts_[s.phase[0]][s.raw[0]] * 1000 - b.offset_uv_[s.phase[0]];
            const int32_t c = b.millivolts_[s.phase[1]][s.raw[1]] * 1000 - b.offset_uv_[s.phase[1]];
            const int32_t peak = std::max({std::abs(a), std::abs(c), std::abs(a + c)});
            if (b.current_trip_filter_.step(peak >= b.trip_uv_, peak >= b.absolute_trip_uv_)) cause = 1;
        } else {
            b.current_trip_filter_.reset();
        }
        portEXIT_CRITICAL_ISR(&b.mux_);
        if (!cause && static_cast<uint32_t>(s.timestamp_us) - b.watchdog_us_.load() > 10000) cause = 2;
        if (cause && !b.trip_.load()) {
            portENTER_CRITICAL_ISR(&b.mux_);
            b.trip_sample_ = s;
            b.trip_.store(cause);
            portEXIT_CRITICAL_ISR(&b.mux_);
        }
        if (b.trip_.load()) gpio_ll_set_level(&GPIO, enable_gpio, 0);
    }
    portENTER_CRITICAL_ISR(&b.mux_);
    b.sample_ = s;
    portEXIT_CRITICAL_ISR(&b.mux_);
    BaseType_t wake = pdFALSE;
    if (b.task_) vTaskNotifyGiveFromISR(b.task_, &wake);
    return wake == pdTRUE;
}
bool IRAM_ATTR Board::read_current_fast(unsigned channel, int& raw) {
    const uint32_t started = esp_cpu_get_cycle_count();
    constexpr uint32_t timeout_cycles = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 20;
    while (HAL_FORCE_READ_U32_REG_FIELD(SENS.sar_slave_addr1, meas_status) != 0) {
        if (esp_cpu_get_cycle_count() - started > timeout_cycles) return false;
    }
    adc_oneshot_ll_set_channel(ADC_UNIT_1, static_cast<adc_channel_t>(channel));
    adc_oneshot_ll_start(ADC_UNIT_1);
    while (!adc_oneshot_ll_get_event(ADC_LL_EVENT_ADC1_ONESHOT_DONE)) {
        if (esp_cpu_get_cycle_count() - started > timeout_cycles) return false;
    }
    raw = adc_oneshot_ll_get_raw_result(ADC_UNIT_1);
    return true;
}
RawSample Board::sample() { portENTER_CRITICAL(&mux_); const auto s = sample_; portEXIT_CRITICAL(&mux_); return s; }
RawSample Board::trip_sample() { portENTER_CRITICAL(&mux_); const auto s = trip_sample_; portEXIT_CRITICAL(&mux_); return s; }
float Board::volts(unsigned p, int raw) const { return millivolts_[p][std::clamp(raw, 0, 4095)] * 0.001f; }
void Board::set_trip(const std::array<float, 3>& zero, float gain, float amperes) {
    std::array<int32_t, 3> offsets{};
    for (int p = 0; p < 3; ++p) {
        offsets[p] = std::lround(zero[p] * 1000000);
    }
    const int32_t threshold = std::ceil(gain * amperes * 1000000);
    const int32_t absolute = std::ceil(gain * absolute_current_trip * 1000000);
    // Publish all three phases atomically relative to the sampling ISR.
    portENTER_CRITICAL(&mux_);
    if (offset_uv_ != offsets || trip_uv_ != threshold || absolute_trip_uv_ != absolute) current_trip_filter_.reset();
    offset_uv_ = offsets; trip_uv_ = threshold; absolute_trip_uv_ = absolute;
    trip_ready_.store(true, std::memory_order_release);
    portEXIT_CRITICAL(&mux_);
}
void Board::reset_trip() {
    portENTER_CRITICAL(&mux_);
    trip_.store(0); current_trip_filter_.reset();
    portEXIT_CRITICAL(&mux_);
}
esp_err_t Board::read_angle(float& angle, uint8_t& status) {
    const uint8_t reg = 0x0b; uint8_t data[3]{};
    const int64_t started = esp_timer_get_time();
    esp_err_t error = i2c_master_transmit_receive(encoder_, &reg, 1, data, sizeof(data), 2);
    // One short NACK/transaction failure can be retried within the freshness
    // budget. Never retry a slow timeout or resume from a latched motor fault.
    if (error != ESP_OK && esp_timer_get_time() - started < 500) {
        ++encoder_retries_;
        error = i2c_master_transmit_receive(encoder_, &reg, 1, data, sizeof(data), 2);
    }
    if (error == ESP_OK && esp_timer_get_time() - started > 1000) error = ESP_ERR_TIMEOUT;
    encoder_transport_error_ = error;
    if (error != ESP_OK) return error;
    status = data[0]; angle = ((data[1] & 0x0f) * 256 + data[2]) * tau / 4096;
#if CONFIG_KNOB_ALLOW_WEAK_MAGNET
    constexpr uint8_t rejected_flags = 0x08;
#else
    constexpr uint8_t rejected_flags = 0x18;
#endif
    return (status & 0x20) && !(status & rejected_flags) ? ESP_OK : ESP_ERR_INVALID_STATE;
}
esp_err_t Board::write_duty(const std::array<float, 3>& duty) {
    std::array<uint32_t, 3> next{};
    for (int p = 0; p < 3; ++p) {
        if (!std::isfinite(duty[p]) || duty[p] < 0 || duty[p] > duty_ceiling) { enable(false); return ESP_ERR_INVALID_ARG; }
        next[p] = std::clamp<uint32_t>(static_cast<uint32_t>(half_period * (1 - duty[p])), 1, half_period - 1);
    }
    portENTER_CRITICAL(&mux_); pending_ = next; duty_pending_ = true; portEXIT_CRITICAL(&mux_);
    return ESP_OK;
}
void Board::enable(bool enabled) {
    if (!enabled) { gpio_set_level(static_cast<gpio_num_t>(enable_gpio), 0); trip_ready_.store(false); }
    enabled_.store(enabled);
    if (enabled) { kick(); gpio_set_level(static_cast<gpio_num_t>(enable_gpio), 1); }
}
void Board::shutdown() {
    enable(false);
    if (timer_running_) { mcpwm_timer_start_stop(timer_, MCPWM_TIMER_STOP_EMPTY); vTaskDelay(pdMS_TO_TICKS(2)); timer_running_ = false; }
    if (timer_enabled_) { mcpwm_timer_disable(timer_); timer_enabled_ = false; }
    if (sample_comparator_) { mcpwm_del_comparator(sample_comparator_); sample_comparator_ = nullptr; }
    for (int p = 2; p >= 0; --p) {
        if (generators_[p]) { mcpwm_del_generator(generators_[p]); generators_[p] = nullptr; }
        if (comparators_[p]) { mcpwm_del_comparator(comparators_[p]); comparators_[p] = nullptr; }
        if (operators_[p]) { mcpwm_del_operator(operators_[p]); operators_[p] = nullptr; }
        if (cali_[p]) { adc_cali_delete_scheme_curve_fitting(cali_[p]); cali_[p] = nullptr; }
    }
    if (timer_) { mcpwm_del_timer(timer_); timer_ = nullptr; }
    if (adc_) { adc_oneshot_del_unit(adc_); adc_ = nullptr; }
    if (encoder_) { i2c_master_bus_rm_device(encoder_); encoder_ = nullptr; }
    if (bus_) { i2c_del_master_bus(bus_); bus_ = nullptr; }
}
}
