#include "mt6701.hpp"
#include "mt6701_frame.hpp"
#include "encoder_config.hpp"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include <algorithm>

namespace knob {
esp_err_t Mt6701::initialize() {
    if (bus_initialized_ || device_ || rx_) return ESP_ERR_INVALID_STATE;
    failed_operation_ = "none"; transport_error_ = ESP_OK;
    const auto check = [this](esp_err_t error, const char* operation) {
        if (error != ESP_OK) { failed_operation_ = operation; shutdown(); }
        return error;
    };
    // Allocate once: the 24-bit transfer occupies a full 32-bit DMA write.
    rx_ = static_cast<uint8_t*>(heap_caps_aligned_alloc(4, 4, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (!rx_) return check(ESP_ERR_NO_MEM, "MT6701 DMA buffer");
    spi_bus_config_t bc{};
    bc.mosi_io_num = -1; bc.miso_io_num = encoder_config::ssi_do_gpio;
    bc.sclk_io_num = encoder_config::ssi_clk_gpio;
    bc.quadwp_io_num = -1; bc.quadhd_io_num = -1;
    bc.data4_io_num = -1; bc.data5_io_num = -1; bc.data6_io_num = -1; bc.data7_io_num = -1;
    bc.max_transfer_sz = 4;
    bc.intr_flags = ESP_INTR_FLAG_LEVEL2;
    esp_err_t error = spi_bus_initialize(host, &bc, SPI_DMA_CH_AUTO);
    if (check(error, "MT6701 SPI2 DMA bus") != ESP_OK) return error;
    bus_initialized_ = true;
    // Disconnected DO should read high and fail CRC, not float to a valid zero.
    error = check(gpio_set_pull_mode(static_cast<gpio_num_t>(encoder_config::ssi_do_gpio), GPIO_PULLUP_ONLY), "MT6701 DO pull-up");
    if (error != ESP_OK) return error;
    spi_device_interface_config_t dc{};
    // Rev.1.8 Fig.24/25: idle LOW, first rising edge presents D13, falling
    // captures it (mode 1). The English idle-high sentence contradicts figures.
    dc.mode = 1;
    dc.clock_speed_hz = encoder_config::ssi_clock_hz;
    dc.spics_io_num = encoder_config::ssi_cs_gpio;
    dc.cs_ena_pretrans = 1; dc.cs_ena_posttrans = 1;
    dc.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY;
    dc.queue_size = 1;
    error = check(spi_bus_add_device(host, &dc, &device_), "MT6701 SSI device");
    if (error != ESP_OK) return error;
    transaction_ = {};
    transaction_.flags = SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL;
    transaction_.rxlength = 24; transaction_.rx_buffer = rx_;
    // No TX/command/address/dummy phases: exactly 24 SSI clocks, not 32.
    // Round up the sensor's 32ms power-up maximum, including a partial tick.
    vTaskDelay(pdMS_TO_TICKS(32) + 1);
    return ESP_OK;
}
esp_err_t Mt6701::finish(TickType_t wait) {
    spi_transaction_t* completed = nullptr;
    const esp_err_t error = spi_device_get_trans_result(device_, &completed, wait);
    // A DMA error can consume the result and return an error simultaneously.
    if (completed == &transaction_) pending_ = false;
    return error;
}
esp_err_t Mt6701::read(EncoderSample& sample) {
    sample = {};
    const auto failed = [this, &sample](esp_err_t error) {
        sample.fault = EncoderFault::transport;
        return transport_error_ = error;
    };
    if (!device_) return failed(ESP_ERR_INVALID_STATE);
    if (pending_) {
        // A timed-out transaction still owns the descriptor and DMA buffer.
        // Drain and discard it; an old angle must not get a fresh timestamp.
        const esp_err_t error = finish(0);
        return failed(error == ESP_OK ? ESP_ERR_TIMEOUT : error);
    }
    const int64_t started = esp_timer_get_time();
    transaction_.flags = SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL;
    esp_err_t error = spi_device_queue_trans(device_, &transaction_, 0);
    if (error != ESP_OK) return failed(error);
    pending_ = true;
    error = finish(std::max<TickType_t>(1, pdMS_TO_TICKS(1)));
    if (error != ESP_OK) return failed(error);
    if (esp_timer_get_time() - started > 1000) return failed(ESP_ERR_TIMEOUT);
    transport_error_ = ESP_OK;
    sample = mt6701::decode(rx_);
    if (sample.fault == EncoderFault::crc) return ESP_ERR_INVALID_CRC;
    return sample.fault == EncoderFault::none ? ESP_OK : ESP_ERR_INVALID_STATE;
}
void Mt6701::shutdown() {
    // Worker is stopped before teardown. Complete ownership handoff even if
    // its last bounded read timed out; DMA must never write into freed memory.
    if (pending_ && device_) finish(portMAX_DELAY);
    if (device_) { spi_bus_remove_device(device_); device_ = nullptr; }
    if (bus_initialized_) { spi_bus_free(host); bus_initialized_ = false; }
    if (rx_) { heap_caps_free(rx_); rx_ = nullptr; }
    transaction_ = {}; pending_ = false;
}
} // namespace knob
