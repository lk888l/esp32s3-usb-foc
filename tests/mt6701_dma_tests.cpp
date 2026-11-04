#include "mt6701.hpp"
#include "encoder_config.hpp"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>

using namespace knob;
namespace {
enum class FailAt { none, allocation, bus, pullup, device };
struct Completion {
    esp_err_t error = ESP_OK;
    bool consumed = true;
    uint32_t frame = 0x48d012;
    int64_t elapsed_us = 20;
};
struct FakeSpi {
    FailAt fail_at = FailAt::none;
    bool bus = false, device = false;
    void* allocation = nullptr;
    spi_transaction_t* active = nullptr;
    std::deque<Completion> completions;
    esp_err_t queue_error = ESP_OK;
    unsigned submitted = 0;
    TickType_t last_wait = 0;
    int64_t now = 0;
} fake;
void reset() {
    assert(!fake.bus && !fake.device && !fake.allocation && !fake.active);
    fake = {};
}
void expect_transport(Mt6701& encoder, EncoderSample& sample, esp_err_t expected) {
    sample.angle = 5; sample.magnet = 0xf; sample.fault = EncoderFault::none;
    assert(encoder.read(sample) == expected);
    assert(sample.fault == EncoderFault::transport);
    assert(sample.angle == 0 && sample.magnet == 0);
    assert(encoder.transport_error() == expected);
}
} // namespace

void* heap_caps_aligned_alloc(size_t alignment, size_t size, uint32_t caps) {
    assert(!fake.allocation && alignment == 4 && size >= 4);
    assert((caps & (MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL)) == (MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (fake.fail_at == FailAt::allocation) return nullptr;
    fake.allocation = std::malloc(size);
    assert(fake.allocation && reinterpret_cast<uintptr_t>(fake.allocation) % alignment == 0);
    return fake.allocation;
}
void heap_caps_free(void* memory) {
    assert(memory == fake.allocation && !fake.active && !fake.device && !fake.bus);
    std::free(memory); fake.allocation = nullptr;
}
esp_err_t spi_bus_initialize(spi_host_device_t host, const spi_bus_config_t* config, int dma) {
    assert(host == SPI2_HOST && dma == SPI_DMA_CH_AUTO && !fake.bus);
    assert(config->mosi_io_num == -1 && config->miso_io_num == encoder_config::ssi_do_gpio);
    assert(config->sclk_io_num == encoder_config::ssi_clk_gpio && config->max_transfer_sz >= 4);
    assert(config->quadwp_io_num == -1 && config->quadhd_io_num == -1);
    assert(config->data4_io_num == -1 && config->data5_io_num == -1);
    assert(config->data6_io_num == -1 && config->data7_io_num == -1);
    if (fake.fail_at == FailAt::bus) return ESP_ERR_INVALID_STATE;
    fake.bus = true;
    return ESP_OK;
}
esp_err_t gpio_set_pull_mode(gpio_num_t pin, int mode) {
    assert(fake.bus && pin == encoder_config::ssi_do_gpio && mode == GPIO_PULLUP_ONLY);
    return fake.fail_at == FailAt::pullup ? ESP_ERR_INVALID_STATE : ESP_OK;
}
esp_err_t spi_bus_add_device(spi_host_device_t host, const spi_device_interface_config_t* config, spi_device_handle_t* output) {
    assert(host == SPI2_HOST && fake.bus && !fake.device);
    assert(config->mode == 1 && config->clock_speed_hz == encoder_config::ssi_clock_hz);
    assert(config->spics_io_num == encoder_config::ssi_cs_gpio);
    assert(config->cs_ena_pretrans >= 1 && config->cs_ena_posttrans >= 1);
    assert(config->flags == (SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY));
    assert(config->queue_size == 1);
    if (fake.fail_at == FailAt::device) return ESP_ERR_INVALID_STATE;
    fake.device = true; *output = &fake;
    return ESP_OK;
}
esp_err_t spi_device_queue_trans(spi_device_handle_t device, spi_transaction_t* transaction, TickType_t wait) {
    assert(device == &fake && fake.device && !fake.active && wait == 0);
    assert(transaction->flags == SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL);
    assert(transaction->length == 0 && transaction->rxlength == 24);
    assert(transaction->rx_buffer == fake.allocation);
    if (fake.queue_error != ESP_OK) return fake.queue_error;
    fake.active = transaction; ++fake.submitted;
    return ESP_OK;
}
esp_err_t spi_device_get_trans_result(spi_device_handle_t device, spi_transaction_t** transaction, TickType_t wait) {
    assert(device == &fake && fake.device && fake.active);
    fake.last_wait = wait;
    Completion completion;
    if (!fake.completions.empty()) {
        completion = fake.completions.front(); fake.completions.pop_front();
    }
    fake.now += completion.elapsed_us;
    if (!completion.consumed) {
        assert(completion.error == ESP_ERR_TIMEOUT);
        *transaction = nullptr;
        return completion.error;
    }
    *transaction = fake.active;
    auto* bytes = static_cast<uint8_t*>(fake.active->rx_buffer);
    bytes[0] = completion.frame >> 16; bytes[1] = completion.frame >> 8; bytes[2] = completion.frame;
    bytes[3] = 0xa5; // Hardware may write the padded fourth byte.
    fake.active = nullptr;
    return completion.error;
}
esp_err_t spi_bus_remove_device(spi_device_handle_t device) {
    assert(device == &fake && fake.device && !fake.active);
    fake.device = false;
    return ESP_OK;
}
esp_err_t spi_bus_free(spi_host_device_t host) {
    assert(host == SPI2_HOST && fake.bus && !fake.device && !fake.active);
    fake.bus = false;
    return ESP_OK;
}
int64_t esp_timer_get_time() { return fake.now; }
void vTaskDelay(TickType_t ticks) { fake.now += ticks * 1000; }

int main() {
    EncoderSample sample;
    {
        Mt6701 encoder;
        expect_transport(encoder, sample, ESP_ERR_INVALID_STATE);
        encoder.shutdown();
    }
    for (const auto stage : {FailAt::allocation, FailAt::bus, FailAt::pullup, FailAt::device}) {
        reset(); fake.fail_at = stage;
        Mt6701 encoder;
        assert(encoder.initialize() != ESP_OK);
        assert(std::strcmp(encoder.failed_operation(), "none") != 0);
        assert(!fake.bus && !fake.device && !fake.allocation);
        fake.fail_at = FailAt::none;
        assert(encoder.initialize() == ESP_OK); // Partial initialization may be retried.
        encoder.shutdown(); encoder.shutdown();
    }
    reset();
    Mt6701 encoder;
    assert(encoder.initialize() == ESP_OK && fake.now >= 32000);
    assert(encoder.initialize() == ESP_ERR_INVALID_STATE);
    assert(encoder.read(sample) == ESP_OK && sample.angle > 0 && sample.magnet == 0);
    assert(encoder.transport_error() == ESP_OK);

    fake.queue_error = ESP_ERR_TIMEOUT;
    expect_transport(encoder, sample, ESP_ERR_TIMEOUT);
    assert(!fake.active);
    fake.queue_error = ESP_OK;
    assert(encoder.read(sample) == ESP_OK);

    // Timed-out DMA keeps ownership; repeated reads may not overwrite its
    // descriptor or accept the late angle under a new sample timestamp.
    fake.completions.push_back({ESP_ERR_TIMEOUT, false});
    expect_transport(encoder, sample, ESP_ERR_TIMEOUT);
    auto* const pending = fake.active;
    const auto submitted = fake.submitted;
    fake.completions.push_back({ESP_ERR_TIMEOUT, false});
    expect_transport(encoder, sample, ESP_ERR_TIMEOUT);
    assert(fake.active == pending && fake.submitted == submitted && fake.last_wait == 0);
    fake.completions.push_back({ESP_OK, true, 0x800029});
    expect_transport(encoder, sample, ESP_ERR_TIMEOUT); // Completed late frame is discarded.
    assert(!fake.active && fake.submitted == submitted);
    assert(encoder.read(sample) == ESP_OK && fake.submitted == submitted + 1);

    // IDF consumes a completed DMA-error descriptor even though its return
    // code is ESP_ERR_INVALID_STATE. The next call must submit fresh work.
    fake.completions.push_back({ESP_ERR_INVALID_STATE, true});
    expect_transport(encoder, sample, ESP_ERR_INVALID_STATE);
    assert(!fake.active);
    assert(encoder.read(sample) == ESP_OK);
    fake.completions.push_back({ESP_ERR_TIMEOUT, false});
    expect_transport(encoder, sample, ESP_ERR_TIMEOUT);
    fake.completions.push_back({ESP_ERR_INVALID_STATE, true});
    expect_transport(encoder, sample, ESP_ERR_INVALID_STATE);
    assert(encoder.read(sample) == ESP_OK);

    // Even a successful completion is stale beyond the configured 1ms budget.
    fake.completions.push_back({ESP_OK, true, 0x800029, 1001});
    expect_transport(encoder, sample, ESP_ERR_TIMEOUT);
    assert(encoder.read(sample) == ESP_OK);

    fake.completions.push_back({ESP_OK, true, 0xffffff});
    assert(encoder.read(sample) == ESP_ERR_INVALID_CRC && sample.fault == EncoderFault::crc);
    assert(encoder.transport_error() == ESP_OK && sample.angle == 0 && sample.magnet == 0);
    fake.completions.push_back({ESP_OK, true, 0x000043}); // Strong field with valid CRC.
    assert(encoder.read(sample) == ESP_ERR_INVALID_STATE && sample.fault == EncoderFault::magnet_strength);
    assert(sample.magnet == 1 && encoder.transport_error() == ESP_OK);

    fake.completions.push_back({ESP_ERR_TIMEOUT, false});
    expect_transport(encoder, sample, ESP_ERR_TIMEOUT);
    encoder.shutdown(); // Fake asserts draining occurs before any free/remove.
    assert(fake.last_wait == portMAX_DELAY && !fake.active);
    assert(!fake.device && !fake.bus && !fake.allocation);
    expect_transport(encoder, sample, ESP_ERR_INVALID_STATE);
    assert(encoder.initialize() == ESP_OK && encoder.read(sample) == ESP_OK);
    encoder.shutdown();
    std::cout << "PASS: MT6701 DMA ownership, stale results, DMA errors, initialization unwind, restart.\n";
}
