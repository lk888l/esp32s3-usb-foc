#pragma once
#include "freertos/FreeRTOS.h"
#include <cstddef>
#include <cstdint>

using esp_err_t = int;
inline constexpr esp_err_t ESP_OK = 0, ESP_ERR_NO_MEM = 0x101,
    ESP_ERR_INVALID_STATE = 0x103, ESP_ERR_TIMEOUT = 0x107, ESP_ERR_INVALID_CRC = 0x109;
using spi_host_device_t = int;
using spi_device_handle_t = void*;
inline constexpr int SPI2_HOST = 1, SPI_DMA_CH_AUTO = 3;
inline constexpr uint32_t SPI_DEVICE_HALFDUPLEX = 1U << 4, SPI_DEVICE_NO_DUMMY = 1U << 6;
inline constexpr uint32_t SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL = 1U << 11;
struct spi_bus_config_t {
    int mosi_io_num{}, miso_io_num{}, sclk_io_num{}, quadwp_io_num{}, quadhd_io_num{};
    int data4_io_num{}, data5_io_num{}, data6_io_num{}, data7_io_num{};
    int max_transfer_sz{}, intr_flags{};
};
struct spi_device_interface_config_t {
    uint8_t mode{};
    int clock_speed_hz{}, spics_io_num{};
    uint16_t cs_ena_pretrans{};
    uint8_t cs_ena_posttrans{};
    uint32_t flags{};
    int queue_size{};
};
struct spi_transaction_t {
    uint32_t flags{};
    size_t length{}, rxlength{};
    void* rx_buffer{};
};
esp_err_t spi_bus_initialize(spi_host_device_t, const spi_bus_config_t*, int dma);
esp_err_t spi_bus_add_device(spi_host_device_t, const spi_device_interface_config_t*, spi_device_handle_t*);
esp_err_t spi_device_queue_trans(spi_device_handle_t, spi_transaction_t*, TickType_t);
esp_err_t spi_device_get_trans_result(spi_device_handle_t, spi_transaction_t**, TickType_t);
esp_err_t spi_bus_remove_device(spi_device_handle_t);
esp_err_t spi_bus_free(spi_host_device_t);
