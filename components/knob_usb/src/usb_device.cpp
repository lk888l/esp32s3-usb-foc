#include "usb_device.hpp"
#include "wheel_tracker.hpp"
#include "app_task.hpp"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "soc/rtc_cntl_reg.h"
#include "hal/usb_serial_jtag_ll.h"
#include <cstdio>

namespace {
// Windows mouhid requires pointer axes in the mouse collection. These fields
// are always zero; the signed relative wheel is the only generated input.
constexpr uint8_t wheel_descriptor[] = {
    0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x09, 0x01, 0xa1, 0x00,
    0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00, 0x25, 0x01,
    0x95, 0x03, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05, 0x81, 0x03,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
    0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x03,
    0x81, 0x06, 0xc0, 0xc0,
};
constexpr tusb_desc_device_t device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t), .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200, .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON, .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = 64, .idVendor = 0x303a, .idProduct = 0x4005,
    .bcdDevice = 0x0101, .iManufacturer = 1, .iProduct = 2,
    .iSerialNumber = 3, .bNumConfigurations = 1,
};
constexpr uint8_t configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 3, 0, TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_HID_DESC_LEN, 0, 100),
    TUD_CDC_DESCRIPTOR(0, 4, 0x81, 8, 0x02, 0x82, 64),
    TUD_HID_DESCRIPTOR(2, 5, HID_ITF_PROTOCOL_NONE, sizeof(wheel_descriptor), 0x83, 8, 1),
};
char serial_number[13]{};
const char* strings[] = {"\x09\x04", "SmartKnob", "SmartKnob S3 Wheel", serial_number,
                         "SmartKnob Configuration", "SmartKnob Mouse Wheel"};
struct EncoderInput { float angle = 0; int64_t timestamp = 0; bool valid = false; };
EncoderInput encoder_input;
portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t wheel_mutex = nullptr;
knob::usb::WheelTracker tracker;
knob::usb::WheelStatus status;
std::atomic<uint32_t> usb_epoch{0};
std::atomic<bool> request_bootloader{false};
uint32_t baud = 115200;
bool previous_dtr = false;
bool installed = false;

void device_event(tinyusb_event_t*, void*) { usb_epoch.fetch_add(1); }
void line_coding(int, cdcacm_event_t* event) {
    baud = event->line_coding_changed_data.p_line_coding->bit_rate;
}
void line_state(int, cdcacm_event_t* event) {
    const bool dtr = event->line_state_changed_data.dtr;
    if (baud == 1200 && previous_dtr && !dtr) request_bootloader.store(true);
    previous_dtr = dtr;
}
class WheelWorker final : public AppTask {
public:
    WheelWorker() : AppTask("knob_wheel", 3072, 7, 0) {}
private:
    void main() override {
        TickType_t wake = xTaskGetTickCount();
        uint32_t epoch = usb_epoch.load();
        int64_t blocked_since = 0;
        while (!should_exit()) {
            EncoderInput input;
            portENTER_CRITICAL(&encoder_mux); input = encoder_input; portEXIT_CRITICAL(&encoder_mux);
            const int64_t now = esp_timer_get_time();
            xSemaphoreTake(wheel_mutex, portMAX_DELAY);
            const uint32_t next_epoch = usb_epoch.load();
            if (epoch != next_epoch) { tracker.reset(); epoch = next_epoch; }
            status.mounted = tud_mounted(); status.suspended = tud_suspended();
            status.ready = status.enabled && status.mounted && !status.suspended &&
                knob::usb::input_allowed.load(std::memory_order_relaxed) && input.valid &&
                input.timestamp > 0 && now - input.timestamp <= knob::usb::WheelTracker::maximum_gap_us;
            tracker.update(input.angle, input.timestamp, status.ready);
            if (!status.ready) blocked_since = 0;
            else if (!tud_hid_ready()) {
                if (!blocked_since) blocked_since = now;
                if (now - blocked_since > 20000) tracker.reset();
            } else {
                // Also rebase on the first ready iteration after a long stall.
                if (blocked_since && now - blocked_since > 20000) tracker.reset();
                blocked_since = 0;
                const int8_t ticks = tracker.pending();
                const int8_t report[] = {0, 0, 0, ticks};
                if (ticks && tud_hid_report(0, report, sizeof(report))) {
                    tracker.sent(ticks); ++status.reports;
                    status.steps += ticks < 0 ? -ticks : ticks;
                }
            }
            xSemaphoreGive(wheel_mutex);
            vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
        }
    }
} worker;
}

extern "C" const uint8_t* tud_hid_descriptor_report_cb(uint8_t) { return wheel_descriptor; }
extern "C" uint16_t tud_hid_get_report_cb(uint8_t, uint8_t, hid_report_type_t, uint8_t*, uint16_t) { return 0; }
extern "C" void tud_hid_set_report_cb(uint8_t, uint8_t, hid_report_type_t, const uint8_t*, uint16_t) {}

namespace knob::usb {
bool initialize() {
    wheel_mutex = xSemaphoreCreateMutex();
    if (!wheel_mutex) return false;
    uint8_t mac[6]{};
    if (esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY) != ESP_OK) return false;
    std::snprintf(serial_number, sizeof(serial_number), "%02X%02X%02X%02X%02X%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    tinyusb_config_t config = TINYUSB_DEFAULT_CONFIG(device_event);
    config.task = {.size = 4096, .priority = 6, .xCoreID = 0};
    config.descriptor.device = &device_descriptor;
    config.descriptor.string = strings;
    config.descriptor.string_count = sizeof(strings) / sizeof(strings[0]);
    config.descriptor.full_speed_config = configuration_descriptor;
    if (tinyusb_driver_install(&config) != ESP_OK) return false;
    installed = true;
    tinyusb_config_cdcacm_t cdc{};
    cdc.cdc_port = TINYUSB_CDC_ACM_0;
    cdc.callback_line_state_changed = line_state;
    cdc.callback_line_coding_changed = line_coding;
    if (tinyusb_cdcacm_init(&cdc) != ESP_OK) return false;
    return worker.start();
}
bool shutdown() {
    if (!worker.stop()) return false;
    if (installed) {
        tinyusb_cdcacm_deinit(TINYUSB_CDC_ACM_0);
        if (tinyusb_driver_uninstall() != ESP_OK) return false;
        installed = false;
    }
    if (wheel_mutex) { vSemaphoreDelete(wheel_mutex); wheel_mutex = nullptr; }
    return true;
}
int read(uint8_t* bytes, size_t capacity) {
    const int64_t deadline = esp_timer_get_time() + 20000;
    do {
        const auto count = tud_cdc_read(bytes, capacity);
        if (count) return static_cast<int>(count);
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (esp_timer_get_time() < deadline);
    return 0;
}
bool write(const char* bytes, size_t size) {
    const int64_t deadline = esp_timer_get_time() + 100000;
    size_t sent = 0;
    while (sent < size && tud_ready() && esp_timer_get_time() < deadline) {
        // Legacy clients deliberately leave DTR low. TinyUSB otherwise treats
        // this FIFO as overwritable, so never offer more than its free space.
        const size_t available = tud_cdc_write_available();
        sent += tud_cdc_write(bytes + sent, std::min(size - sent, available));
        tud_cdc_write_flush();
        if (sent < size) vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (sent != size) tud_cdc_write_clear();
    return sent == size;
}
void publish_encoder(float angle, int64_t timestamp_us, bool valid) {
    portENTER_CRITICAL(&encoder_mux);
    encoder_input = {angle, timestamp_us, valid};
    portEXIT_CRITICAL(&encoder_mux);
}
void set_wheel_enabled(bool enabled) {
    xSemaphoreTake(wheel_mutex, portMAX_DELAY);
    if (status.enabled != enabled) { tracker.reset(); status.ready = false; }
    status.enabled = enabled;
    xSemaphoreGive(wheel_mutex);
}
WheelStatus wheel_status() {
    if (!wheel_mutex) return {};
    xSemaphoreTake(wheel_mutex, portMAX_DELAY);
    const auto result = status;
    xSemaphoreGive(wheel_mutex);
    return result;
}
bool bootloader_requested() { return request_bootloader.load(); }
[[noreturn]] void enter_bootloader() {
    input_allowed.store(false);
    shutdown();
    // Restore the ROM Serial/JTAG PHY and force download on the next reset.
    // No eFuse or NVS writes. Normal 115200-baud DTR/RTS never resets the board.
    usb_serial_jtag_ll_phy_enable_external(false);
    REG_SET_BIT(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    esp_restart();
    __builtin_unreachable();
}
}
