#include "app_task.hpp"

#include "esp_log.h"

namespace {

constexpr char kTag[] = "app_task";

} // namespace

AppTask::AppTask(std::string_view name,
                 uint32_t stack_size,
                 UBaseType_t priority,
                 BaseType_t core)
    : name_(name), stack_size_(stack_size), priority_(priority), core_(core)
{
}

AppTask::~AppTask()
{
    if (!stop(portMAX_DELAY)) {
        ESP_LOGE(kTag, "task '%s' could not be stopped safely", name_.c_str());
    }
}

bool AppTask::start()
{
    if (handle_.load(std::memory_order_acquire) || running_.load(std::memory_order_acquire)) {
        return false;
    }

    exit_requested_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    TaskHandle_t handle = nullptr;
    const BaseType_t result = xTaskCreatePinnedToCore(
        task_entry, name_.c_str(), stack_size_, this, priority_, &handle, core_);
    if (result != pdPASS) {
        running_.store(false, std::memory_order_release);
        return false;
    }
    handle_.store(handle, std::memory_order_release);
    return true;
}

bool AppTask::stop(TickType_t timeout)
{
    if (!handle_.load(std::memory_order_acquire) && !running_.load(std::memory_order_acquire)) {
        return true;
    }

    exit_requested_.store(true, std::memory_order_release);
    const TickType_t started = xTaskGetTickCount();
    while (running_.load(std::memory_order_acquire)) {
        if (timeout != portMAX_DELAY && xTaskGetTickCount() - started >= timeout) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return true;
}

void AppTask::task_entry(void* argument)
{
    auto* self = static_cast<AppTask*>(argument);
    self->main();
    self->cleanup();
    self->handle_.store(nullptr, std::memory_order_release);
    self->running_.store(false, std::memory_order_release);
    vTaskDelete(nullptr);
}
