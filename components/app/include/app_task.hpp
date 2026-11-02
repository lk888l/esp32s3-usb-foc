#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <string_view>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class AppTask {
public:
    AppTask(std::string_view name,
            uint32_t stack_size,
            UBaseType_t priority,
            BaseType_t core = tskNO_AFFINITY);
    virtual ~AppTask();

    bool start();
    bool stop(TickType_t timeout = pdMS_TO_TICKS(1500));
    bool is_running() const { return running_.load(std::memory_order_acquire); }

protected:
    virtual void main() = 0;
    virtual void cleanup() {}
    bool should_exit() const { return exit_requested_.load(std::memory_order_acquire); }

private:
    static void task_entry(void* argument);

    std::string name_;
    uint32_t stack_size_;
    UBaseType_t priority_;
    BaseType_t core_;
    std::atomic<TaskHandle_t> handle_{nullptr};
    std::atomic<bool> running_{false};
    std::atomic<bool> exit_requested_{false};
};
