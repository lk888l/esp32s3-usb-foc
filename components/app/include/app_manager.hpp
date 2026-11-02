#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string_view>

#include "app_module.hpp"

namespace app {

enum class RegistrationStatus {
    ok,
    null_module,
    duplicate_name,
    registry_full,
    invalid_state,
};

struct RegistrationResult {
    RegistrationStatus status = RegistrationStatus::ok;
    std::string_view module_name{};

    explicit operator bool() const { return status == RegistrationStatus::ok; }
};

enum class LifecycleStatus {
    ok,
    invalid_state,
    module_failed,
    rollback_failed,
};

struct LifecycleResult {
    LifecycleStatus status = LifecycleStatus::ok;
    std::string_view module_name{};

    explicit operator bool() const { return status == LifecycleStatus::ok; }
};

class AppManager {
public:
    static constexpr size_t kMaxModules = 8;

    RegistrationResult register_module(std::unique_ptr<AppModule> module);
    LifecycleResult initialize_all();
    LifecycleResult deinitialize_all();
    void process_all();

    bool is_running() const { return state_ == State::running; }
    bool has_cleanup_failure() const { return state_ == State::faulted; }
    size_t module_count() const { return module_count_; }

private:
    enum class State {
        configuring,
        running,
        faulted,
    };

    std::array<std::unique_ptr<AppModule>, kMaxModules> modules_{};
    size_t module_count_ = 0;
    State state_ = State::configuring;
};

} // namespace app
