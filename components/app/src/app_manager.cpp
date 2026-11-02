#include "app_manager.hpp"

namespace app {

RegistrationResult AppManager::register_module(std::unique_ptr<AppModule> module)
{
    if (state_ != State::configuring) {
        return {RegistrationStatus::invalid_state, module ? module->name() : std::string_view{}};
    }
    if (!module) {
        return {RegistrationStatus::null_module, {}};
    }
    if (module_count_ >= modules_.size()) {
        return {RegistrationStatus::registry_full, module->name()};
    }

    const std::string_view name = module->name();
    for (size_t index = 0; index < module_count_; ++index) {
        if (modules_[index] && modules_[index]->name() == name) {
            return {RegistrationStatus::duplicate_name, name};
        }
    }

    modules_[module_count_++] = std::move(module);
    return {RegistrationStatus::ok, name};
}

LifecycleResult AppManager::initialize_all()
{
    if (state_ == State::running) {
        return {LifecycleStatus::ok, {}};
    }
    if (state_ != State::configuring) {
        return {LifecycleStatus::invalid_state, {}};
    }

    size_t initialized = 0;
    for (; initialized < module_count_; ++initialized) {
        auto& module = modules_[initialized];
        if (!module->initialize()) {
            const std::string_view failed_name = module->name();
            if (module->state() == AppModuleState::cleanup_failed) {
                state_ = State::faulted;
                return {LifecycleStatus::rollback_failed, failed_name};
            }

            while (initialized > 0) {
                auto& rollback_module = modules_[--initialized];
                if (!rollback_module->deinitialize()) {
                    state_ = State::faulted;
                    return {LifecycleStatus::rollback_failed, rollback_module->name()};
                }
            }
            return {LifecycleStatus::module_failed, failed_name};
        }
    }

    state_ = State::running;
    return {LifecycleStatus::ok, {}};
}

LifecycleResult AppManager::deinitialize_all()
{
    if (state_ == State::configuring) {
        return {LifecycleStatus::ok, {}};
    }

    for (size_t remaining = module_count_; remaining > 0; --remaining) {
        auto& module = modules_[remaining - 1];
        if (module->state() != AppModuleState::stopped && !module->deinitialize()) {
            state_ = State::faulted;
            return {LifecycleStatus::module_failed, module->name()};
        }
    }

    state_ = State::configuring;
    return {LifecycleStatus::ok, {}};
}

void AppManager::process_all()
{
    if (state_ != State::running) {
        return;
    }

    for (size_t index = 0; index < module_count_; ++index) {
        auto& module = modules_[index];
        if (module && module->is_initialized()) {
            module->process();
        }
    }
}

} // namespace app
