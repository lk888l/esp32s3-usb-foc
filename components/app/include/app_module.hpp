#pragma once

#include <string_view>

enum class AppModuleState {
    stopped,
    initialized,
    cleanup_failed,
};

class AppModule {
public:
    virtual ~AppModule() = default;

    bool initialize()
    {
        if (state_ == AppModuleState::initialized) {
            return true;
        }
        if (state_ == AppModuleState::cleanup_failed) {
            return false;
        }

        if (on_initialize()) {
            state_ = AppModuleState::initialized;
            return true;
        }

        state_ = on_deinitialize() ? AppModuleState::stopped
                                   : AppModuleState::cleanup_failed;
        return false;
    }

    bool deinitialize()
    {
        if (state_ == AppModuleState::stopped) {
            return true;
        }

        const bool cleaned = on_deinitialize();
        state_ = cleaned ? AppModuleState::stopped : AppModuleState::cleanup_failed;
        return cleaned;
    }

    bool is_initialized() const { return state_ == AppModuleState::initialized; }
    AppModuleState state() const { return state_; }
    // Module names are registry keys and must reference storage that outlives the module.
    virtual std::string_view name() const = 0;
    virtual void process() {}

protected:
    AppModule() = default;
    virtual bool on_initialize() = 0;
    virtual bool on_deinitialize() = 0;

private:
    AppModule(const AppModule&) = delete;
    AppModule& operator=(const AppModule&) = delete;

    AppModuleState state_ = AppModuleState::stopped;
};
