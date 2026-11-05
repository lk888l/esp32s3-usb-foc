#include "settings_identity.hpp"
#include <bit>
#include <cassert>
#include <iostream>

using namespace knob;

int main() {
    constexpr auto as5600 = static_cast<uint32_t>(EncoderKind::as5600);
    constexpr auto mt6701 = static_cast<uint32_t>(EncoderKind::mt6701);
    // Legacy settings retain their configuration, but their calibration must
    // never energize a newly fitted encoder using the previous electrical zero.
    assert(known_settings_version(1));
    assert(settings_calibration_matches(1, as5600));
    assert(!settings_calibration_matches(1, mt6701));
    for (const auto saved : {as5600, mt6701}) {
        const auto version = encoder_settings_version(saved);
        assert(known_settings_version(version));
        for (const auto selected : {as5600, mt6701})
            assert(settings_calibration_matches(version, selected) == (saved == selected));
    }
    for (const auto unknown : {0U, 2U, 0x10003U, 0x20001U, 0xffffffffU}) {
        assert(!known_settings_version(unknown));
        assert(!settings_calibration_matches(unknown, as5600));
        assert(!settings_calibration_matches(unknown, mt6701));
    }
    Config saved;
    // Reproduce the pre-v8 record layout. Removing the public voltage knob
    // must not shift PI/haptic fields or discard an existing calibration.
    struct LegacyConfig {
        float bus_voltage, sense_gain, current_limit, voltage_limit;
        float current_kp, current_ki, width, strength, endstop, damping, snap;
        int min_position, max_position;
        Mode mode;
    };
    static_assert(sizeof(LegacyConfig) == sizeof(Config));
    const LegacyConfig legacy{12, .15f, .7f, 7.8f, .6f, 180, .2f, 6.5f, 8, .03f, .55f, 0, 11, Mode::spring};
    saved = std::bit_cast<Config>(legacy);
    assert(saved.legacy_voltage_limit == 7.8f && saved.current_kp == .6f && saved.current_ki == 180);
    saved.strength = 6.5f;
    saved.current_limit = .7f;
    saved.width = .2f;
    saved.mode = Mode::spring;
    Calibration saved_calibration{true, -1, 7, .75f};
    for (const auto version : {1U, encoder_settings_version(as5600), encoder_settings_version(mt6701)}) {
        for (const auto selected : {as5600, mt6701}) {
            Config restored;
            Calibration calibration;
            assert(restore_encoder_settings(version, saved, saved_calibration, selected, restored, calibration));
            assert(restored.strength == saved.strength && restored.current_limit == saved.current_limit);
            assert(restored.width == saved.width && restored.mode == saved.mode);
            assert(restored.legacy_voltage_limit == 7.8f);
            assert(restored.current_kp == .6f && restored.current_ki == 180);
            assert(max_output_voltage(restored.bus_voltage) > 0);
            const bool matches = settings_calibration_matches(version, selected);
            assert(calibration.valid == matches);
            if (matches) {
                assert(calibration.direction == -1 && calibration.electrical_zero == .75f);
            } else {
                assert(calibration.direction == 1 && calibration.electrical_zero == 0);
            }
        }
    }
    // A bad saved record must leave live defaults intact, including calibration.
    Config restored;
    Calibration calibration;
    Config invalid = saved;
    invalid.current_limit = max_motor_current + .01f;
    assert(!restore_encoder_settings(1, invalid, saved_calibration, mt6701, restored, calibration));
    assert(restored.strength == Config{}.strength && !calibration.valid);
    assert(!restore_encoder_settings(99, saved, saved_calibration, mt6701, restored, calibration));
    assert(restored.strength == Config{}.strength && !calibration.valid);
    std::cout << "PASS: legacy settings layout, retired voltage limit, and sensor-specific calibration identities.\n";
}
