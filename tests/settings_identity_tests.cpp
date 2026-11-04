#include "settings_identity.hpp"
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
    std::cout << "PASS: legacy and sensor-specific calibration identities.\n";
}
