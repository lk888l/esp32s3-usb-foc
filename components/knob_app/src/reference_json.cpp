#include "reference_json.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>

namespace knob {
namespace {
struct Field { const char* name; float ReferenceModeConfig::*member; double minimum, maximum; };
constexpr Field fields[] = {
    {"width", &ReferenceModeConfig::width, pi / 180.0f, pi},
    {"endstop", &ReferenceModeConfig::endstop, 0, 20},
    {"snap", &ReferenceModeConfig::snap, .5, 2},
    {"bias", &ReferenceModeConfig::bias, 0, 1.5},
    {"strength", &ReferenceModeConfig::strength, 0, 1000},
    {"damping", &ReferenceModeConfig::damping, 0, .5},
    {"endstop_damping", &ReferenceModeConfig::endstop_damping, 0, .5},
    {"scale", &ReferenceModeConfig::scale, 0, 1},
    {"friction", &ReferenceModeConfig::friction, 0, .45},
    {"click", &ReferenceModeConfig::click, 0, .45},
};
struct IntegerField { const char* name; int ReferenceModeConfig::*member; };
constexpr IntegerField integers[] = {
    {"position", &ReferenceModeConfig::position},
    {"min_position", &ReferenceModeConfig::min_position},
    {"max_position", &ReferenceModeConfig::max_position},
};
bool integer(const cJSON* value) {
    return cJSON_IsNumber(value) && std::isfinite(value->valuedouble) &&
        value->valuedouble >= -10000 && value->valuedouble <= 10000 &&
        std::floor(value->valuedouble) == value->valuedouble;
}
}

const char* decode_reference_modes(const cJSON* object, ReferenceModeBank& modes, uint16_t& mask) {
    mask = 0;
    if (!cJSON_IsObject(object)) return "reference_modes_required";
    uint16_t accepted = 0;
    for (const cJSON* entry = object->child; entry; entry = entry->next) {
        int preset = -1;
        for (int i = 0; i < reference_config_count; ++i) {
            char key[4]{}; std::snprintf(key, sizeof(key), "%d", i + reference_mode_first);
            if (entry->string && !std::strcmp(key, entry->string)) { preset = i; break; }
        }
        if (preset < 0 || (accepted & (1u << preset))) return "invalid_reference_mode";
        if (!cJSON_IsObject(entry)) return "reference_config_required";
        for (const cJSON* field = entry->child; field; field = field->next)
            for (const cJSON* next = field->next; next; next = next->next)
                if (field->string && next->string && !std::strcmp(field->string, next->string))
                    return "reference_duplicate_field";
        ReferenceModeConfig candidate{};
        for (auto field : fields) {
            const auto* value = cJSON_GetObjectItemCaseSensitive(entry, field.name);
            if (!cJSON_IsNumber(value) || !std::isfinite(value->valuedouble)) return "reference_field_missing";
            if (value->valuedouble < field.minimum || value->valuedouble > field.maximum)
                return "reference_field_range";
            candidate.*(field.member) = static_cast<float>(value->valuedouble);
        }
        for (auto field : integers) {
            const auto* value = cJSON_GetObjectItemCaseSensitive(entry, field.name);
            if (!integer(value)) return "invalid_reference_positions";
            candidate.*(field.member) = value->valueint;
        }
        const auto* bounded = cJSON_GetObjectItemCaseSensitive(entry, "bounded");
        if (!cJSON_IsBool(bounded)) return "reference_bounded_required";
        candidate.bounded = cJSON_IsTrue(bounded);
        const auto* click_enabled = cJSON_GetObjectItemCaseSensitive(entry, "click_enabled");
        if (!cJSON_IsBool(click_enabled)) return "reference_click_enabled_required";
        candidate.click_enabled = cJSON_IsTrue(click_enabled);
        const auto* magnets = cJSON_GetObjectItemCaseSensitive(entry, "magnetic_positions");
        if (!cJSON_IsArray(magnets) || cJSON_GetArraySize(magnets) > 8) return "invalid_reference_magnets";
        for (const auto* value = magnets->child; value; value = value->next) {
            if (!integer(value)) return "invalid_reference_magnets";
            candidate.magnetic_positions[candidate.magnetic_count++] = value->valueint;
        }
        if (const char* error = validate_reference_config(candidate, preset)) return error;
        modes[preset] = candidate;
        accepted |= 1u << preset;
    }
    mask = accepted;
    return nullptr;
}

cJSON* encode_reference_modes(const ReferenceModeBank& modes) {
    cJSON* bank = cJSON_CreateObject();
    if (!bank) return nullptr;
    for (int i = 0; i < reference_config_count; ++i) {
        char key[4]{}; std::snprintf(key, sizeof(key), "%d", i + reference_mode_first);
        cJSON* entry = cJSON_AddObjectToObject(bank, key);
        if (!entry) { cJSON_Delete(bank); return nullptr; }
        const auto& config = modes[i];
        for (auto field : fields)
            if (!cJSON_AddNumberToObject(entry, field.name, config.*(field.member))) { cJSON_Delete(bank); return nullptr; }
        for (auto field : integers)
            if (!cJSON_AddNumberToObject(entry, field.name, config.*(field.member))) { cJSON_Delete(bank); return nullptr; }
        if (!cJSON_AddBoolToObject(entry, "bounded", config.bounded) ||
            !cJSON_AddBoolToObject(entry, "click_enabled", config.click_enabled) ||
            config.magnetic_count > config.magnetic_positions.size()) { cJSON_Delete(bank); return nullptr; }
        cJSON* magnets = cJSON_CreateIntArray(config.magnetic_positions.data(), config.magnetic_count);
        if (!magnets || !cJSON_AddItemToObject(entry, "magnetic_positions", magnets)) {
            cJSON_Delete(magnets); cJSON_Delete(bank); return nullptr;
        }
    }
    return bank;
}
}
