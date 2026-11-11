#include "reference_json.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <string>

namespace {
using Json = std::unique_ptr<cJSON, decltype(&cJSON_Delete)>;
void check(bool condition, const char* message) {
    if (!condition) { std::fprintf(stderr, "%s\n", message); std::exit(1); }
}
Json own(cJSON* value) {
    check(value != nullptr, "fixture allocation failed");
    return Json(value, cJSON_Delete);
}
Json single(int mode = 4) {
    auto bank = own(knob::encode_reference_modes(knob::default_reference_modes()));
    auto patch = own(cJSON_CreateObject());
    const auto key = std::to_string(mode);
    check(cJSON_AddItemToObject(patch.get(), key.c_str(), cJSON_Duplicate(cJSON_GetObjectItemCaseSensitive(bank.get(), key.c_str()), true)),
          "fixture mode copy failed");
    return patch;
}
cJSON* record(const Json& patch) { return patch->child; }
void replace(cJSON* object, const char* name, cJSON* value) {
    check(value != nullptr && cJSON_ReplaceItemInObjectCaseSensitive(object, name, value), "fixture field replacement failed");
}
void number(cJSON* object, const char* name, double value) { replace(object, name, cJSON_CreateNumber(value)); }
void expect_rejected(const cJSON* patch, const char* message) {
    auto bank = knob::default_reference_modes();
    uint16_t mask = 0xffff;
    check(knob::decode_reference_modes(patch, bank, mask) != nullptr, message);
    check(mask == 0, "failed decode must clear a stale output mask");
}
void roundtrip_and_patch() {
    const auto defaults = knob::default_reference_modes();
    auto encoded = own(knob::encode_reference_modes(defaults));
    check(cJSON_GetArraySize(encoded.get()) == knob::reference_config_count, "encoder must include twelve independently keyed modes");
    auto decoded = defaults;
    for (auto& config : decoded) config.strength = 99;
    uint16_t mask = 0;
    check(!knob::decode_reference_modes(encoded.get(), decoded, mask), "default bank roundtrip accepted");
    check(mask == 0x0fff, "full bank reports all twelve replacement bits");
    for (int i = 0; i < knob::reference_config_count; ++i)
        check(knob::same_reference_config(defaults[i], decoded[i]), "default roundtrip retains every exact float and magnetic position");

    auto patch = single(8);
    number(record(patch), "strength", 42.5);
    check(cJSON_AddStringToObject(record(patch), "future_metadata", "retained by newer clients"), "extension fixture failed");
    check(!knob::decode_reference_modes(patch.get(), decoded, mask), "complete record with future extension accepted");
    check(mask == (1u << 4) && decoded[4].strength == 42.5f, "partial bank replaces the selected mode");
    for (int i = 0; i < knob::reference_config_count; ++i)
        if (i != 4) check(knob::same_reference_config(defaults[i], decoded[i]), "omitted modes remain untouched");

    auto empty = own(cJSON_CreateObject());
    const auto prior = decoded; mask = 0xffff;
    check(!knob::decode_reference_modes(empty.get(), decoded, mask) && mask == 0, "empty object is an explicit no-op");
    for (int i = 0; i < knob::reference_config_count; ++i)
        check(knob::same_reference_config(prior[i], decoded[i]), "empty patch preserves current device values");

    auto valid = single(4);
    char* bytes = cJSON_PrintUnformatted(valid.get()); check(bytes != nullptr, "print fixture failed");
    const std::string line(bytes); cJSON_free(bytes);
    for (size_t size = 0; size < line.size(); ++size) {
        Json truncated(cJSON_ParseWithLengthOpts(line.c_str(), size, nullptr, true), cJSON_Delete);
        check(!truncated, "truncated mode record is not a JSON frame");
    }
    auto parsed = own(cJSON_ParseWithLengthOpts(line.c_str(), line.size() + 1, nullptr, true));
    check(!knob::decode_reference_modes(parsed.get(), decoded, mask), "complete serialized single-mode frame accepted");
}
void completeness_and_identity() {
    const char* names[] = {"width", "endstop", "snap", "bias", "strength", "damping", "scale", "friction", "click",
                          "position", "min_position", "max_position", "bounded", "magnetic_positions", "endstop_damping", "click_enabled"};
    for (const char* field : names) {
        auto patch = single();
        cJSON_DeleteItemFromObjectCaseSensitive(record(patch), field);
        expect_rejected(patch.get(), "partial mode record cannot silently fill a default field");
    }
    expect_rejected(nullptr, "missing bank rejected");
    auto array = own(cJSON_CreateArray()); expect_rejected(array.get(), "array cannot masquerade as mode map");
    const char* invalid_modes[] = {"3", "16", "-1", "04", "+4", "4.0", "4 ", "x", "99999999999999"};
    for (const char* mode : invalid_modes) {
        auto source = single(); auto patch = own(cJSON_CreateObject());
        check(cJSON_AddItemToObject(patch.get(), mode, cJSON_Duplicate(record(source), true)), "mode key fixture failed");
        expect_rejected(patch.get(), "only canonical integer mode keys 4 through 15 accepted");
    }
    auto scalar = own(cJSON_CreateObject()); cJSON_AddNumberToObject(scalar.get(), "4", 1);
    expect_rejected(scalar.get(), "mode record must be an object");
    auto duplicate_mode = single();
    check(cJSON_AddItemToObject(duplicate_mode.get(), "4", cJSON_Duplicate(record(duplicate_mode), true)), "duplicate key fixture failed");
    expect_rejected(duplicate_mode.get(), "duplicate mode records rejected");
    auto duplicate_field = single(); cJSON_AddNumberToObject(record(duplicate_field), "strength", 12);
    expect_rejected(duplicate_field.get(), "duplicate numeric field rejected");
    auto duplicate_extension = single();
    cJSON_AddNumberToObject(record(duplicate_extension), "future", 1); cJSON_AddNumberToObject(record(duplicate_extension), "future", 2);
    expect_rejected(duplicate_extension.get(), "duplicate future field rejected consistently");

    auto mixed = single(4); number(record(mixed), "strength", 33);
    cJSON_AddNumberToObject(mixed.get(), "8", 123);
    auto bank = knob::default_reference_modes(); uint16_t mask = 0xffff;
    check(knob::decode_reference_modes(mixed.get(), bank, mask) != nullptr, "later malformed record rejects whole request");
    check(mask == 0, "no partial application mask escapes a later-record failure");
}
void numeric_contract() {
    struct Bound { const char* name; double minimum, maximum; };
    const Bound fields[] = {
        {"width", knob::pi / 180.0f, knob::pi}, {"endstop", 0, 20}, {"snap", .5, 2}, {"bias", 0, 1.5},
        {"strength", 0, 1000}, {"damping", 0, .5}, {"endstop_damping", 0, .5}, {"scale", 0, 1}, {"friction", 0, .45}, {"click", 0, .45},
    };
    for (const auto& field : fields) {
        for (double value : {std::nextafter(field.minimum, -std::numeric_limits<double>::infinity()),
                             std::nextafter(field.maximum, std::numeric_limits<double>::infinity()),
                             std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
            auto patch = single(); number(record(patch), field.name, value);
            expect_rejected(patch.get(), "double boundary must be checked before float32 rounds an illegal value inward");
        }
        for (double value : {field.minimum, field.maximum}) {
            auto patch = single(); number(record(patch), "snap", 2); number(record(patch), field.name, value);
            auto bank = knob::default_reference_modes(); uint16_t mask = 0;
            check(!knob::decode_reference_modes(patch.get(), bank, mask), "inclusive numeric endpoint accepted");
        }
        auto boolean = single(); replace(record(boolean), field.name, cJSON_CreateTrue());
        expect_rejected(boolean.get(), "boolean is not a numeric mode parameter");
        auto text = single(); replace(record(text), field.name, cJSON_CreateString("1"));
        expect_rejected(text.get(), "numeric strings cannot enter motor configuration");
    }
    for (const char* field : {"position", "min_position", "max_position"}) {
        for (double value : {-10001.0, 10001.0, .5, std::numeric_limits<double>::infinity()}) {
            auto patch = single(); number(record(patch), field, value);
            expect_rejected(patch.get(), "position fields enforce bounded integers");
        }
        auto boolean = single(); replace(record(boolean), field, cJSON_CreateFalse());
        expect_rejected(boolean.get(), "boolean is not an integer position");
    }
    auto flag = single(); replace(record(flag), "click_enabled", cJSON_CreateNumber(1));
    expect_rejected(flag.get(), "click enable requires an explicit boolean");
    auto bounded = single(); replace(record(bounded), "bounded", cJSON_CreateNumber(1));
    expect_rejected(bounded.get(), "bounded requires a JSON boolean");
    bounded = single(); replace(record(bounded), "bounded", cJSON_CreateTrue());
    expect_rejected(bounded.get(), "newly enabled range cannot keep inverted unbounded defaults");
    number(record(bounded), "min_position", -2); number(record(bounded), "max_position", 2); number(record(bounded), "position", 3);
    expect_rejected(bounded.get(), "initial position must fit enabled travel");
    auto snap = single(); number(record(snap), "bias", .51);
    expect_rejected(snap.get(), "cross-field inward snap margin enforced");
}
void magnetic_contract() {
    auto patch = single(14); auto* magnets = cJSON_GetObjectItemCaseSensitive(record(patch), "magnetic_positions");
    check(cJSON_AddItemToArray(magnets, cJSON_CreateNumber(2)), "duplicate magnet fixture failed");
    expect_rejected(patch.get(), "duplicate magnetic positions rejected");
    for (double value : {1.5, -10001.0, 10001.0, 32.0}) {
        patch = single(14); magnets = cJSON_GetObjectItemCaseSensitive(record(patch), "magnetic_positions");
        check(cJSON_ReplaceItemInArray(magnets, 0, cJSON_CreateNumber(value)), "magnet value fixture failed");
        expect_rejected(patch.get(), "magnetic values require integers within configured travel");
    }
    patch = single(14); magnets = cJSON_CreateArray();
    for (int i = 0; i < 9; ++i) cJSON_AddItemToArray(magnets, cJSON_CreateNumber(i));
    replace(record(patch), "magnetic_positions", magnets);
    expect_rejected(patch.get(), "ninth magnetic entry rejected");
    patch = single(14); replace(record(patch), "magnetic_positions", cJSON_CreateTrue());
    expect_rejected(patch.get(), "magnetic positions require an array");
    patch = single(14); magnets = cJSON_GetObjectItemCaseSensitive(record(patch), "magnetic_positions");
    check(cJSON_ReplaceItemInArray(magnets, 0, cJSON_CreateTrue()), "boolean magnet fixture failed");
    expect_rejected(patch.get(), "magnetic position cannot be a boolean");
    patch = single(); magnets = cJSON_GetObjectItemCaseSensitive(record(patch), "magnetic_positions");
    cJSON_AddItemToArray(magnets, cJSON_CreateNumber(0));
    expect_rejected(patch.get(), "other modes cannot receive hidden magnetic edits");
    patch = single(14); replace(record(patch), "magnetic_positions", cJSON_CreateArray());
    auto bank = knob::default_reference_modes(); uint16_t mask = 0;
    check(!knob::decode_reference_modes(patch.get(), bank, mask) && bank[10].magnetic_count == 0,
          "empty magnetic list deliberately removes sparse detents");
}
size_t allocation_call = 0, fail_allocation = 0, live_allocations = 0;
void* test_allocate(size_t size) {
    if (++allocation_call == fail_allocation) return nullptr;
    void* pointer = std::malloc(size);
    if (pointer) ++live_allocations;
    return pointer;
}
void test_free(void* pointer) {
    if (pointer) { check(live_allocations > 0, "unexpected allocator ownership"); --live_allocations; }
    std::free(pointer);
}
void allocation_failure() {
    const auto defaults = knob::default_reference_modes();
    const cJSON_Hooks hooks{test_allocate, test_free};
    bool all_sites_covered = false;
    for (size_t failure = 1; failure < 1000; ++failure) {
        allocation_call = live_allocations = 0; fail_allocation = failure;
        auto installed = hooks; cJSON_InitHooks(&installed);
        cJSON* bank = knob::encode_reference_modes(defaults);
        if (allocation_call < fail_allocation) {
            check(bank != nullptr, "encoder succeeds when all allocations succeed");
            all_sites_covered = true;
        } else {
            check(bank == nullptr, "allocation failure cannot escape as a partial encoded configuration");
        }
        cJSON_Delete(bank);
        check(live_allocations == 0, "encoder frees every allocation when construction fails");
        cJSON_InitHooks(nullptr);
        if (all_sites_covered) break;
    }
    check(all_sites_covered, "fault injection covered every cJSON encoder allocation site");
    auto corrupted = defaults; corrupted[10].magnetic_count = 9;
    Json invalid(knob::encode_reference_modes(corrupted), cJSON_Delete);
    check(!invalid, "corrupted magnetic count cannot read beyond fixed array during encoding");
}
}
int main() {
    roundtrip_and_patch(); completeness_and_identity(); numeric_contract(); magnetic_contract(); allocation_failure();
    std::puts("Reference JSON roundtrip, partial banks, atomic masks, complete records, strict boundaries and allocation failures passed.");
}
