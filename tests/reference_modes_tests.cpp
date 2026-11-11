#include "reference_modes.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <cstring>

namespace {
constexpr float degree = knob::pi / 180.0f;
void check(bool condition, const char* message) {
    if (!condition) { std::fprintf(stderr, "%s\n", message); std::exit(1); }
}
void near(float value, float expected, const char* message, float tolerance = 1e-5f) {
    check(std::abs(value - expected) <= tolerance, message);
}
void parameter_validation() {
    using knob::default_reference_config;
    using knob::validate_reference_config;
    for (int preset = 0; preset < knob::reference_config_count; ++preset)
        check(!validate_reference_config(default_reference_config(preset), preset), "every shipped default must validate");
    check(validate_reference_config(default_reference_config(0), -1), "negative preset rejected");
    check(validate_reference_config(default_reference_config(0), 12), "frictionless cannot enter reference bank");
    check(!default_reference_config(0).bounded && !default_reference_config(1).bounded,
          "free defaults explicitly advertise unbounded geometry");
    check(default_reference_config(0).max_position == -1, "old unbounded range default preserved");

    constexpr float knob::ReferenceModeConfig::* numbers[] = {
        &knob::ReferenceModeConfig::width, &knob::ReferenceModeConfig::endstop,
        &knob::ReferenceModeConfig::snap, &knob::ReferenceModeConfig::bias,
        &knob::ReferenceModeConfig::strength, &knob::ReferenceModeConfig::damping,
        &knob::ReferenceModeConfig::scale, &knob::ReferenceModeConfig::friction,
        &knob::ReferenceModeConfig::click, &knob::ReferenceModeConfig::endstop_damping,
    };
    for (const auto member : numbers) {
        auto config = default_reference_config(0);
        config.*member = std::numeric_limits<float>::quiet_NaN();
        check(validate_reference_config(config, 0), "NaN reference parameter rejected");
        config.*member = std::numeric_limits<float>::infinity();
        check(validate_reference_config(config, 0), "infinite reference parameter rejected");
        config.*member = -1;
        check(validate_reference_config(config, 0), "negative reference parameter rejected");
    }
    auto config = default_reference_config(0);
    config.width = degree; check(!validate_reference_config(config, 0), "minimum width accepted");
    config.width = knob::pi; check(!validate_reference_config(config, 0), "maximum width accepted");
    config.width = .99f * degree; check(validate_reference_config(config, 0), "subdegree width rejected");
    config = default_reference_config(0);
    config.strength = 1000; config.damping = .5f; config.scale = 1;
    config.friction = .45f; config.click = .45f; config.endstop = 20;
    config.snap = 2; config.bias = 1.5f;
    check(!validate_reference_config(config, 0), "advertised upper boundaries accepted together");
    constexpr float maxima[] = {knob::pi, 20, 2, 1.5f, 1000, .5f, 1, .45f, .45f, .5f};
    for (unsigned i = 0; i < std::size(numbers); ++i) {
        auto over = default_reference_config(0);
        over.*numbers[i] = std::nextafter(maxima[i], std::numeric_limits<float>::infinity());
        check(validate_reference_config(over, 0), "one float32 step above a numeric maximum rejected");
    }
    config = default_reference_config(11);
    config.bias = config.snap;
    check(validate_reference_config(config, 11), "zero inward snap threshold rejected");
    config.bias = .51f;
    check(validate_reference_config(config, 11), "less than 0.05 snap margin rejected");
    config = default_reference_config(0); config.bounded = true;
    check(validate_reference_config(config, 0), "enabling bounds requires valid explicit range");
    config.min_position = -2; config.max_position = 2; config.position = 3;
    check(validate_reference_config(config, 0), "initial position outside bounds rejected");
    config.position = 2;
    check(!validate_reference_config(config, 0), "initial position on endstop accepted");
    config.position = 10001; config.bounded = false;
    check(validate_reference_config(config, 0), "unbounded initial position still has serialization bounds");
    config = default_reference_config(10);
    config.magnetic_positions[1] = config.magnetic_positions[0];
    check(validate_reference_config(config, 10), "duplicate magnetic position rejected");
    config = default_reference_config(10); config.max_position = 20;
    check(validate_reference_config(config, 10), "magnetic positions outside new travel rejected");
    config = default_reference_config(10); config.magnetic_count = 9;
    check(validate_reference_config(config, 10), "oversized magnetic count rejected before indexing");
    config = default_reference_config(10);
    check(validate_reference_config(config, 8), "magnetic list cannot silently affect another preset");
    config.magnetic_count = 0;
    check(!validate_reference_config(config, 10), "empty magnetic list gives continuous smooth interior");
}

void parity_and_independence() {
    // Differential sweep against the established controller, including snaps,
    // reversals, saturation, idle centers and positive-work attenuation.
    for (float width : {1*degree, 10*degree, 60*degree})
    for (float gain : {1.0f, 20.0f, 100.0f})
    for (float damping : {0.0f, .003f, .04f}) {
        knob::Config old; old.mode=knob::Mode::detents; old.width=width;
        old.strength=gain; old.damping=damping; old.current_limit=.8f;
        auto c=knob::default_reference_config(0);
        c.width=width; c.strength=gain; c.damping=damping; c.snap=old.snap;
        c.scale=1; c.friction=0; c.click_enabled=false;
        knob::Haptics a; knob::ReferenceHaptics b; a.reset(0); b.reset(0,0,c);
        for (int n=0;n<1200;++n) {
            const float angle=4*width*std::sin(n*.013f);
            const float velocity=35*std::cos(n*.013f);
            near(b.step(0,c,angle,velocity,.001f,.8f),a.step(old,angle,velocity),
                 "new detent must match original current curve at equal settings",2e-4f);
            check(b.position()==-a.position(), "reference logical direction preserved");
        }
    }
    // Bounded geometry has opposite logical numbering but identical physical
    // force, including direct endpoint springs and explicit endpoint damping.
    {
        knob::Config old; old.mode=knob::Mode::bounded; old.min_position=-3; old.max_position=3;
        auto c=knob::default_reference_config(8); c.min_position=-3;c.max_position=3;c.position=0;
        c.strength=old.strength;c.damping=old.damping;c.endstop=old.endstop;c.width=old.width;
        c.endstop_damping=knob::elastic_min_damping;
        knob::Haptics a; knob::ReferenceHaptics b; a.reset(0); b.reset(0,8,c);
        for(int n=0;n<2000;++n){
            const float angle=1.2f*std::sin(n*.013f), velocity=30*std::cos(n*.013f);
            near(b.step(8,c,angle,velocity,.001f,.8f),a.step(old,angle,velocity),
                 "bounded detents and endpoints match original physical current",2e-4f);
        }
    }
    auto defaults=knob::default_reference_modes(), edited=defaults;
    edited[8].strength=37; edited[8].damping=.002f; edited[8].width=20*degree;
    for(int i=0;i<12;++i) if(i!=8)
        check(knob::same_reference_config(defaults[i],edited[i]),"per-mode tuning must not leak");
    knob::ReferenceHaptics h;
    auto c=defaults[8]; c.bounded=false; c.strength=1000;
    h.reset(0,8,c);
    near(h.step(8,c,3*degree,0,.001f,.8f),-.8f,"new detents use operating limit instead of old 450mA ceiling");
    near(h.step(8,c,3*degree,0,.001f,.2f),-.2f,"lower operating limit is enforced");
    c=defaults[2]; c.endstop=2; c.endstop_damping=.01f;
    h.reset(0,2,c);
    near(h.step(2,c,2*degree,1,.001f,.8f),-2*2*degree-.01f,
         "endpoint has direct gain, no dead zone, independent endpoint damping");
    c.endstop_damping=0; h.reset(0,2,c);
    near(h.step(2,c,2*degree,1,.001f,.8f),-2*2*degree,"zero endpoint damping is honored");
    c=defaults[7]; h.reset(0,7,c);
    check(h.step(7,c,.4f*degree,0,.001f,.8f)<-.1f,"fine detents have real center restoring torque without a pulse");
    for(int i=0;i<12;++i)check(!defaults[i].click_enabled,"factory clicks opt in only");
}
void optional_clicks() {
    auto c=knob::default_reference_config(0); c.strength=0;c.damping=0;c.click=.2f;
    knob::ReferenceHaptics h;
    h.reset(0,0,c);
    near(h.step(0,c,6*degree,0,.001f,.8f),0,"disabled click amplitude cannot generate pulses");
    c.click_enabled=true;h.reset(0,0,c);
    near(h.step(0,c,6*degree,0,.001f,.8f),-.2f,"first lobe opposes positive crossing");
    near(h.step(0,c,6*degree,0,.002f,.8f),.2f,"second lobe reverses");
    near(h.step(0,c,6*degree,0,.002f,.8f),0,"pulse stops after four milliseconds");
    h.reset(0,0,c);
    near(h.step(0,c,-6*degree,0,.001f,.8f),.2f,"reverse crossing reverses pulse consistently");
    h.reset(0,0,c);
    near(h.step(0,c,6*degree,0,.001f,.8f),-.2f,"rapid train first lobe");
    near(h.step(0,c,16*degree,0,.001f,.8f),-.2f,"crossing cannot reset pulse clock");
    near(h.step(0,c,26*degree,0,.001f,.8f),.2f,"release lobe survives rapid crossings");
    near(h.step(0,c,36*degree,0,.001f,.8f),.2f,"release lobe completes before retrigger");
    near(h.step(0,c,36*degree,0,.001f,.8f),0,"train ends without residual click current");
    c.click_enabled=false;h.reset(0,0,c);
    near(h.step(0,c,6*degree,0,.001f,.8f),0,"reset clears any outstanding pulse");
}
void geometry_and_migration() {
    knob::ReferenceHaptics h;
    auto c=knob::default_reference_config(10);
    h.reset(0,10,c);near(h.step(10,c,-2*degree,0,.001f,.8f),0,"magnetic gaps have no spring");
    h.step(10,c,-14*degree,0,.001f,.8f);
    check(h.position()==2,"magnetic position retained");
    check(h.step(10,c,-16*degree,0,.001f,.8f)>0,"selected magnet has restoring force");
    c=knob::default_reference_config(11);h.reset(0,11,c);
    h.step(11,c,-56*degree,0,.001f,.8f);check(h.position()==0,"center bias delays outward snap");
    h.step(11,c,-58*degree,0,.001f,.8f);check(h.position()==1,"outward threshold advances");
    h.step(11,c,-50*degree,0,.001f,.8f);check(h.position()==0,"return threshold retains bias");
    auto old=knob::legacy_reference_modes();auto migrated=knob::default_reference_modes();
    check(knob::migrate_reference_modes(old,migrated),"v1 defaults migrate");
    for(int i=0;i<12;++i)check(knob::same_reference_config(migrated[i],knob::default_reference_config(i)),"untouched defaults receive optimized presets");
    old[8].width=20*degree;old[8].strength=30;old[8].damping=.2f;old[8].click=.3f;
    check(knob::migrate_reference_modes(old,migrated),"customized v1 bank migrates");
    near(migrated[8].strength,6,"legacy P converted to physical units");
    near(migrated[8].damping,.04f,"legacy D converted to physical units");
    near(migrated[8].endstop,.8f,"legacy endpoint factor converted");
    near(migrated[8].width,20*degree,"custom geometry retained");
    near(migrated[8].scale,1,"converted gains not multiplied twice");
    check(!migrated[8].click_enabled&&migrated[8].click==.3f,"legacy amplitude retained but pulses opt-in");
    auto before=migrated;old[11].width=0;
    check(!knob::migrate_reference_modes(old,migrated),"invalid v1 bank rejects atomically");
    for(int i=0;i<12;++i)check(knob::same_reference_config(before[i],migrated[i]),"failed migration preserves destination");
    for(int i=0;i<12;++i){
        h.reset(0,i);near(h.step(i,0,0,.001f,.8f),0,"defaults quiet at start");
        near(h.step(i,1,1,.03f,.8f),0,"stale dt prevents output");
        near(h.step(i,1,1,.001f,0),0,"zero budget prevents output");
    }
}
}
int main(){parameter_validation();parity_and_independence();optional_clicks();geometry_and_migration();
    std::puts("Shared-detent parity, independent gains, optional pulses, geometry and v1 migration passed.");}
