#include "live_config.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace knob;
static bool near(float a, float b) { return std::abs(a-b) < 1e-5f; }
int main() {
    Config c; c.mode = Mode::reference_coarse_strong;
    const auto bank = default_reference_modes(); auto patch = bank;
    FrictionlessConfig frictionless; LiveConfigChange change;
    auto plan = [&](ConfigState state, const Config& next, uint16_t mask) {
        return plan_live_config(state, c, next, bank, patch, mask, frictionless, &frictionless, change);
    };
    patch[8].strength += 1;
    assert(!plan(ConfigState::active,c,1u<<8) && change.reference_feel && change.output);
    assert(!change.mode && !change.frictionless && !change.electrical);
    auto invalid = c; invalid.current_kp += 1;
    assert(plan(ConfigState::active,invalid,1u<<8));
    assert(plan(ConfigState::unavailable,c,1u<<8));
    patch[9].width=0;
    assert(plan(ConfigState::active,c,(1u<<8)|(1u<<9))); // Whole patch fails.
    assert(same_reference_config(bank[8],default_reference_config(8)));
    patch=bank;patch[9].strength+=1;
    assert(!plan(ConfigState::active,c,1u<<9) && !change.output);
    patch=bank;patch[8].position+=1;
    assert(!plan(ConfigState::active,c,1u<<8) && !change.reference_feel && !change.output);
    auto next=c;next.current_limit=.2f;
    assert(!plan(ConfigState::active,next,0) && change.current && change.output);

    ReferenceHaptics ref; auto p=bank[8];p.width=.1f;p.damping=0;p.click=0;
    ref.reset(0,8,p);ref.step(8,p,-.42f,0,.001f,.8f);
    assert(ref.position()==4);const auto sub=ref.sub_position();
    auto tuned=p;tuned.strength+=2;
    ref.reconfigure(-.42f,8,p,tuned);ref.step(8,tuned,-.42f,0,.001f,.8f);
    assert(ref.position()==4 && near(ref.sub_position(),sub));
    p=tuned;tuned.width=.2f;ref.reconfigure(-.42f,8,p,tuned);
    assert(ref.position()==4 && near(ref.sub_position(),0));
    p=tuned;tuned.max_position=2;ref.reconfigure(-.42f,8,p,tuned);
    assert(ref.position()==2 && near(ref.sub_position(),0));
    p=tuned;tuned.position=1;ref.reconfigure(-.42f,8,p,tuned);
    assert(ref.position()==2);ref.reset(-.42f,8,tuned);assert(ref.position()==1);
    // Changing a snap threshold does not manufacture a click.
    p=bank[0];p.strength=p.damping=0;p.width=.1f;p.snap=1;p.click=.2f;p.click_enabled=true;
    ref.reset(0,0,p);ref.step(0,p,.08f,0,.001f,.8f);
    tuned=p;tuned.snap=.55f;ref.reconfigure(.08f,0,p,tuned);
    assert(near(ref.step(0,tuned,.08f,0,.001f,.8f),0));

    Haptics h;Config old;old.width=.1f;old.mode=Mode::bounded;old.max_position=10;
    h.reset(0);h.step(old,.42f,0);assert(h.position()==4);
    next=old;next.width=.2f;h.reconfigure(old,next,.42f);assert(h.position()==4);
    old=next;next.max_position=2;h.reconfigure(old,next,.42f);assert(h.position()==2);

    HapticTransition transition;assert(near(transition.step(.5f,.8f,.001f),.5f));
    transition.begin();assert(near(transition.step(-.5f,.8f,.001f),.45f));
    for(int i=0;i<19;++i)transition.step(-.5f,.8f,.001f);
    assert(near(transition.step(-.5f,.8f,.001f),-.5f) && !transition.active());
    assert(near(transition.step(.7f,.8f,.001f),.7f)); // Ordinary clicks are immediate.
    transition.begin();const auto during=transition.step(-.7f,.8f,.001f);
    transition.begin();assert(near(transition.step(.7f,.8f,.001f),during+(.7f-during)*.05f));
    assert(std::abs(transition.step(.7f,.1f,.001f))<=.1f); // Lower budget wins now.
    assert(near(transition.step(.7f,.8f,.1f),0));
    // Keeping the same hold state through tuning preserves its acquired goal.
    Frictionless hold;for(int i=0;i<1800;++i)hold.step(frictionless,2,.001f,.8f);
    const float goal=hold.reference();assert(goal>0);
    frictionless.hold_kp*=.5f;hold.step(frictionless,2,.001f,.8f);
    assert(near(hold.reference(),goal));
    std::puts("Live configuration isolation, geometry, hold continuity and bounded handover passed.");
}
