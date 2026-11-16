#include "wheel_tracker.hpp"
#include <cassert>
#include <cmath>
#include <limits>
using knob::usb::WheelTracker;
int main() {
    WheelTracker w;
    int64_t t = 1000;
    auto sample = [&](float ticks, bool valid = true) { t += 1000; w.update(ticks * w.step_radians, t, valid); };
    sample(0); assert(w.pending() == 0);
    sample(.54f); assert(w.pending() == 0);
    sample(.56f); assert(w.pending() == 1); w.sent(1);
    for (int i = 0; i < 100; ++i) { sample(.54f); sample(.56f); assert(w.pending() == 0); }
    sample(.44f); assert(w.pending() == -1); w.sent(-1);
    w.reset(); sample(0);
    int total = 0;
    for (int i = 1; i <= 720; ++i) {
        sample(std::fmod(i * .1f, 36.f)); total += w.pending(); w.sent(w.pending());
    }
    assert(total == 72);
    for (int i = 719; i >= 0; --i) {
        sample(std::fmod(i * .1f, 36.f)); total += w.pending(); w.sent(w.pending());
    }
    assert(total == 0);
    w.reset(); sample(0); sample(.4f); sample(10, false); sample(20); assert(w.pending() == 0);
    sample(20.6f); assert(w.pending() == 1);
    t += 21000; sample(30); assert(w.pending() == 0);
    sample(30.6f); assert(w.pending() == 1);
    w.reset(); sample(0); sample(10); assert(w.pending() == 0); // Impossible 1 ms jump.
    sample(11); assert(w.pending() == 0); // Re-anchor after jump.
    w.update(std::numeric_limits<float>::quiet_NaN(), ++t, true); assert(w.pending() == 0);
    sample(2); w.update(3, t - 1, true); assert(w.pending() == 0);
    w.reset(); sample(0);
    for (int i = 1; i < 300; ++i) sample(std::fmod(i * 1.f, 36.f));
    assert(w.pending() == 127); w.sent(127); assert(w.pending() == 0);
    w.reset(); sample(0);
    for (int i = 1; i < 300; ++i) sample(std::fmod(-i * 1.f, 36.f));
    assert(w.pending() == -127);
}
