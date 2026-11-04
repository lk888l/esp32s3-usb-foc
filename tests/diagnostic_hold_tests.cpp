#include "diagnostics.hpp"
#include <cassert>
#include <cmath>
#include <limits>
using namespace knob;
int main() {
    DiagnosticHold hold;
    assert(!hold.start(.3f, 0, 1, .17f, 1, 100));
    assert(!hold.start(.15f, 2, 1, .17f, 1, 100));
    assert(!hold.start(.15f, 0, 1, .17f, .1f, 100));
    assert(!hold.start(std::numeric_limits<float>::quiet_NaN(), 0, 1, .17f, 1, 100));
    assert(!hold.active());
    assert(hold.start(-.15f, 0, 1, .17f, 1, 100));
    assert(hold.current() == -.15f);
    assert(!hold.start(.15f, 0, 1, .17f, 1, 700000)); // Cannot extend or reverse.
    assert(!hold.must_stop(750099, 1.001f, .1f));
    assert(hold.must_stop(750100, 1, 0)); // Fixed deadline, independent of host polls.
    assert(hold.must_stop(101, 1.043f, 0));
    assert(hold.must_stop(101, .957f, 0)); // Release excursion, either direction.
    assert(hold.must_stop(101, 1, 5));
    assert(hold.must_stop(101, 1, std::numeric_limits<float>::quiet_NaN()));
    hold.reset();
    assert(!hold.active() && hold.current() == 0);
    assert(!hold.must_stop(900000, 1, 10));
    assert(hold.start(.1f, 0, 1, 1, 1, 900000));
    assert(hold.must_stop(900001, 1.051f, 0)); // Excursion capped even for wide detents.
}
