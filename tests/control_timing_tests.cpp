#include "control_timing.hpp"
#include <cassert>
#include <cmath>
using namespace knob;

static bool near(float a, float b) { return std::abs(a - b) < 1e-7f; }

int main() {
    // Same constant current error over 2 ms: accepting one in three frames
    // must retain the same integral gain as accepting every 200 us frame.
    ControlSampleClock regular_clock, sparse_clock;
    Pi regular_pi, sparse_pi;
    float regular = 0, sparse = 0;
    for (int frame = 0; frame < 10; ++frame) {
        const int64_t timestamp = 100000 + frame * 200;
        const auto regular_dt = regular_clock.accept(timestamp, 200);
        regular = regular_pi.step(.5f, regular_dt.integration_s, 0, 60, 10);
        if (frame % 3 == 0) {
            const auto sparse_dt = sparse_clock.accept(timestamp, 200);
            assert(near(sparse_dt.elapsed_s, frame ? .0006f : .0002f));
            sparse = sparse_pi.step(.5f, sparse_dt.integration_s, 0, 60, 10);
        }
        // Rejected frames never advance sparse_clock or run its PI.
    }
    assert(near(regular, .06f));
    assert(near(sparse, regular));

    // Deduplication/invalid timestamps cannot erase an elapsed interval.
    assert(sparse_clock.accept(101800, 200).integration_s == 0);
    assert(sparse_clock.accept(101799, 200).integration_s == 0);
    assert(sparse_clock.accept(0, 200).integration_s == 0);
    assert(near(sparse_clock.accept(102400, 200).elapsed_s, .0006f));

    const auto delayed = sparse_clock.accept(109400, 200);
    assert(near(delayed.elapsed_s, .007f));
    assert(near(delayed.integration_s, .002f));
    // The PI and its clock reset together on feedback recovery/re-enable.
    sparse_pi.reset(); sparse_clock.reset();
    const auto restarted = sparse_clock.accept(2000000, 200);
    assert(near(restarted.elapsed_s, .0002f));
    assert(near(sparse_pi.step(.5f, restarted.integration_s, 0, 60, 10), .006f));
}
