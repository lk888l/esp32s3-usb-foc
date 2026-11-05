#include "sampling_recovery.hpp"
#include <cassert>
#include <cmath>
using namespace knob;

int main() {
    SamplingRecovery recovery;
    recovery.reset(100);
    assert(!recovery.observe(true, 300));
    assert(!recovery.observe(false, 500));
    assert(!recovery.observe(false, 2300));
    assert(recovery.observe(false, 2301));
    // Missing frames and an indefinitely closed window request only one zero
    // vector; repeated task wakes cannot flood or postpone the recovery.
    assert(!recovery.observe(false, 4000));
    assert(!recovery.observe(false, 100000));
    assert(!recovery.observe(true, 100200));
    assert(!recovery.observe(false, 102200));
    assert(recovery.observe(false, 102201));
    recovery.reset(200000);
    assert(!recovery.observe(false, 202000));
    assert(recovery.observe(false, 202001));

    // Fresh physical data remains usable after a delayed task interval. Stale,
    // duplicate, out-of-order, future and physically invalid data never enters PI.
    assert(SamplingRecovery::sample_usable(true, 12000, 5000, 12100));
    assert(SamplingRecovery::sample_usable(true, 12000, 5000, 14000));
    assert(!SamplingRecovery::sample_usable(true, 12000, 5000, 14001));
    assert(!SamplingRecovery::sample_usable(false, 12000, 5000, 12100));
    assert(!SamplingRecovery::sample_usable(true, 12000, 12000, 12100));
    assert(!SamplingRecovery::sample_usable(true, 11999, 12000, 12100));
    assert(!SamplingRecovery::sample_usable(true, 12101, 12000, 12100));
    assert(!SamplingRecovery::sample_usable(true, 0, -1, 100));
    // A fresh transport may enter zero-output offset acquisition even when
    // its latest physical window was rejected. That same invalid sample still
    // cannot advance offsets or PI; stale/missing/desynchronized data cannot arm.
    assert(SamplingRecovery::can_start_offsets(true, 12000, 12100));
    assert(!SamplingRecovery::sample_usable(false, 12000, 11000, 12100));
    assert(SamplingRecovery::can_start_offsets(true, 12000, 14000));
    assert(!SamplingRecovery::can_start_offsets(true, 12000, 14001));
    assert(!SamplingRecovery::can_start_offsets(false, 12000, 12100));
    assert(!SamplingRecovery::can_start_offsets(true, 0, 100));
    assert(!SamplingRecovery::can_start_offsets(true, 12101, 12100));
    assert(std::abs(SamplingRecovery::integration_dt(.0002f) - .0002f) < 1e-8f);
    assert(std::abs(SamplingRecovery::integration_dt(.007f) - .002f) < 1e-8f);

    EncoderFreshness encoder;
    using Action = EncoderFreshnessAction;
    assert(encoder.observe(0) == Action::ready);
    assert(encoder.observe(2000) == Action::ready);
    assert(encoder.observe(2001) == Action::zero_voltage);
    assert(encoder.observe(4000) == Action::waiting);
    assert(encoder.observe(10000) == Action::waiting);
    assert(encoder.observe(10001) == Action::lost);
    // One fresh sensor frame ends the mute, including after a transient age
    // excursion. Each subsequent episode requests exactly one zero vector.
    assert(encoder.observe(200) == Action::ready);
    assert(encoder.observe(3000) == Action::zero_voltage);
    assert(encoder.observe(3001) == Action::waiting);
    encoder.reset();
    assert(encoder.observe(3000) == Action::zero_voltage);
}
