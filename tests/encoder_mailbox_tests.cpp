#include "encoder_mailbox.hpp"
#include <cassert>
#include <iostream>
using namespace knob;

static EncoderFrame frame(int64_t timestamp, int error, uint32_t failures, uint32_t retries = 0) {
    EncoderFrame result;
    result.timestamp_us = timestamp;
    result.angle = timestamp * .001f;
    result.error = error;
    result.transport_error = error == 7 ? error : 0;
    result.magnet = error == 8 ? 0 : 0x20;
    result.failures = failures;
    result.retries = retries;
    return result;
}

int main() {
    EncoderMailbox mailbox;
    assert(mailbox.consume().timestamp_us == 0);
    mailbox.publish(frame(1, 0, 0));
    mailbox.publish(frame(2, 7, 1, 1));
    mailbox.publish(frame(3, 0, 1, 2));
    auto sample = mailbox.consume();
    assert(sample.timestamp_us == 2 && sample.error == 7 && sample.transport_error == 7);
    assert(sample.failures == 1 && sample.retries == 2);
    sample = mailbox.consume();
    assert(sample.timestamp_us == 3 && sample.error == 0 && sample.failures == 1);

    // Multiple failures before a slower motor read retain the first cause,
    // without hiding the accumulated count behind that older frame's metadata.
    mailbox.publish(frame(4, 8, 2, 2));
    mailbox.publish(frame(5, 7, 3, 3));
    mailbox.publish(frame(6, 0, 3, 3));
    sample = mailbox.consume();
    assert(sample.timestamp_us == 4 && sample.error == 8 && sample.magnet == 0);
    assert(sample.failures == 3 && sample.retries == 3);
    sample = mailbox.consume();
    assert(sample.timestamp_us == 6 && sample.error == 0 && sample.failures == 3);

    // A failure arriving after consumption gets its own pending slot.
    mailbox.publish(frame(7, 7, 4, 4));
    assert(mailbox.consume().timestamp_us == 7);
    mailbox.publish(frame(8, 8, 5, 4));
    mailbox.publish(frame(9, 0, 5, 4));
    assert(mailbox.consume().timestamp_us == 8);
    assert(mailbox.consume().timestamp_us == 9);

    // clear acknowledges old events while retaining the newest healthy sample
    // and lifetime counts, so it does not force an unnecessary extra clear.
    mailbox.publish(frame(10, 7, 6, 5));
    mailbox.publish(frame(11, 0, 6, 5));
    mailbox.acknowledge_errors();
    sample = mailbox.consume();
    assert(sample.timestamp_us == 11 && sample.error == 0 && sample.failures == 6);

    // A currently failed sensor cannot become healthy merely through clear.
    mailbox.publish(frame(12, 8, 7, 5));
    mailbox.acknowledge_errors();
    sample = mailbox.consume();
    assert(sample.timestamp_us == 12 && sample.error == 8 && sample.failures == 7);
    mailbox.publish(frame(13, 7, 8, 6));
    mailbox.publish(frame(14, 0, 8, 6));
    sample = mailbox.consume();
    assert(sample.timestamp_us == 13 && sample.error == 7 && sample.failures == 8);

    // A new worker run must not replay an old error or an old healthy angle.
    mailbox.publish(frame(15, 8, 9, 6));
    mailbox.reset();
    sample = mailbox.consume();
    assert(sample.timestamp_us == 0 && sample.error == 0 && sample.failures == 0);
    mailbox.publish(frame(16, 0, 0));
    assert(mailbox.consume().timestamp_us == 16);

    // The business layer must retain the driver's classified fault alongside
    // raw status. A healthy MT6701 status is zero, unlike AS5600's MD bit.
    auto crc_failure = frame(17, 9, 1);
    crc_failure.magnet = 0;
    crc_failure.fault = EncoderFault::crc;
    mailbox.publish(crc_failure);
    auto healthy_mt = frame(18, 0, 1);
    healthy_mt.magnet = 0;
    healthy_mt.fault = EncoderFault::none;
    mailbox.publish(healthy_mt);
    sample = mailbox.consume();
    assert(sample.error == 9 && sample.fault == EncoderFault::crc && sample.magnet == 0);
    sample = mailbox.consume();
    assert(sample.error == 0 && sample.fault == EncoderFault::none && sample.magnet == 0);
    auto tracking_failure = frame(19, 10, 2);
    tracking_failure.magnet = 8;
    tracking_failure.fault = EncoderFault::loss_of_track;
    mailbox.publish(tracking_failure);
    assert(mailbox.consume().fault == EncoderFault::loss_of_track);
    std::cout << "PASS: first unread encoder failure, current counters, clear and restart recovery.\n";
}
