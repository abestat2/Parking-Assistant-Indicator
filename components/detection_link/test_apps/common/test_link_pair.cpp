// Tier 1: a receiver and a detector talking through a simulated radio with scripted losses

#include "unity.h"
#include "fake_transport.hpp"
#include <functional>

using namespace detection_link;
using namespace test_support;

static uint32_t s_next_id;
static uint32_t pair_exchange_ids() { return s_next_id++; }

// Each update round the radio carries any packet in flight to the other side and reports its
// result. A dropped packet is reported as not delivered, as ESP-NOW does when the peer's MAC
// layer does not acknowledge it.
struct Pair {
    FakeTransport receiver_radio;
    FakeTransport detector_radio;
    DetectionLink receiver{receiver_radio, fake_clock};
    DetectionLink detector{detector_radio, fake_clock};
    // Return true to drop the packet
    std::function<bool(const Message&)> drop = [](const Message&) { return false; };
    std::vector<LinkEvent> receiver_events;
    std::vector<LinkEvent> detector_events;

    Pair()
    {
        g_now_ms = 0;
        s_next_id = 1;
        LinkConfig config;
        config.response_timeout_ms = 30;
        config.retry_delay_ms = 5;
        config.exchange_deadline_ms = 300;
        config.detector_ack_deadline_ms = 400;
        config.exchange_id_source = pair_exchange_ids;

        config.transport.peer = DETECTOR_MAC;
        TEST_ASSERT_EQUAL(ESP_OK, receiver.begin(Role::Receiver, config));
        config.transport.peer = RECEIVER_MAC;
        TEST_ASSERT_EQUAL(ESP_OK, detector.begin(Role::Detector, config));
    }

    void carry(FakeTransport& from, FakeTransport& to, const MacAddress& from_mac)
    {
        if (!from.in_flight()) {
            return;
        }
        const Message& message = from.last_sent().message;
        const bool dropped = drop(message);
        if (!dropped) {
            to.receive(from_mac, message);
        }
        from.complete(!dropped);
    }

    // Run 1 ms rounds until the receiver's exchange ends (or a round limit is hit)
    void run_exchange()
    {
        TEST_ASSERT_EQUAL(ESP_OK, receiver.request_status());
        for (int round = 0; round < 1000 && receiver.exchange_in_progress(); ++round) {
            carry(receiver_radio, detector_radio, RECEIVER_MAC);
            carry(detector_radio, receiver_radio, DETECTOR_MAC);
            advance(1);
            detector.update();
            receiver.update();
        }
        TEST_ASSERT_FALSE(receiver.exchange_in_progress());
        // Let anything still in flight land on the detector
        for (int round = 0; round < 3; ++round) {
            carry(receiver_radio, detector_radio, RECEIVER_MAC);
            carry(detector_radio, receiver_radio, DETECTOR_MAC);
            detector.update();
        }
        auto events = take_events(receiver);
        receiver_events.insert(receiver_events.end(), events.begin(), events.end());
        events = take_events(detector);
        detector_events.insert(detector_events.end(), events.begin(), events.end());
    }

    size_t count_sent(const FakeTransport& radio, MessageType type) const
    {
        size_t count = 0;
        for (const SentPacket& packet : radio.sent) {
            count += packet.message.type == type ? 1 : 0;
        }
        return count;
    }
};

static void assert_completed(const std::vector<LinkEvent>& events, uint32_t id, ItemState state)
{
    TEST_ASSERT_EQUAL(1, events.size());
    TEST_ASSERT_EQUAL(LinkEventType::ExchangeComplete, events[0].type);
    TEST_ASSERT_EQUAL_UINT32(id, events[0].exchange_id);
    TEST_ASSERT_EQUAL(state, events[0].item_state);
}

TEST_CASE("pair: Present, Absent and Unknown each complete on both sides", "[detection_link][pair]")
{
    const ItemState states[] = {ItemState::Present, ItemState::Absent, ItemState::Unknown};
    for (ItemState state : states) {
        Pair pair;
        pair.detector.set_local_item_state(state);
        pair.run_exchange();
        assert_completed(pair.receiver_events, 1, state);
        assert_completed(pair.detector_events, 1, state);
        // Exactly three messages: no extra listening or retransmission
        TEST_ASSERT_EQUAL(2, pair.receiver_radio.sent.size());
        TEST_ASSERT_EQUAL(1, pair.detector_radio.sent.size());
    }
}

TEST_CASE("pair: a lost request is retried and completes", "[detection_link][pair]")
{
    Pair pair;
    pair.detector.set_local_item_state(ItemState::Present);
    int requests = 0;
    pair.drop = [&](const Message& m) { return m.type == MessageType::StatusRequest && ++requests == 1; };
    pair.run_exchange();
    assert_completed(pair.receiver_events, 1, ItemState::Present);
    assert_completed(pair.detector_events, 1, ItemState::Present);
    TEST_ASSERT_EQUAL(2, pair.count_sent(pair.receiver_radio, MessageType::StatusRequest));
}

TEST_CASE("pair: a lost response is resent from the same snapshot", "[detection_link][pair]")
{
    Pair pair;
    pair.detector.set_local_item_state(ItemState::Present);
    int responses = 0;
    pair.drop = [&](const Message& m) {
        if (m.type != MessageType::StatusResponse) {
            return false;
        }
        // The sensor changes while the first response is lost
        pair.detector.set_local_item_state(ItemState::Absent);
        return ++responses == 1;
    };
    pair.run_exchange();
    assert_completed(pair.receiver_events, 1, ItemState::Present);
    assert_completed(pair.detector_events, 1, ItemState::Present);
}

TEST_CASE("pair: a lost acknowledgment is retried and both sides complete", "[detection_link][pair]")
{
    Pair pair;
    int acks = 0;
    pair.drop = [&](const Message& m) { return m.type == MessageType::StatusAck && ++acks == 1; };
    pair.run_exchange();
    assert_completed(pair.receiver_events, 1, ItemState::Unknown);
    assert_completed(pair.detector_events, 1, ItemState::Unknown);
    TEST_ASSERT_EQUAL(2, pair.count_sent(pair.receiver_radio, MessageType::StatusAck));
}

TEST_CASE("pair: detector absent gives a bounded response timeout", "[detection_link][pair]")
{
    Pair pair;
    pair.drop = [](const Message&) { return true; };
    const int64_t start = g_now_ms;
    pair.run_exchange();
    TEST_ASSERT_EQUAL(2, pair.receiver_events.size());
    TEST_ASSERT_EQUAL(LinkEventType::ExchangeFailed, pair.receiver_events[0].type);
    TEST_ASSERT_EQUAL(FailureReason::ResponseTimeout, pair.receiver_events[0].reason);
    TEST_ASSERT_LESS_OR_EQUAL(300, g_now_ms - start);
    TEST_ASSERT_EQUAL(3, pair.receiver_radio.sent.size());
    TEST_ASSERT_TRUE(pair.detector_events.empty());
}

TEST_CASE("pair: acknowledgments never getting through disagree as documented", "[detection_link][pair]")
{
    Pair pair;
    pair.drop = [](const Message& m) { return m.type == MessageType::StatusAck; };
    pair.run_exchange();
    TEST_ASSERT_EQUAL(FailureReason::AckTransmissionFailure, pair.receiver_events[0].reason);

    // The detector gives up at its acknowledgment deadline
    advance(400);
    pair.detector.update();
    auto events = take_events(pair.detector);
    TEST_ASSERT_EQUAL(1, events.size());
    TEST_ASSERT_EQUAL(FailureReason::AcknowledgmentMissing, events[0].reason);
}

TEST_CASE("pair: consecutive wake cycles use fresh IDs and snapshots", "[detection_link][pair]")
{
    Pair pair;
    const ItemState states[] = {ItemState::Absent, ItemState::Present, ItemState::Present, ItemState::Unknown};
    for (ItemState state : states) {
        pair.detector.set_local_item_state(state);
        pair.run_exchange();
        advance(1000);
    }
    TEST_ASSERT_EQUAL(4, pair.receiver_events.size());
    TEST_ASSERT_EQUAL(4, pair.detector_events.size());
    for (size_t i = 0; i < 4; ++i) {
        TEST_ASSERT_EQUAL_UINT32(i + 1, pair.receiver_events[i].exchange_id);
        TEST_ASSERT_EQUAL(states[i], pair.receiver_events[i].item_state);
        TEST_ASSERT_EQUAL_UINT32(i + 1, pair.detector_events[i].exchange_id);
    }
}
