// Tier 1: DetectionLink detector role against a scripted transport and a manual clock

#include "unity.h"
#include "fake_transport.hpp"

using namespace detection_link;
using namespace test_support;

static constexpr uint32_t RETRY_DELAY = 10;
static constexpr uint32_t ACK_DEADLINE = 500;
static constexpr uint32_t CONTACT_INTERVAL = 60000;
static constexpr uint32_t CONTACT_MARGIN = 5000;
static constexpr uint32_t INITIAL_GRACE = 100000;

static LinkConfig detector_config()
{
    LinkConfig config;
    config.transport.channel = 6;
    config.transport.peer = RECEIVER_MAC;
    config.retry_delay_ms = RETRY_DELAY;
    config.response_attempt_limit = 3;
    config.detector_ack_deadline_ms = ACK_DEADLINE;
    config.expected_contact_interval_ms = CONTACT_INTERVAL;
    config.contact_margin_ms = CONTACT_MARGIN;
    config.initial_contact_grace_ms = INITIAL_GRACE;
    return config;
}

struct Detector {
    FakeTransport transport;
    DetectionLink link{transport, fake_clock};

    explicit Detector(const LinkConfig& config = detector_config())
    {
        g_now_ms = 20000;
        link.set_local_item_state(ItemState::Present);
        TEST_ASSERT_EQUAL(ESP_OK, link.begin(Role::Detector, config));
    }

    // Deliver a request and return the response it produced
    const SentPacket& request_and_respond(uint32_t id)
    {
        const size_t before = transport.sent.size();
        transport.receive(RECEIVER_MAC, request(id));
        link.update();
        TEST_ASSERT_EQUAL(before + 1, transport.sent.size());
        return transport.last_sent();
    }

    // A full exchange: request, delivered response, acknowledgment
    void complete_exchange(uint32_t id)
    {
        request_and_respond(id);
        transport.complete(true);
        transport.receive(RECEIVER_MAC, ack(id));
        link.update();
    }
};

static void assert_response(const SentPacket& packet, uint32_t id, ItemState state)
{
    TEST_ASSERT_EQUAL_HEX8_ARRAY(RECEIVER_MAC.data(), packet.to.data(), 6);
    TEST_ASSERT_EQUAL(MessageType::StatusResponse, packet.message.type);
    TEST_ASSERT_EQUAL_UINT32(id, packet.message.exchange_id);
    TEST_ASSERT_EQUAL(state, packet.message.item_state);
}

static void assert_single_event(DetectionLink& link, LinkEventType type, uint32_t id)
{
    auto events = take_events(link);
    TEST_ASSERT_EQUAL(1, events.size());
    TEST_ASSERT_EQUAL(type, events[0].type);
    TEST_ASSERT_EQUAL_UINT32(id, events[0].exchange_id);
}

// ---- Responses ----

TEST_CASE("detector: begin does not need an exchange ID source", "[detection_link][detector]")
{
    Detector d;
    TEST_ASSERT_TRUE(d.link.started());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, d.link.request_status());

    FakeTransport transport;
    DetectionLink link(transport, fake_clock);
    LinkConfig config = detector_config();
    config.response_attempt_limit = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, link.begin(Role::Detector, config));
}

TEST_CASE("detector: a request is answered in the same update from the current state", "[detection_link][detector]")
{
    Detector d;
    d.link.set_local_item_state(ItemState::Absent);
    assert_response(d.request_and_respond(7), 7, ItemState::Absent);
    TEST_ASSERT_EQUAL(1, d.link.diagnostics().requests_received);
    // Nothing is complete until the acknowledgment arrives
    d.transport.complete(true);
    d.link.update();
    TEST_ASSERT_TRUE(take_events(d.link).empty());
}

TEST_CASE("detector: no valid sensor state is reported as Unknown", "[detection_link][detector]")
{
    Detector d;
    d.link.set_local_item_state(ItemState::Unknown);
    assert_response(d.request_and_respond(7), 7, ItemState::Unknown);
}

TEST_CASE("detector: acknowledgment completes the exchange with its snapshot", "[detection_link][detector]")
{
    Detector d;
    d.complete_exchange(7);
    auto events = take_events(d.link);
    TEST_ASSERT_EQUAL(1, events.size());
    TEST_ASSERT_EQUAL(LinkEventType::ExchangeComplete, events[0].type);
    TEST_ASSERT_EQUAL_UINT32(7, events[0].exchange_id);
    TEST_ASSERT_EQUAL(ItemState::Present, events[0].item_state);
    TEST_ASSERT_EQUAL(Health::Ok, d.link.health());
}

TEST_CASE("detector: duplicate requests get the same snapshot even if the sensor changed", "[detection_link][detector]")
{
    Detector d;
    d.request_and_respond(7);
    d.transport.complete(true);
    d.link.update();

    d.link.set_local_item_state(ItemState::Absent);
    assert_response(d.request_and_respond(7), 7, ItemState::Present);
    TEST_ASSERT_EQUAL(1, d.link.diagnostics().duplicate_requests);
    d.transport.complete(true);
    d.transport.receive(RECEIVER_MAC, ack(7));
    d.link.update();
    auto events = take_events(d.link);
    TEST_ASSERT_EQUAL(ItemState::Present, events[0].item_state);

    // A new exchange takes a fresh snapshot
    assert_response(d.request_and_respond(8), 8, ItemState::Absent);
}

TEST_CASE("detector: duplicate requests while a response is in flight coalesce into one resend", "[detection_link][detector]")
{
    Detector d;
    d.request_and_respond(7);
    for (int i = 0; i < 5; ++i) {
        d.transport.receive(RECEIVER_MAC, request(7));
    }
    d.link.update();
    TEST_ASSERT_EQUAL(1, d.transport.sent.size());

    d.transport.complete(true);
    d.link.update();
    TEST_ASSERT_EQUAL(2, d.transport.sent.size());
    assert_response(d.transport.last_sent(), 7, ItemState::Present);

    d.transport.complete(true);
    advance(RETRY_DELAY * 10);
    d.link.update();
    TEST_ASSERT_EQUAL(2, d.transport.sent.size());
}

// ---- Response retries ----

TEST_CASE("detector: failed response transmissions retry within the attempt limit, then stop", "[detection_link][detector]")
{
    Detector d;
    d.request_and_respond(7);
    for (int i = 0; i < 5; ++i) {
        if (d.transport.in_flight()) {
            d.transport.complete(false);
        }
        d.link.update();
        advance(RETRY_DELAY);
        d.link.update();
    }
    TEST_ASSERT_EQUAL(3, d.transport.sent.size());

    // No unsolicited responses while waiting; the acknowledgment deadline ends the exchange
    advance(ACK_DEADLINE);
    d.link.update();
    TEST_ASSERT_EQUAL(3, d.transport.sent.size());
    auto events = take_events(d.link);
    TEST_ASSERT_EQUAL(1, events.size());
    TEST_ASSERT_EQUAL(LinkEventType::ExchangeFailed, events[0].type);
    TEST_ASSERT_EQUAL_UINT32(7, events[0].exchange_id);
    TEST_ASSERT_EQUAL(FailureReason::AcknowledgmentMissing, events[0].reason);
}

TEST_CASE("detector: a duplicate request after exhausted retries is answered again", "[detection_link][detector]")
{
    LinkConfig config = detector_config();
    config.response_attempt_limit = 1;
    Detector d(config);
    d.request_and_respond(7);
    d.transport.complete(false);
    advance(RETRY_DELAY);
    d.link.update();
    TEST_ASSERT_EQUAL(1, d.transport.sent.size());

    assert_response(d.request_and_respond(7), 7, ItemState::Present);
}

TEST_CASE("detector: submission errors are retried after the retry delay", "[detection_link][detector]")
{
    Detector d;
    d.transport.submit_results = {SubmitResult::Failed};
    d.transport.receive(RECEIVER_MAC, request(7));
    d.link.update();
    TEST_ASSERT_EQUAL(0, d.transport.sent.size());
    d.link.update();
    TEST_ASSERT_EQUAL(0, d.transport.sent.size());
    advance(RETRY_DELAY);
    d.link.update();
    TEST_ASSERT_EQUAL(1, d.transport.sent.size());
    TEST_ASSERT_EQUAL(1, d.link.diagnostics().submit_failures);
}

TEST_CASE("detector: delivered response without an acknowledgment fails at the deadline", "[detection_link][detector]")
{
    Detector d;
    d.request_and_respond(7);
    d.transport.complete(true);
    advance(ACK_DEADLINE - 1);
    d.link.update();
    TEST_ASSERT_TRUE(take_events(d.link).empty());
    advance(1);
    d.link.update();
    auto events = take_events(d.link);
    TEST_ASSERT_EQUAL(1, events.size());
    TEST_ASSERT_EQUAL(FailureReason::AcknowledgmentMissing, events[0].reason);
    TEST_ASSERT_EQUAL(1, d.transport.sent.size());

    // A late acknowledgment for the abandoned exchange does not complete it
    d.transport.receive(RECEIVER_MAC, ack(7));
    d.link.update();
    TEST_ASSERT_TRUE(take_events(d.link).empty());
}

// ---- Acknowledgments ----

TEST_CASE("detector: acknowledgment before the response completion completes, and the late failure is ignored", "[detection_link][detector]")
{
    Detector d;
    d.request_and_respond(7);
    // The ACK arrives while the response's completion is still pending, then the radio reports failure
    d.transport.receive(RECEIVER_MAC, ack(7));
    d.transport.complete(false);
    d.link.update();
    assert_single_event(d.link, LinkEventType::ExchangeComplete, 7);

    advance(RETRY_DELAY * 10);
    d.link.update();
    TEST_ASSERT_EQUAL(1, d.transport.sent.size());
    TEST_ASSERT_TRUE(take_events(d.link).empty());
}

TEST_CASE("detector: duplicate acknowledgments are harmless", "[detection_link][detector]")
{
    Detector d;
    d.complete_exchange(7);
    take_events(d.link);
    d.transport.receive(RECEIVER_MAC, ack(7));
    d.transport.receive(RECEIVER_MAC, ack(7));
    d.link.update();
    TEST_ASSERT_TRUE(take_events(d.link).empty());
    TEST_ASSERT_EQUAL(2, d.link.diagnostics().duplicate_acks);
    // A late duplicate request for it gets no new response either
    d.transport.receive(RECEIVER_MAC, request(7));
    d.link.update();
    TEST_ASSERT_EQUAL(1, d.transport.sent.size());
}

TEST_CASE("detector: an acknowledgment for an unknown exchange cannot complete one", "[detection_link][detector]")
{
    // As after a detector restart during an exchange
    Detector d;
    d.transport.receive(RECEIVER_MAC, ack(7));
    d.link.update();
    TEST_ASSERT_TRUE(take_events(d.link).empty());
    TEST_ASSERT_EQUAL(1, d.link.diagnostics().unknown_acks);
    TEST_ASSERT_EQUAL(Health::Unknown, d.link.health());

    // A fresh request recovers
    d.complete_exchange(8);
    assert_single_event(d.link, LinkEventType::ExchangeComplete, 8);
}

TEST_CASE("detector: a new request abandons an unacknowledged exchange", "[detection_link][detector]")
{
    Detector d;
    d.request_and_respond(7);
    d.transport.complete(true);
    d.link.update();

    d.link.set_local_item_state(ItemState::Absent);
    assert_response(d.request_and_respond(8), 8, ItemState::Absent);
    assert_single_event(d.link, LinkEventType::ExchangeFailed, 7);

    d.transport.complete(true);
    d.transport.receive(RECEIVER_MAC, ack(8));
    d.link.update();
    assert_single_event(d.link, LinkEventType::ExchangeComplete, 8);
}

TEST_CASE("detector: exchanges recover after a failed one", "[detection_link][detector]")
{
    Detector d;
    d.request_and_respond(7);
    d.transport.complete(true);
    advance(ACK_DEADLINE);
    d.link.update();
    assert_single_event(d.link, LinkEventType::ExchangeFailed, 7);

    d.complete_exchange(8);
    assert_single_event(d.link, LinkEventType::ExchangeComplete, 8);
}

// ---- Filtering ----

TEST_CASE("detector: ignores other peers, malformed packets and responses", "[detection_link][detector]")
{
    Detector d;
    d.transport.receive(STRANGER_MAC, request(7));
    d.transport.receive(RECEIVER_MAC, response(7, ItemState::Present));
    d.transport.receive_raw(RECEIVER_MAC, {0x50, 0x41, 1, 1, 7, 0, 0, 0, 0});
    d.transport.receive_raw(RECEIVER_MAC, {});
    d.link.update();
    TEST_ASSERT_EQUAL(0, d.transport.sent.size());
    const LinkDiagnostics diagnostics = d.link.diagnostics();
    TEST_ASSERT_EQUAL(1, diagnostics.wrong_peer);
    TEST_ASSERT_EQUAL(1, diagnostics.stale_messages);
    TEST_ASSERT_EQUAL(2, diagnostics.malformed);
    TEST_ASSERT_EQUAL(0, diagnostics.requests_received);
}

// ---- Contact health ----

TEST_CASE("detector: no contact within the initial grace reports the receiver overdue once", "[detection_link][detector]")
{
    Detector d;
    advance(INITIAL_GRACE - 1);
    d.link.update();
    TEST_ASSERT_TRUE(take_events(d.link).empty());
    TEST_ASSERT_EQUAL(Health::Unknown, d.link.health());

    advance(1);
    d.link.update();
    auto events = take_events(d.link);
    TEST_ASSERT_EQUAL(1, events.size());
    TEST_ASSERT_EQUAL(LinkEventType::CommunicationLost, events[0].type);
    TEST_ASSERT_EQUAL(FailureReason::ReceiverOverdue, events[0].reason);
    TEST_ASSERT_EQUAL(Health::Lost, d.link.health());

    // Not repeated on every update
    for (int i = 0; i < 10; ++i) {
        advance(CONTACT_INTERVAL);
        d.link.update();
    }
    TEST_ASSERT_TRUE(take_events(d.link).empty());
}

TEST_CASE("detector: a completed exchange restores contact, even with an Unknown item state", "[detection_link][detector]")
{
    Detector d;
    d.link.set_local_item_state(ItemState::Unknown);
    advance(INITIAL_GRACE);
    d.link.update();
    take_events(d.link);

    d.complete_exchange(7);
    auto events = take_events(d.link);
    TEST_ASSERT_EQUAL(2, events.size());
    TEST_ASSERT_EQUAL(LinkEventType::ExchangeComplete, events[0].type);
    TEST_ASSERT_EQUAL(ItemState::Unknown, events[0].item_state);
    TEST_ASSERT_EQUAL(LinkEventType::CommunicationRestored, events[1].type);
    TEST_ASSERT_EQUAL(Health::Ok, d.link.health());
}

TEST_CASE("detector: contact deadline runs from the last completion, not from requests or duplicate ACKs", "[detection_link][detector]")
{
    Detector d;
    d.complete_exchange(7);
    take_events(d.link);

    // Requests without acknowledgments and repeated ACKs do not count as contact
    advance(CONTACT_INTERVAL);
    d.transport.receive(RECEIVER_MAC, ack(7));
    d.link.update();
    d.request_and_respond(8);
    d.transport.complete(true);
    d.link.update();

    advance(CONTACT_MARGIN - 1);
    d.link.update();
    TEST_ASSERT_EQUAL(Health::Ok, d.link.health());
    advance(1);
    d.link.update();
    TEST_ASSERT_EQUAL(Health::Lost, d.link.health());
    auto events = take_events(d.link);
    TEST_ASSERT_EQUAL(LinkEventType::CommunicationLost, events.back().type);
}

TEST_CASE("detector: end stops the transport and ignores traffic afterwards", "[detection_link][detector]")
{
    Detector d;
    d.request_and_respond(7);
    d.link.end();
    TEST_ASSERT_EQUAL(1, d.transport.end_calls);
    d.transport.receive(RECEIVER_MAC, ack(7));
    d.link.update();
    TEST_ASSERT_TRUE(take_events(d.link).empty());
}
