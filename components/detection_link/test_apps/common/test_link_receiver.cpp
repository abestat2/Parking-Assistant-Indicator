// Tier 1: DetectionLink receiver role against a scripted transport and a manual clock

#include "unity.h"
#include "fake_transport.hpp"

using namespace detection_link;
using namespace test_support;

static uint32_t s_next_id;
static uint32_t test_exchange_ids() { return s_next_id++; }

static constexpr uint32_t FIRST_ID = 100;
static constexpr uint32_t RESPONSE_TIMEOUT = 50;
static constexpr uint32_t RETRY_DELAY = 10;
static constexpr uint32_t EXCHANGE_DEADLINE = 1000;

static LinkConfig receiver_config()
{
    LinkConfig config;
    config.transport.channel = 6;
    config.transport.peer = DETECTOR_MAC;
    config.response_timeout_ms = RESPONSE_TIMEOUT;
    config.request_attempt_limit = 3;
    config.ack_attempt_limit = 3;
    config.retry_delay_ms = RETRY_DELAY;
    config.exchange_deadline_ms = EXCHANGE_DEADLINE;
    config.exchange_id_source = test_exchange_ids;
    return config;
}

struct Receiver {
    FakeTransport transport;
    DetectionLink link{transport, fake_clock};

    explicit Receiver(const LinkConfig& config = receiver_config())
    {
        g_now_ms = 5000;
        s_next_id = FIRST_ID;
        TEST_ASSERT_EQUAL(ESP_OK, link.begin(Role::Receiver, config));
    }

    // Start an exchange and let its first request transmit successfully
    void start_and_transmit_request()
    {
        TEST_ASSERT_EQUAL(ESP_OK, link.request_status());
        transport.complete(true);
        link.update();
    }

    // Deliver a response and let the resulting acknowledgment transmit successfully
    void respond_and_ack(ItemState state)
    {
        transport.receive(DETECTOR_MAC, response(FIRST_ID, state));
        link.update();
        transport.complete(true);
        link.update();
    }
};

static void assert_sent(const SentPacket& packet, MessageType type, uint32_t id)
{
    TEST_ASSERT_EQUAL_HEX8_ARRAY(DETECTOR_MAC.data(), packet.to.data(), 6);
    TEST_ASSERT_EQUAL(type, packet.message.type);
    TEST_ASSERT_EQUAL_UINT32(id, packet.message.exchange_id);
}

static void assert_failed(const std::vector<LinkEvent>& events, uint32_t id, FailureReason reason)
{
    // Exactly one ExchangeFailed, followed by the health transition of a first failure
    TEST_ASSERT_EQUAL(2, events.size());
    TEST_ASSERT_EQUAL(LinkEventType::ExchangeFailed, events[0].type);
    TEST_ASSERT_EQUAL_UINT32(id, events[0].exchange_id);
    TEST_ASSERT_EQUAL(reason, events[0].reason);
    TEST_ASSERT_EQUAL(LinkEventType::CommunicationLost, events[1].type);
    TEST_ASSERT_EQUAL(reason, events[1].reason);
}

// ---- begin() ----

TEST_CASE("receiver: begin starts the transport with the configured peer and channel", "[detection_link][receiver]")
{
    Receiver r;
    TEST_ASSERT_TRUE(r.transport.started);
    TEST_ASSERT_EQUAL_UINT8(6, r.transport.config.channel);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(DETECTOR_MAC.data(), r.transport.config.peer.data(), 6);
    TEST_ASSERT_TRUE(r.link.started());
    TEST_ASSERT_EQUAL(Health::Unknown, r.link.health());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, r.link.begin(Role::Receiver, receiver_config()));
    TEST_ASSERT_EQUAL(1, r.transport.begin_calls);
}

TEST_CASE("receiver: begin rejects a config without an ID source or attempts", "[detection_link][receiver]")
{
    FakeTransport transport;
    DetectionLink link(transport, fake_clock);

    LinkConfig config = receiver_config();
    config.exchange_id_source = nullptr;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, link.begin(Role::Receiver, config));
    config = receiver_config();
    config.request_attempt_limit = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, link.begin(Role::Receiver, config));
    config = receiver_config();
    config.ack_attempt_limit = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, link.begin(Role::Receiver, config));
    TEST_ASSERT_EQUAL(0, transport.begin_calls);
}

TEST_CASE("receiver: a transport initialization error is returned and leaves the link stopped", "[detection_link][receiver]")
{
    FakeTransport transport;
    transport.begin_result = ESP_FAIL;
    DetectionLink link(transport, fake_clock);
    TEST_ASSERT_EQUAL(ESP_FAIL, link.begin(Role::Receiver, receiver_config()));
    TEST_ASSERT_FALSE(link.started());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, link.request_status());

    transport.begin_result = ESP_OK;
    TEST_ASSERT_EQUAL(ESP_OK, link.begin(Role::Receiver, receiver_config()));
}

// ---- Normal exchange ----

TEST_CASE("receiver: request_status submits a request immediately", "[detection_link][receiver]")
{
    Receiver r;
    TEST_ASSERT_EQUAL(ESP_OK, r.link.request_status());
    TEST_ASSERT_EQUAL(1, r.transport.sent.size());
    assert_sent(r.transport.last_sent(), MessageType::StatusRequest, FIRST_ID);
    TEST_ASSERT_TRUE(r.link.exchange_in_progress());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, r.link.request_status());
}

TEST_CASE("receiver: normal exchange completes once the acknowledgment has transmitted", "[detection_link][receiver]")
{
    Receiver r;
    r.start_and_transmit_request();

    r.transport.receive(DETECTOR_MAC, response(FIRST_ID, ItemState::Present));
    r.link.update();
    TEST_ASSERT_EQUAL(2, r.transport.sent.size());
    assert_sent(r.transport.last_sent(), MessageType::StatusAck, FIRST_ID);

    // A successful submission alone is not success
    r.link.update();
    TEST_ASSERT_TRUE(take_events(r.link).empty());
    TEST_ASSERT_TRUE(r.link.exchange_in_progress());

    r.transport.complete(true);
    r.link.update();
    auto events = take_events(r.link);
    TEST_ASSERT_EQUAL(1, events.size());
    TEST_ASSERT_EQUAL(LinkEventType::ExchangeComplete, events[0].type);
    TEST_ASSERT_EQUAL_UINT32(FIRST_ID, events[0].exchange_id);
    TEST_ASSERT_EQUAL(ItemState::Present, events[0].item_state);
    TEST_ASSERT_FALSE(r.link.exchange_in_progress());
    TEST_ASSERT_EQUAL(Health::Ok, r.link.health());

    // No listening window, no further traffic, no repeated event
    advance(EXCHANGE_DEADLINE * 2);
    r.link.update();
    TEST_ASSERT_EQUAL(2, r.transport.sent.size());
    TEST_ASSERT_TRUE(take_events(r.link).empty());
}

TEST_CASE("receiver: Absent and Unknown responses are successful exchanges", "[detection_link][receiver]")
{
    const ItemState states[] = {ItemState::Absent, ItemState::Unknown};
    for (ItemState state : states) {
        Receiver r;
        r.start_and_transmit_request();
        r.respond_and_ack(state);
        auto events = take_events(r.link);
        TEST_ASSERT_EQUAL(1, events.size());
        TEST_ASSERT_EQUAL(LinkEventType::ExchangeComplete, events[0].type);
        TEST_ASSERT_EQUAL(state, events[0].item_state);
        TEST_ASSERT_EQUAL(Health::Ok, r.link.health());
    }
}

TEST_CASE("receiver: each exchange gets a fresh ID", "[detection_link][receiver]")
{
    Receiver r;
    r.start_and_transmit_request();
    r.respond_and_ack(ItemState::Present);
    take_events(r.link);

    TEST_ASSERT_EQUAL(ESP_OK, r.link.request_status());
    assert_sent(r.transport.last_sent(), MessageType::StatusRequest, FIRST_ID + 1);
}

// ---- Ordering ----

TEST_CASE("receiver: response arriving before the request completion is accepted", "[detection_link][receiver]")
{
    Receiver r;
    TEST_ASSERT_EQUAL(ESP_OK, r.link.request_status());

    // The request is still in flight when the response arrives
    r.transport.receive(DETECTOR_MAC, response(FIRST_ID, ItemState::Absent));
    r.link.update();
    // The acknowledgment waits for the transport to be free
    TEST_ASSERT_EQUAL(1, r.transport.sent.size());

    // The request is then reported as failed; the accepted response supersedes it
    r.transport.complete(false);
    r.link.update();
    TEST_ASSERT_EQUAL(2, r.transport.sent.size());
    assert_sent(r.transport.last_sent(), MessageType::StatusAck, FIRST_ID);

    r.transport.complete(true);
    r.link.update();
    auto events = take_events(r.link);
    TEST_ASSERT_EQUAL(1, events.size());
    TEST_ASSERT_EQUAL(LinkEventType::ExchangeComplete, events[0].type);
    TEST_ASSERT_EQUAL(ItemState::Absent, events[0].item_state);
}

// ---- Request retries ----

TEST_CASE("receiver: failed request transmission retries after the retry delay", "[detection_link][receiver]")
{
    Receiver r;
    TEST_ASSERT_EQUAL(ESP_OK, r.link.request_status());
    r.transport.complete(false);
    r.link.update();
    TEST_ASSERT_EQUAL(1, r.transport.sent.size());

    advance(RETRY_DELAY - 1);
    r.link.update();
    TEST_ASSERT_EQUAL(1, r.transport.sent.size());

    // Well before the response timeout
    advance(1);
    r.link.update();
    TEST_ASSERT_EQUAL(2, r.transport.sent.size());
    assert_sent(r.transport.last_sent(), MessageType::StatusRequest, FIRST_ID);
    TEST_ASSERT_EQUAL(1, r.link.diagnostics().transmit_failures);
}

TEST_CASE("receiver: missing response retries after the response timeout with the same ID", "[detection_link][receiver]")
{
    Receiver r;
    r.start_and_transmit_request();

    advance(RESPONSE_TIMEOUT - 1);
    r.link.update();
    TEST_ASSERT_EQUAL(1, r.transport.sent.size());

    advance(1);
    r.link.update();
    TEST_ASSERT_EQUAL(2, r.transport.sent.size());
    assert_sent(r.transport.last_sent(), MessageType::StatusRequest, FIRST_ID);

    r.transport.complete(true);
    r.link.update();
    r.respond_and_ack(ItemState::Present);
    auto events = take_events(r.link);
    TEST_ASSERT_EQUAL(1, events.size());
    TEST_ASSERT_EQUAL(LinkEventType::ExchangeComplete, events[0].type);
}

TEST_CASE("receiver: exhausted requests fail once with a response timeout", "[detection_link][receiver]")
{
    Receiver r;
    TEST_ASSERT_EQUAL(ESP_OK, r.link.request_status());
    for (int attempt = 0; attempt < 3; ++attempt) {
        TEST_ASSERT_EQUAL(attempt + 1, r.transport.sent.size());
        r.transport.complete(true);
        r.link.update();
        advance(RESPONSE_TIMEOUT);
        r.link.update();
        if (attempt == 0) {
            TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, r.link.request_status());
        }
    }
    TEST_ASSERT_EQUAL(3, r.transport.sent.size());
    assert_failed(take_events(r.link), FIRST_ID, FailureReason::ResponseTimeout);
    TEST_ASSERT_FALSE(r.link.exchange_in_progress());
    TEST_ASSERT_EQUAL(Health::Lost, r.link.health());

    // A late response for the failed exchange cannot turn it into a success
    r.transport.receive(DETECTOR_MAC, response(FIRST_ID, ItemState::Present));
    advance(1000);
    r.link.update();
    TEST_ASSERT_EQUAL(3, r.transport.sent.size());
    TEST_ASSERT_TRUE(take_events(r.link).empty());
}

TEST_CASE("receiver: requests that never transmit fail with a response timeout", "[detection_link][receiver]")
{
    Receiver r;
    TEST_ASSERT_EQUAL(ESP_OK, r.link.request_status());
    for (int attempt = 0; attempt < 3; ++attempt) {
        r.transport.complete(false);
        r.link.update();
        advance(RETRY_DELAY);
        r.link.update();
    }
    TEST_ASSERT_EQUAL(3, r.transport.sent.size());
    assert_failed(take_events(r.link), FIRST_ID, FailureReason::ResponseTimeout);
}

TEST_CASE("receiver: submission errors retry, and exhaust as a transport failure", "[detection_link][receiver]")
{
    Receiver r;
    r.transport.submit_results = {SubmitResult::Failed, SubmitResult::Failed, SubmitResult::Failed};
    TEST_ASSERT_EQUAL(ESP_OK, r.link.request_status());
    TEST_ASSERT_EQUAL(1, r.transport.submit_calls);

    r.link.update();
    TEST_ASSERT_EQUAL(1, r.transport.submit_calls);
    advance(RETRY_DELAY);
    r.link.update();
    TEST_ASSERT_EQUAL(2, r.transport.submit_calls);
    advance(RETRY_DELAY);
    r.link.update();
    TEST_ASSERT_EQUAL(3, r.transport.submit_calls);
    TEST_ASSERT_TRUE(take_events(r.link).empty());

    advance(RETRY_DELAY);
    r.link.update();
    TEST_ASSERT_EQUAL(3, r.transport.submit_calls);
    assert_failed(take_events(r.link), FIRST_ID, FailureReason::TransportFailure);
    TEST_ASSERT_EQUAL(3, r.link.diagnostics().submit_failures);
}

TEST_CASE("receiver: a submission error followed by success still completes", "[detection_link][receiver]")
{
    Receiver r;
    r.transport.submit_results = {SubmitResult::Failed};
    TEST_ASSERT_EQUAL(ESP_OK, r.link.request_status());
    advance(RETRY_DELAY);
    r.link.update();
    TEST_ASSERT_EQUAL(1, r.transport.sent.size());
    r.transport.complete(true);
    r.link.update();
    r.respond_and_ack(ItemState::Absent);
    auto events = take_events(r.link);
    TEST_ASSERT_EQUAL(1, events.size());
    TEST_ASSERT_EQUAL(LinkEventType::ExchangeComplete, events[0].type);
}

// ---- Acknowledgment ----

TEST_CASE("receiver: failed acknowledgment is retried and then completes", "[detection_link][receiver]")
{
    Receiver r;
    r.start_and_transmit_request();
    r.transport.receive(DETECTOR_MAC, response(FIRST_ID, ItemState::Present));
    r.link.update();
    r.transport.complete(false);
    r.link.update();
    TEST_ASSERT_TRUE(take_events(r.link).empty());

    advance(RETRY_DELAY);
    r.link.update();
    TEST_ASSERT_EQUAL(3, r.transport.sent.size());
    assert_sent(r.transport.last_sent(), MessageType::StatusAck, FIRST_ID);
    r.transport.complete(true);
    r.link.update();
    auto events = take_events(r.link);
    TEST_ASSERT_EQUAL(1, events.size());
    TEST_ASSERT_EQUAL(LinkEventType::ExchangeComplete, events[0].type);
}

TEST_CASE("receiver: exhausted acknowledgments fail as an ACK transmission failure", "[detection_link][receiver]")
{
    Receiver r;
    r.start_and_transmit_request();
    r.transport.receive(DETECTOR_MAC, response(FIRST_ID, ItemState::Present));
    r.link.update();
    for (int attempt = 0; attempt < 3; ++attempt) {
        r.transport.complete(false);
        r.link.update();
        advance(RETRY_DELAY);
        r.link.update();
    }
    TEST_ASSERT_EQUAL(4, r.transport.sent.size()); // 1 request + 3 acknowledgments
    assert_failed(take_events(r.link), FIRST_ID, FailureReason::AckTransmissionFailure);
}

TEST_CASE("receiver: duplicate responses do not add acknowledgments or events", "[detection_link][receiver]")
{
    Receiver r;
    r.start_and_transmit_request();
    r.transport.receive(DETECTOR_MAC, response(FIRST_ID, ItemState::Present));
    r.transport.receive(DETECTOR_MAC, response(FIRST_ID, ItemState::Present));
    r.link.update();
    r.transport.receive(DETECTOR_MAC, response(FIRST_ID, ItemState::Absent));
    r.transport.complete(true);
    r.link.update();
    r.transport.receive(DETECTOR_MAC, response(FIRST_ID, ItemState::Present));
    r.link.update();

    TEST_ASSERT_EQUAL(2, r.transport.sent.size());
    auto events = take_events(r.link);
    TEST_ASSERT_EQUAL(1, events.size());
    TEST_ASSERT_EQUAL(ItemState::Present, events[0].item_state);
    TEST_ASSERT_EQUAL(3, r.link.diagnostics().duplicate_responses);
}

// ---- Deadline ----

TEST_CASE("receiver: exchange deadline with a send in flight fails and ignores the late completion", "[detection_link][receiver]")
{
    Receiver r;
    r.start_and_transmit_request();
    r.transport.receive(DETECTOR_MAC, response(FIRST_ID, ItemState::Present));
    r.link.update();
    // The acknowledgment is in flight and its completion never comes in time
    advance(EXCHANGE_DEADLINE);
    r.link.update();
    assert_failed(take_events(r.link), FIRST_ID, FailureReason::AckTransmissionFailure);

    // The late success belongs to a terminated exchange
    r.transport.complete(true);
    r.link.update();
    TEST_ASSERT_TRUE(take_events(r.link).empty());
    TEST_ASSERT_EQUAL(Health::Lost, r.link.health());

    // Controlled shutdown stops the transport
    r.link.end();
    TEST_ASSERT_EQUAL(1, r.transport.end_calls);
    TEST_ASSERT_FALSE(r.transport.started);
}

TEST_CASE("receiver: exchange deadline while requesting is a response timeout", "[detection_link][receiver]")
{
    LinkConfig config = receiver_config();
    config.request_attempt_limit = 100;
    Receiver r(config);
    TEST_ASSERT_EQUAL(ESP_OK, r.link.request_status());
    while (take_events(r.link).empty()) {
        r.transport.complete(true);
        r.link.update();
        advance(RESPONSE_TIMEOUT);
        r.link.update();
        TEST_ASSERT_LESS_OR_EQUAL(EXCHANGE_DEADLINE / RESPONSE_TIMEOUT + 1, r.transport.sent.size());
    }
    TEST_ASSERT_FALSE(r.link.exchange_in_progress());
}

// ---- Filtering ----

TEST_CASE("receiver: ignores packets from other peers, other exchanges, other types and malformed", "[detection_link][receiver]")
{
    Receiver r;
    r.start_and_transmit_request();

    r.transport.receive(STRANGER_MAC, response(FIRST_ID, ItemState::Present));
    r.transport.receive(DETECTOR_MAC, response(FIRST_ID + 7, ItemState::Present));
    r.transport.receive(DETECTOR_MAC, request(FIRST_ID));
    r.transport.receive(DETECTOR_MAC, ack(FIRST_ID));
    r.transport.receive_raw(DETECTOR_MAC, {0x50, 0x41, 1, 2, 100, 0, 0});
    r.transport.receive_raw(DETECTOR_MAC, {0x50, 0x41, 9, 2, 100, 0, 0, 0, 2});
    r.link.update();

    TEST_ASSERT_EQUAL(1, r.transport.sent.size());
    TEST_ASSERT_TRUE(take_events(r.link).empty());
    const LinkDiagnostics diagnostics = r.link.diagnostics();
    TEST_ASSERT_EQUAL(1, diagnostics.wrong_peer);
    TEST_ASSERT_EQUAL(3, diagnostics.stale_messages);
    TEST_ASSERT_EQUAL(2, diagnostics.malformed);

    // They did not extend the response deadline either
    advance(RESPONSE_TIMEOUT);
    r.link.update();
    TEST_ASSERT_EQUAL(2, r.transport.sent.size());
}

TEST_CASE("receiver: a completion with a foreign token is ignored", "[detection_link][receiver]")
{
    Receiver r;
    TEST_ASSERT_EQUAL(ESP_OK, r.link.request_status());
    r.transport.inject_completion(r.transport.last_sent().token + 1000, true);
    r.link.update();
    // Still waiting on the real request, so no response timer has started
    advance(RESPONSE_TIMEOUT * 3);
    r.link.update();
    TEST_ASSERT_EQUAL(1, r.transport.sent.size());
}

// ---- Health ----

TEST_CASE("receiver: health changes are reported on transitions only", "[detection_link][receiver]")
{
    LinkConfig config = receiver_config();
    config.request_attempt_limit = 1;
    Receiver r(config);

    // Two failures: one CommunicationLost
    for (int i = 0; i < 2; ++i) {
        TEST_ASSERT_EQUAL(ESP_OK, r.link.request_status());
        r.transport.complete(false);
        r.link.update();
        advance(RETRY_DELAY);
        r.link.update();
    }
    auto events = take_events(r.link);
    TEST_ASSERT_EQUAL(3, events.size());
    TEST_ASSERT_EQUAL(LinkEventType::ExchangeFailed, events[0].type);
    TEST_ASSERT_EQUAL(LinkEventType::CommunicationLost, events[1].type);
    TEST_ASSERT_EQUAL(LinkEventType::ExchangeFailed, events[2].type);

    // Success restores it once
    TEST_ASSERT_EQUAL(ESP_OK, r.link.request_status());
    const uint32_t id = r.transport.last_sent().message.exchange_id;
    r.transport.complete(true);
    r.link.update();
    r.transport.receive(DETECTOR_MAC, response(id, ItemState::Unknown));
    r.link.update();
    r.transport.complete(true);
    r.link.update();
    events = take_events(r.link);
    TEST_ASSERT_EQUAL(2, events.size());
    TEST_ASSERT_EQUAL(LinkEventType::ExchangeComplete, events[0].type);
    TEST_ASSERT_EQUAL(LinkEventType::CommunicationRestored, events[1].type);
    TEST_ASSERT_EQUAL(Health::Ok, r.link.health());
}

// ---- Shutdown ----

TEST_CASE("receiver: end cancels the exchange without an event and stops the transport", "[detection_link][receiver]")
{
    Receiver r;
    TEST_ASSERT_EQUAL(ESP_OK, r.link.request_status());
    r.link.end();
    TEST_ASSERT_FALSE(r.link.started());
    TEST_ASSERT_FALSE(r.link.exchange_in_progress());
    TEST_ASSERT_EQUAL(1, r.transport.end_calls);
    r.link.update();
    TEST_ASSERT_TRUE(take_events(r.link).empty());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, r.link.request_status());

    // end() is idempotent, and the destructor does not stop the transport again
    r.link.end();
    TEST_ASSERT_EQUAL(1, r.transport.end_calls);
}

TEST_CASE("receiver: the destructor stops a running transport", "[detection_link][receiver]")
{
    FakeTransport transport;
    {
        DetectionLink link(transport, fake_clock);
        s_next_id = FIRST_ID;
        TEST_ASSERT_EQUAL(ESP_OK, link.begin(Role::Receiver, receiver_config()));
    }
    TEST_ASSERT_EQUAL(1, transport.end_calls);
}

TEST_CASE("receiver: detector-only calls are harmless", "[detection_link][receiver]")
{
    Receiver r;
    r.link.set_local_item_state(ItemState::Present);
    r.transport.receive(DETECTOR_MAC, request(5));
    r.link.update();
    TEST_ASSERT_EQUAL(0, r.transport.sent.size());
}
