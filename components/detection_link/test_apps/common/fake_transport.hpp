// Test doubles shared by the DetectionLink tests: a scriptable transport and a manual clock
#pragma once

#include "detection_link/DetectionLink.hpp"
#include "unity.h"
#include <deque>
#include <vector>

namespace test_support {

using namespace detection_link;

constexpr MacAddress DETECTOR_MAC = {0x02, 0x00, 0x00, 0x00, 0x00, 0xD1};
constexpr MacAddress RECEIVER_MAC = {0x02, 0x00, 0x00, 0x00, 0x00, 0xE1};
constexpr MacAddress STRANGER_MAC = {0x02, 0x00, 0x00, 0x00, 0x00, 0x99};

// Manual clock: tests move time with advance()
inline int64_t g_now_ms = 0;
inline int64_t fake_clock() { return g_now_ms; }
inline void advance(int64_t ms) { g_now_ms += ms; }

struct SentPacket {
    MacAddress to;
    std::vector<uint8_t> bytes;
    uint32_t token;
    Message message;
};

/**
 * @brief Transport whose traffic is driven by the test
 *
 * send() records the packet and keeps it in flight until the test calls complete(). Packets
 * queued with receive() and completions are dispatched on the next update(), packets first,
 * like EspNowTransport.
 */
class FakeTransport : public Transport {
public:
    esp_err_t begin_result = ESP_OK;
    // Consumed one per send() that reaches the radio; Ok when empty
    std::deque<SubmitResult> submit_results;

    bool started = false;
    int begin_calls = 0;
    int end_calls = 0;
    TransportConfig config = {};
    std::vector<SentPacket> sent;

    esp_err_t begin(const TransportConfig& cfg) override {
        ++begin_calls;
        config = cfg;
        started = begin_result == ESP_OK;
        return begin_result;
    }

    SubmitResult send(const MacAddress& peer, const uint8_t* data, size_t length, uint32_t token) override {
        if (!started) {
            return SubmitResult::NotStarted;
        }
        if (send_pending()) {
            return SubmitResult::Busy;
        }
        ++submit_calls;
        SubmitResult result = SubmitResult::Ok;
        if (!submit_results.empty()) {
            result = submit_results.front();
            submit_results.pop_front();
        }
        if (result == SubmitResult::Ok) {
            SentPacket packet = {peer, std::vector<uint8_t>(data, data + length), token, {}};
            TEST_ASSERT_EQUAL(DecodeResult::Ok, decode(data, length, packet.message));
            sent.push_back(packet);
            m_in_flight = true;
            m_in_flight_token = token;
        }
        return result;
    }

    bool send_pending() const override {
        return m_in_flight || !m_completions.empty();
    }

    void update(TransportHandler& handler) override {
        while (!m_rx.empty()) {
            Incoming packet = m_rx.front();
            m_rx.pop_front();
            handler.on_packet(packet.from, packet.bytes.data(), packet.bytes.size());
        }
        while (!m_completions.empty()) {
            Completion completion = m_completions.front();
            m_completions.pop_front();
            handler.on_send_complete(completion.token, completion.delivered);
        }
    }

    void end() override {
        ++end_calls;
        started = false;
        m_in_flight = false;
        m_rx.clear();
        m_completions.clear();
    }

    TransportStats stats() const override { return {}; }

    // ---- Test controls ----

    int submit_calls = 0;

    bool in_flight() const { return m_in_flight; }

    // Report the result of the send in flight; dispatched on the next update()
    void complete(bool delivered) {
        TEST_ASSERT_TRUE_MESSAGE(m_in_flight, "complete() with nothing in flight");
        m_in_flight = false;
        m_completions.push_back({m_in_flight_token, delivered});
    }

    // Deliver a completion for an arbitrary token, e.g. one from a cancelled exchange
    void inject_completion(uint32_t token, bool delivered) {
        m_completions.push_back({token, delivered});
    }

    void receive(const MacAddress& from, const Message& message) {
        uint8_t bytes[MAX_MESSAGE_SIZE];
        const size_t length = encode(message, bytes, sizeof(bytes));
        TEST_ASSERT_NOT_EQUAL(0, length);
        receive_raw(from, std::vector<uint8_t>(bytes, bytes + length));
    }

    void receive_raw(const MacAddress& from, const std::vector<uint8_t>& bytes) {
        m_rx.push_back({from, bytes});
    }

    const SentPacket& last_sent() const {
        TEST_ASSERT_FALSE_MESSAGE(sent.empty(), "nothing was sent");
        return sent.back();
    }

private:
    struct Incoming {
        MacAddress from;
        std::vector<uint8_t> bytes;
    };
    struct Completion {
        uint32_t token;
        bool delivered;
    };

    bool m_in_flight = false;
    uint32_t m_in_flight_token = 0;
    std::deque<Incoming> m_rx;
    std::deque<Completion> m_completions;
};

inline Message request(uint32_t id) { return {MessageType::StatusRequest, id, ItemState::Unknown}; }
inline Message response(uint32_t id, ItemState state) { return {MessageType::StatusResponse, id, state}; }
inline Message ack(uint32_t id) { return {MessageType::StatusAck, id, ItemState::Unknown}; }

// Drain all queued events
inline std::vector<LinkEvent> take_events(DetectionLink& link) {
    std::vector<LinkEvent> events;
    LinkEvent event;
    while (link.poll_event(event)) {
        events.push_back(event);
    }
    return events;
}

} // namespace test_support
