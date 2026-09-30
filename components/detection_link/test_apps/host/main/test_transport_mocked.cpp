// Tier 2: EspNowTransport against the CMock mock of ESP-NOW

#include <cstring>
#include <type_traits>
#include <utility>
#include <vector>
#include "unity.h"
#include "detection_link/EspNowTransport.hpp"
#include "../../common/fake_transport.hpp"

extern "C" {
#include "Mockesp_now.h"
}

using namespace detection_link;
using namespace test_support;

static_assert(!std::is_copy_constructible_v<EspNowTransport>, "EspNowTransport must not be copyable");

// State captured by the stubs below
static esp_now_recv_cb_t s_recv_cb;
static esp_now_send_cb_t s_send_cb;
static esp_now_peer_info_t s_peer;
static std::vector<std::vector<uint8_t>> s_sent;
static MacAddress s_sent_to;
static esp_err_t s_send_result;

extern "C" void setUp(void)
{
    Mockesp_now_Init();
    s_recv_cb = nullptr;
    s_send_cb = nullptr;
    s_peer = {};
    s_sent.clear();
    s_send_result = ESP_OK;
    g_now_ms = 1000;
}

extern "C" void tearDown(void)
{
    Mockesp_now_Verify();
    Mockesp_now_Destroy();
}

static esp_err_t stub_register_recv(esp_now_recv_cb_t cb, int calls)
{
    s_recv_cb = cb;
    return ESP_OK;
}

static esp_err_t stub_register_send(esp_now_send_cb_t cb, int calls)
{
    s_send_cb = cb;
    return ESP_OK;
}

static esp_err_t stub_add_peer(const esp_now_peer_info_t* peer, int calls)
{
    s_peer = *peer;
    return ESP_OK;
}

static esp_err_t stub_send(const uint8_t* peer_addr, const uint8_t* data, size_t len, int calls)
{
    std::memcpy(s_sent_to.data(), peer_addr, 6);
    s_sent.emplace_back(data, data + len);
    return s_send_result;
}

static TransportConfig transport_config()
{
    TransportConfig config;
    config.channel = 11;
    config.peer = DETECTOR_MAC;
    config.send_completion_timeout_ms = 100;
    return config;
}

static void expect_begin_ok()
{
    esp_now_init_ExpectAndReturn(ESP_OK);
    esp_now_register_recv_cb_Stub(stub_register_recv);
    esp_now_register_send_cb_Stub(stub_register_send);
    esp_now_add_peer_Stub(stub_add_peer);
}

static void expect_end()
{
    esp_now_unregister_recv_cb_ExpectAndReturn(ESP_OK);
    esp_now_unregister_send_cb_ExpectAndReturn(ESP_OK);
    esp_now_deinit_ExpectAndReturn(ESP_OK);
}

// Simulate the Wi-Fi task delivering a packet
static void radio_receive(const MacAddress& from, const std::vector<uint8_t>& bytes)
{
    uint8_t src[6];
    std::memcpy(src, from.data(), 6);
    esp_now_recv_info_t info = {};
    info.src_addr = src;
    s_recv_cb(&info, bytes.data(), static_cast<int>(bytes.size()));
}

static void radio_send_result(bool success)
{
    esp_now_send_info_t info = {};
    s_send_cb(&info, success ? ESP_NOW_SEND_SUCCESS : ESP_NOW_SEND_FAIL);
}

struct Recorder : TransportHandler {
    std::vector<std::pair<MacAddress, std::vector<uint8_t>>> packets;
    std::vector<std::pair<uint32_t, bool>> completions;

    void on_packet(const MacAddress& from, const uint8_t* data, size_t length) override
    {
        packets.emplace_back(from, std::vector<uint8_t>(data, data + length));
    }
    void on_send_complete(uint32_t token, bool delivered) override
    {
        completions.emplace_back(token, delivered);
    }
};

static const uint8_t PAYLOAD[] = {1, 2, 3, 4};

// ---- begin() ----

TEST_CASE("transport: begin initializes ESP-NOW and adds the peer on the configured channel", "[detection_link][transport]")
{
    EspNowTransport transport(fake_clock);
    expect_begin_ok();
    TEST_ASSERT_EQUAL(ESP_OK, transport.begin(transport_config()));
    TEST_ASSERT_NOT_NULL(s_recv_cb);
    TEST_ASSERT_NOT_NULL(s_send_cb);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(DETECTOR_MAC.data(), s_peer.peer_addr, 6);
    TEST_ASSERT_EQUAL_UINT8(11, s_peer.channel);
    TEST_ASSERT_EQUAL(WIFI_IF_STA, s_peer.ifidx);
    TEST_ASSERT_FALSE(s_peer.encrypt);

    // Already started: nothing else is called
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, transport.begin(transport_config()));
    expect_end();
    transport.end();
}

TEST_CASE("transport: only one instance can be started", "[detection_link][transport]")
{
    EspNowTransport first(fake_clock);
    EspNowTransport second(fake_clock);
    expect_begin_ok();
    TEST_ASSERT_EQUAL(ESP_OK, first.begin(transport_config()));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, second.begin(transport_config()));
    expect_end();
    first.end();
    // The second is not started, so its destructor calls nothing
}

TEST_CASE("transport: begin errors deinitialize ESP-NOW and are returned", "[detection_link][transport]")
{
    EspNowTransport transport(fake_clock);

    esp_now_init_ExpectAndReturn(ESP_ERR_ESPNOW_NOT_INIT);
    TEST_ASSERT_EQUAL(ESP_ERR_ESPNOW_NOT_INIT, transport.begin(transport_config()));

    esp_now_init_ExpectAndReturn(ESP_OK);
    esp_now_register_recv_cb_Stub(stub_register_recv);
    esp_now_register_send_cb_Stub(stub_register_send);
    esp_now_add_peer_ExpectAnyArgsAndReturn(ESP_ERR_ESPNOW_FULL);
    esp_now_deinit_ExpectAndReturn(ESP_OK);
    TEST_ASSERT_EQUAL(ESP_ERR_ESPNOW_FULL, transport.begin(transport_config()));
    TEST_ASSERT_EQUAL(SubmitResult::NotStarted, transport.send(DETECTOR_MAC, PAYLOAD, sizeof(PAYLOAD), 1));

    // A later begin can still succeed
    expect_begin_ok();
    TEST_ASSERT_EQUAL(ESP_OK, transport.begin(transport_config()));
    expect_end();
}

// ---- send() ----

TEST_CASE("transport: send submits to the peer and reports the completion in update", "[detection_link][transport]")
{
    EspNowTransport transport(fake_clock);
    expect_begin_ok();
    TEST_ASSERT_EQUAL(ESP_OK, transport.begin(transport_config()));

    esp_now_send_Stub(stub_send);
    TEST_ASSERT_EQUAL(SubmitResult::Ok, transport.send(DETECTOR_MAC, PAYLOAD, sizeof(PAYLOAD), 42));
    TEST_ASSERT_EQUAL(1, s_sent.size());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(DETECTOR_MAC.data(), s_sent_to.data(), 6);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(PAYLOAD, s_sent[0].data(), sizeof(PAYLOAD));
    TEST_ASSERT_TRUE(transport.send_pending());

    // One transmission in flight at a time
    TEST_ASSERT_EQUAL(SubmitResult::Busy, transport.send(DETECTOR_MAC, PAYLOAD, sizeof(PAYLOAD), 43));

    // The completion is only dispatched from update(), and the slot stays reserved until then
    radio_send_result(true);
    TEST_ASSERT_TRUE(transport.send_pending());
    TEST_ASSERT_EQUAL(SubmitResult::Busy, transport.send(DETECTOR_MAC, PAYLOAD, sizeof(PAYLOAD), 43));

    Recorder recorder;
    transport.update(recorder);
    TEST_ASSERT_EQUAL(1, recorder.completions.size());
    TEST_ASSERT_EQUAL_UINT32(42, recorder.completions[0].first);
    TEST_ASSERT_TRUE(recorder.completions[0].second);
    TEST_ASSERT_FALSE(transport.send_pending());

    // A failed transmission is reported as not delivered
    TEST_ASSERT_EQUAL(SubmitResult::Ok, transport.send(DETECTOR_MAC, PAYLOAD, sizeof(PAYLOAD), 44));
    radio_send_result(false);
    transport.update(recorder);
    TEST_ASSERT_EQUAL(2, recorder.completions.size());
    TEST_ASSERT_EQUAL_UINT32(44, recorder.completions[1].first);
    TEST_ASSERT_FALSE(recorder.completions[1].second);

    expect_end();
}

TEST_CASE("transport: submission errors are immediate and leave the transport free", "[detection_link][transport]")
{
    EspNowTransport transport(fake_clock);
    expect_begin_ok();
    TEST_ASSERT_EQUAL(ESP_OK, transport.begin(transport_config()));

    esp_now_send_Stub(stub_send);
    s_send_result = ESP_ERR_ESPNOW_NO_MEM;
    TEST_ASSERT_EQUAL(SubmitResult::Failed, transport.send(DETECTOR_MAC, PAYLOAD, sizeof(PAYLOAD), 1));
    TEST_ASSERT_FALSE(transport.send_pending());
    TEST_ASSERT_EQUAL(1, transport.stats().submit_failures);

    Recorder recorder;
    transport.update(recorder);
    TEST_ASSERT_TRUE(recorder.completions.empty());

    s_send_result = ESP_OK;
    TEST_ASSERT_EQUAL(SubmitResult::Ok, transport.send(DETECTOR_MAC, PAYLOAD, sizeof(PAYLOAD), 2));
    expect_end();
}

TEST_CASE("transport: empty and oversized sends are rejected without touching the radio", "[detection_link][transport]")
{
    EspNowTransport transport(fake_clock);
    expect_begin_ok();
    TEST_ASSERT_EQUAL(ESP_OK, transport.begin(transport_config()));
    uint8_t big[MAX_PACKET_SIZE + 1] = {};
    TEST_ASSERT_EQUAL(SubmitResult::InvalidArgument, transport.send(DETECTOR_MAC, big, sizeof(big), 1));
    TEST_ASSERT_EQUAL(SubmitResult::InvalidArgument, transport.send(DETECTOR_MAC, big, 0, 1));
    TEST_ASSERT_EQUAL(SubmitResult::InvalidArgument, transport.send(DETECTOR_MAC, nullptr, 4, 1));
    expect_end();
}

TEST_CASE("transport: a completion that never arrives times out as a failure", "[detection_link][transport]")
{
    EspNowTransport transport(fake_clock);
    expect_begin_ok();
    TEST_ASSERT_EQUAL(ESP_OK, transport.begin(transport_config()));
    esp_now_send_Stub(stub_send);
    TEST_ASSERT_EQUAL(SubmitResult::Ok, transport.send(DETECTOR_MAC, PAYLOAD, sizeof(PAYLOAD), 7));

    Recorder recorder;
    advance(99);
    transport.update(recorder);
    TEST_ASSERT_TRUE(recorder.completions.empty());
    advance(1);
    transport.update(recorder);
    TEST_ASSERT_EQUAL(1, recorder.completions.size());
    TEST_ASSERT_EQUAL_UINT32(7, recorder.completions[0].first);
    TEST_ASSERT_FALSE(recorder.completions[0].second);
    TEST_ASSERT_EQUAL(1, transport.stats().completion_timeouts);

    // The next send is not credited with the abandoned send's late completion
    TEST_ASSERT_EQUAL(SubmitResult::Ok, transport.send(DETECTOR_MAC, PAYLOAD, sizeof(PAYLOAD), 8));
    radio_send_result(true);
    TEST_ASSERT_TRUE(transport.send_pending());
    radio_send_result(true);
    transport.update(recorder);
    TEST_ASSERT_EQUAL(2, recorder.completions.size());
    TEST_ASSERT_EQUAL_UINT32(8, recorder.completions[1].first);
    TEST_ASSERT_EQUAL(1, transport.stats().unexpected_completions);
    expect_end();
}

TEST_CASE("transport: a completion with nothing in flight is counted and ignored", "[detection_link][transport]")
{
    EspNowTransport transport(fake_clock);
    expect_begin_ok();
    TEST_ASSERT_EQUAL(ESP_OK, transport.begin(transport_config()));
    radio_send_result(true);
    Recorder recorder;
    transport.update(recorder);
    TEST_ASSERT_TRUE(recorder.completions.empty());
    TEST_ASSERT_EQUAL(1, transport.stats().unexpected_completions);
    expect_end();
}

// ---- Receive ----

TEST_CASE("transport: received packets are copied and dispatched in order from update", "[detection_link][transport]")
{
    EspNowTransport transport(fake_clock);
    expect_begin_ok();
    TEST_ASSERT_EQUAL(ESP_OK, transport.begin(transport_config()));

    std::vector<uint8_t> bytes = {9, 8, 7};
    radio_receive(DETECTOR_MAC, bytes);
    // The radio's buffer is only valid during the callback
    bytes[0] = 0;
    radio_receive(STRANGER_MAC, {5});

    Recorder recorder;
    transport.update(recorder);
    TEST_ASSERT_EQUAL(2, recorder.packets.size());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(DETECTOR_MAC.data(), recorder.packets[0].first.data(), 6);
    const uint8_t expected[] = {9, 8, 7};
    TEST_ASSERT_EQUAL(3, recorder.packets[0].second.size());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, recorder.packets[0].second.data(), 3);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(STRANGER_MAC.data(), recorder.packets[1].first.data(), 6);

    transport.update(recorder);
    TEST_ASSERT_EQUAL(2, recorder.packets.size());
    TEST_ASSERT_EQUAL(2, transport.stats().packets_queued);
    expect_end();
}

TEST_CASE("transport: oversized and empty received packets are rejected", "[detection_link][transport]")
{
    EspNowTransport transport(fake_clock);
    expect_begin_ok();
    TEST_ASSERT_EQUAL(ESP_OK, transport.begin(transport_config()));
    radio_receive(DETECTOR_MAC, std::vector<uint8_t>(MAX_PACKET_SIZE + 1, 0xAA));
    radio_receive(DETECTOR_MAC, {});
    radio_receive(DETECTOR_MAC, std::vector<uint8_t>(MAX_PACKET_SIZE, 0xAA));

    Recorder recorder;
    transport.update(recorder);
    TEST_ASSERT_EQUAL(1, recorder.packets.size());
    TEST_ASSERT_EQUAL(MAX_PACKET_SIZE, recorder.packets[0].second.size());
    TEST_ASSERT_EQUAL(2, transport.stats().packets_rejected);
    expect_end();
}

TEST_CASE("transport: a full receive queue drops packets but never the send completion", "[detection_link][transport]")
{
    EspNowTransport transport(fake_clock);
    expect_begin_ok();
    TEST_ASSERT_EQUAL(ESP_OK, transport.begin(transport_config()));
    esp_now_send_Stub(stub_send);
    TEST_ASSERT_EQUAL(SubmitResult::Ok, transport.send(DETECTOR_MAC, PAYLOAD, sizeof(PAYLOAD), 5));

    for (uint8_t i = 0; i < EspNowTransport::RX_QUEUE_DEPTH + 3; ++i) {
        radio_receive(DETECTOR_MAC, {i});
    }
    radio_send_result(true);

    Recorder recorder;
    transport.update(recorder);
    TEST_ASSERT_EQUAL(EspNowTransport::RX_QUEUE_DEPTH, recorder.packets.size());
    TEST_ASSERT_EQUAL_UINT8(0, recorder.packets.front().second[0]);
    TEST_ASSERT_EQUAL(1, recorder.completions.size());
    TEST_ASSERT_EQUAL_UINT32(5, recorder.completions[0].first);
    TEST_ASSERT_EQUAL(3, transport.stats().packets_dropped);
    expect_end();
}

// ---- Shutdown ----

TEST_CASE("transport: end stops ESP-NOW and later callbacks are ignored", "[detection_link][transport]")
{
    EspNowTransport transport(fake_clock);
    expect_begin_ok();
    TEST_ASSERT_EQUAL(ESP_OK, transport.begin(transport_config()));
    esp_now_send_Stub(stub_send);
    TEST_ASSERT_EQUAL(SubmitResult::Ok, transport.send(DETECTOR_MAC, PAYLOAD, sizeof(PAYLOAD), 5));
    radio_receive(DETECTOR_MAC, {1});

    expect_end();
    transport.end();
    TEST_ASSERT_FALSE(transport.send_pending());

    // Callbacks already on their way find no instance
    radio_receive(DETECTOR_MAC, {2});
    radio_send_result(true);
    Recorder recorder;
    transport.update(recorder);
    TEST_ASSERT_TRUE(recorder.packets.empty());
    TEST_ASSERT_TRUE(recorder.completions.empty());
    TEST_ASSERT_EQUAL(SubmitResult::NotStarted, transport.send(DETECTOR_MAC, PAYLOAD, sizeof(PAYLOAD), 6));

    // end() twice does nothing
    transport.end();
}

TEST_CASE("transport: callbacks after the object is destroyed are ignored", "[detection_link][transport]")
{
    {
        EspNowTransport transport(fake_clock);
        expect_begin_ok();
        TEST_ASSERT_EQUAL(ESP_OK, transport.begin(transport_config()));
        expect_end();
    }
    radio_receive(DETECTOR_MAC, {1});
    radio_send_result(false);

    // And a new instance can start
    EspNowTransport next(fake_clock);
    expect_begin_ok();
    TEST_ASSERT_EQUAL(ESP_OK, next.begin(transport_config()));
    Recorder recorder;
    next.update(recorder);
    TEST_ASSERT_TRUE(recorder.packets.empty());
    expect_end();
}

// ---- With DetectionLink ----

TEST_CASE("transport: a detector link answers a request through ESP-NOW", "[detection_link][transport]")
{
    EspNowTransport transport(fake_clock);
    DetectionLink link(transport, fake_clock);
    LinkConfig config;
    config.transport = transport_config();
    config.transport.peer = RECEIVER_MAC;
    expect_begin_ok();
    TEST_ASSERT_EQUAL(ESP_OK, link.begin(Role::Detector, config));
    link.set_local_item_state(ItemState::Present);

    uint8_t bytes[MAX_MESSAGE_SIZE];
    const size_t length = encode(request(77), bytes, sizeof(bytes));
    radio_receive(RECEIVER_MAC, std::vector<uint8_t>(bytes, bytes + length));
    esp_now_send_Stub(stub_send);
    link.update();

    TEST_ASSERT_EQUAL(1, s_sent.size());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(RECEIVER_MAC.data(), s_sent_to.data(), 6);
    Message sent;
    TEST_ASSERT_EQUAL(DecodeResult::Ok, decode(s_sent[0].data(), s_sent[0].size(), sent));
    TEST_ASSERT_EQUAL(MessageType::StatusResponse, sent.type);
    TEST_ASSERT_EQUAL_UINT32(77, sent.exchange_id);
    TEST_ASSERT_EQUAL(ItemState::Present, sent.item_state);

    radio_send_result(true);
    const size_t ack_length = encode(ack(77), bytes, sizeof(bytes));
    radio_receive(RECEIVER_MAC, std::vector<uint8_t>(bytes, bytes + ack_length));
    link.update();
    auto events = take_events(link);
    TEST_ASSERT_EQUAL(1, events.size());
    TEST_ASSERT_EQUAL(LinkEventType::ExchangeComplete, events[0].type);

    expect_end();
    link.end();
}
