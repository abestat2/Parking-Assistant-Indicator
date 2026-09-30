// Tier 3: on-target tests. [hw] tests use the real Wi-Fi radio and need an ESP32-C3 board,
// but no second device: they talk to a peer that does not exist.

#include "unity.h"
#include "detection_link/DetectionLink.hpp"
#include "detection_link/EspNowTransport.hpp"
#include "detection_link/ExchangeId.hpp"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace detection_link;

extern size_t g_test_leak_threshold;

static constexpr uint8_t TEST_CHANNEL = 1;
// Locally administered address that no real device uses
static constexpr MacAddress ABSENT_PEER = {0x02, 0x00, 0x00, 0x00, 0x00, 0x77};

// The application owns Wi-Fi; here it is started once and left running for all [hw] tests
static void ensure_wifi_started()
{
    static bool started = false;
    // The Wi-Fi driver keeps its buffers after the first start
    g_test_leak_threshold = 64 * 1024;
    if (started) {
        return;
    }
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        TEST_ESP_OK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    TEST_ESP_OK(err);
    err = esp_event_loop_create_default();
    TEST_ASSERT_TRUE(err == ESP_OK || err == ESP_ERR_INVALID_STATE);

    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    TEST_ESP_OK(esp_wifi_init(&config));
    TEST_ESP_OK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    TEST_ESP_OK(esp_wifi_set_mode(WIFI_MODE_STA));
    TEST_ESP_OK(esp_wifi_start());
    TEST_ESP_OK(esp_wifi_set_channel(TEST_CHANNEL, WIFI_SECOND_CHAN_NONE));
    started = true;
}

struct Recorder : TransportHandler {
    int completions = 0;
    uint32_t token = 0;
    bool delivered = true;

    void on_packet(const MacAddress& from, const uint8_t* data, size_t length) override {}
    void on_send_complete(uint32_t t, bool d) override
    {
        ++completions;
        token = t;
        delivered = d;
    }
};

TEST_CASE("hw: transport reports a failed send to an absent peer and stops cleanly", "[detection_link][hw]")
{
    ensure_wifi_started();
    EspNowTransport transport;
    TransportConfig config;
    config.channel = TEST_CHANNEL;
    config.peer = ABSENT_PEER;
    TEST_ESP_OK(transport.begin(config));

    uint8_t bytes[MAX_MESSAGE_SIZE];
    const size_t length = encode({MessageType::StatusRequest, 1, ItemState::Unknown}, bytes, sizeof(bytes));
    TEST_ASSERT_EQUAL(SubmitResult::Ok, transport.send(ABSENT_PEER, bytes, length, 123));

    Recorder recorder;
    for (int i = 0; i < 1000 && recorder.completions == 0; ++i) {
        vTaskDelay(pdMS_TO_TICKS(1));
        transport.update(recorder);
    }
    TEST_ASSERT_EQUAL(1, recorder.completions);
    TEST_ASSERT_EQUAL_UINT32(123, recorder.token);
    // Nobody acknowledged it at the MAC layer
    TEST_ASSERT_FALSE(recorder.delivered);
    TEST_ASSERT_EQUAL(0, transport.stats().completion_timeouts);

    transport.end();
    // ESP-NOW can be started again
    TEST_ESP_OK(transport.begin(config));
    transport.end();
}

TEST_CASE("hw: shutdown with a send in flight is safe", "[detection_link][hw]")
{
    ensure_wifi_started();
    for (int i = 0; i < 10; ++i) {
        EspNowTransport transport;
        TransportConfig config;
        config.channel = TEST_CHANNEL;
        config.peer = ABSENT_PEER;
        TEST_ESP_OK(transport.begin(config));
        uint8_t bytes[MAX_MESSAGE_SIZE];
        const size_t length = encode({MessageType::StatusRequest, 1, ItemState::Unknown}, bytes, sizeof(bytes));
        TEST_ASSERT_EQUAL(SubmitResult::Ok, transport.send(ABSENT_PEER, bytes, length, 1));
        // Destroyed while the radio is still retrying; its callback must not touch freed memory
    }
    vTaskDelay(pdMS_TO_TICKS(50));
}

TEST_CASE("hw: receiver link with no detector fails with a bounded response timeout", "[detection_link][hw]")
{
    ensure_wifi_started();
    EspNowTransport transport;
    DetectionLink link(transport);
    LinkConfig config;
    config.transport.channel = TEST_CHANNEL;
    config.transport.peer = ABSENT_PEER;
    config.exchange_id_source = retained_exchange_id;
    TEST_ESP_OK(link.begin(Role::Receiver, config));

    const int64_t start = steady_clock_ms();
    TEST_ESP_OK(link.request_status());
    while (link.exchange_in_progress() && steady_clock_ms() - start < 2000) {
        vTaskDelay(pdMS_TO_TICKS(1));
        link.update();
    }
    const int64_t elapsed = steady_clock_ms() - start;

    LinkEvent event;
    TEST_ASSERT_TRUE(link.poll_event(event));
    TEST_ASSERT_EQUAL(LinkEventType::ExchangeFailed, event.type);
    TEST_ASSERT_EQUAL(FailureReason::ResponseTimeout, event.reason);
    TEST_ASSERT_LESS_OR_EQUAL(config.exchange_deadline_ms + 20, elapsed);
    printf("Receiver gave up after %lld ms with %lu transmit failures\n", static_cast<long long>(elapsed),
           static_cast<unsigned long>(link.diagnostics().transmit_failures));
    link.end();
}
