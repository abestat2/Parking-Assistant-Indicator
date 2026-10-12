#include <cinttypes>
#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "detection_link/DetectionLink.hpp"
#include "detection_link/EspNowTransport.hpp"
#include "detection_link/ExchangeId.hpp"
#include "ParkingIndicator.hpp"
#include "RangeBench.hpp"

using namespace detection_link;

static const char* TAG = "INDICATOR";

// Data in of the Adafruit 2869 NeoPixel Stick (8 x SK6812 RGBW)
#define LED_STRIP_PIN GPIO_NUM_3

// Kept dim: each LED draws up to about 18 mA per channel at full brightness
static constexpr uint8_t LED_LEVEL = 32;
static constexpr LEDColor COLOR_PRESENT = {LED_LEVEL, 0, 0, 0};
static constexpr LEDColor COLOR_ABSENT = {0, LED_LEVEL, 0, 0};
static constexpr LEDColor COLOR_UNKNOWN = {LED_LEVEL, LED_LEVEL / 3, 0, 0};
static constexpr LEDColor COLOR_NO_CONTACT = {0, 0, LED_LEVEL, 0};
// One LED of the eight: the bar lights fully for any level below one LED's step
static constexpr float NO_CONTACT_LEVEL = 12.5f;

// Kept through deep sleep; cleared by a power-on or reset
RTC_DATA_ATTR static uint32_t s_wake_count;
RTC_DATA_ATTR static uint32_t s_failures_in_row;

// Parse "aa:bb:cc:dd:ee:ff". Returns false for an empty or malformed address.
static bool parse_mac(const char* text, MacAddress& mac)
{
    unsigned int bytes[6];
    char extra;
    if (std::sscanf(text, "%2x:%2x:%2x:%2x:%2x:%2x%c", &bytes[0], &bytes[1], &bytes[2], &bytes[3], &bytes[4],
                    &bytes[5], &extra) != 6) {
        return false;
    }
    for (size_t i = 0; i < mac.size(); ++i) {
        mac[i] = static_cast<uint8_t>(bytes[i]);
    }
    return true;
}

// ESP-NOW needs Wi-Fi started in station mode on the agreed channel, without connecting to an AP
static void start_wifi(uint8_t channel)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
    // Awake only for one exchange, so keep the receiver on rather than power-saving through it
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
}

struct ExchangeResult {
    bool complete;
    ItemState state;
    FailureReason reason;
    int64_t elapsed_ms;
};

// Run one request/response/acknowledge exchange with the detector. Bounded by the link's
// exchange deadline, so this always returns.
static ExchangeResult run_exchange(DetectionLink& link)
{
    const int64_t start_us = esp_timer_get_time();
    ExchangeResult result = {false, ItemState::Unknown, FailureReason::None, 0};
    if (link.request_status() != ESP_OK) {
        result.reason = FailureReason::TransportFailure;
        return result;
    }
    bool done = false;
    while (!done) {
        link.update();
        LinkEvent event;
        while (link.poll_event(event)) {
            if (event.type == LinkEventType::ExchangeComplete) {
                result.complete = true;
                result.state = event.item_state;
                done = true;
            } else if (event.type == LinkEventType::ExchangeFailed) {
                result.reason = event.reason;
                done = true;
            }
        }
        if (!done) {
            vTaskDelay(1);
        }
    }
    result.elapsed_ms = (esp_timer_get_time() - start_us) / 1000;
    return result;
}

static void show(ParkingIndicator& bar, const ExchangeResult& result)
{
    LEDColor color;
    float level = 100.0f;
    if (result.complete) {
        switch (result.state) {
        case ItemState::Present: color = COLOR_PRESENT; break;
        case ItemState::Absent: color = COLOR_ABSENT; break;
        default: color = COLOR_UNKNOWN; break;
        }
    } else if (s_failures_in_row >= CONFIG_INDICATOR_FAILURES_BEFORE_NO_CONTACT) {
        color = COLOR_NO_CONTACT;
        level = NO_CONTACT_LEVEL;
    } else {
        // A missed exchange or two keeps showing the last state until the retry
        return;
    }
    bar.set_indicator_color(&color);
    bar.write_indicator_level(level);
}

#if CONFIG_INDICATOR_DEEP_SLEEP
[[noreturn]] static void sleep_until_next_request(uint32_t delay_ms)
{
    // The LEDs keep their colour while powered; hold the data line low so they see no glitches
    gpio_set_direction(LED_STRIP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_STRIP_PIN, 0);
    gpio_hold_en(LED_STRIP_PIN);
    gpio_deep_sleep_hold_en();
    esp_wifi_stop();
    esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(delay_ms) * 1000);
    esp_deep_sleep_start();
}
#endif

extern "C" void app_main(void)
{
#if CONFIG_INDICATOR_RANGE_BENCH
    run_range_bench();
#endif
    // Released before the LED driver takes the line back
    gpio_hold_dis(LED_STRIP_PIN);
    const bool woke_from_sleep = esp_sleep_get_wakeup_causes() & BIT(ESP_SLEEP_WAKEUP_TIMER);
    ++s_wake_count;
    if (!woke_from_sleep) {
        s_failures_in_row = 0;
    }

    static ParkingIndicator bar(LED_STRIP_PIN);
    bar.enable_indicator();

    start_wifi(CONFIG_INDICATOR_ESPNOW_CHANNEL);
    if (!woke_from_sleep) {
        uint8_t own_mac[6];
        ESP_ERROR_CHECK(esp_read_mac(own_mac, ESP_MAC_WIFI_STA));
        ESP_LOGI(TAG, "Indicator MAC " MACSTR " on channel %d; set this as a stall's indicator on the detector",
                 MAC2STR(own_mac), CONFIG_INDICATOR_ESPNOW_CHANNEL);
    }

    LinkConfig config;
    config.transport.channel = CONFIG_INDICATOR_ESPNOW_CHANNEL;
    config.exchange_id_source = retained_exchange_id;
    MacAddress detector;
    if (!parse_mac(CONFIG_INDICATOR_DETECTOR_MAC, detector)) {
        ESP_LOGE(TAG, "No detector MAC address configured (\"%s\"); set it in menuconfig",
                 CONFIG_INDICATOR_DETECTOR_MAC);
        s_failures_in_row = CONFIG_INDICATOR_FAILURES_BEFORE_NO_CONTACT;
        show(bar, {false, ItemState::Unknown, FailureReason::InitializationFailure, 0});
        while (true) {
            vTaskDelay(portMAX_DELAY);
        }
    }
    config.transport.add_peer(detector);

    static EspNowTransport transport;
    static DetectionLink link(transport);

    while (true) {
        ExchangeResult result = {false, ItemState::Unknown, FailureReason::InitializationFailure, 0};
        if (link.started() || link.begin(Role::Receiver, config) == ESP_OK) {
            result = run_exchange(link);
        }

        if (result.complete) {
            s_failures_in_row = 0;
            ESP_LOGI(TAG, "Wake %" PRIu32 ": stall %s (%" PRId64 " ms)", s_wake_count, to_string(result.state),
                     result.elapsed_ms);
        } else {
            ++s_failures_in_row;
            ESP_LOGW(TAG, "Wake %" PRIu32 ": exchange failed: %s (%" PRId64 " ms, %" PRIu32 " in a row)",
                     s_wake_count, to_string(result.reason), result.elapsed_ms, s_failures_in_row);
        }
        show(bar, result);

        const uint32_t delay_ms =
            result.complete ? CONFIG_INDICATOR_REQUEST_INTERVAL_MS : CONFIG_INDICATOR_RETRY_INTERVAL_MS;
#if CONFIG_INDICATOR_DEEP_SLEEP
        link.end();
        sleep_until_next_request(delay_ms);
#else
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        ++s_wake_count;
#endif
    }
}
