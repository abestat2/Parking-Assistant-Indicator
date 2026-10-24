#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <optional>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_attr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "detection_link/DetectionLink.hpp"
#include "detection_link/EspNowTransport.hpp"
#include "detection_link/ExchangeId.hpp"
#include "DistanceLimits.hpp"
#include "GuidanceBar.hpp"
#include "I2cBus.hpp"
#include "ParkingIndicator.hpp"
#include "RangeBench.hpp"
#include "VL53L1X.hpp"

using namespace detection_link;

static const char* TAG = "INDICATOR";

// Qwiic connector on the SparkFun Pro Micro ESP32-C3
#define I2C_SDA_PIN GPIO_NUM_5
#define I2C_SCL_PIN GPIO_NUM_6
#define RANGE_DETECTOR_XSHUT_PIN GPIO_NUM_4
// Data in of the Adafruit 2869 NeoPixel Stick (8 x SK6812 RGBW)
#define LED_STRIP_PIN GPIO_NUM_3
// Gate of the N-channel FET in the LED bar's ground: high powers the bar
#define LED_POWER_PIN static_cast<gpio_num_t>(CONFIG_INDICATOR_LED_POWER_GPIO)
// Battery through a resistor divider
#define BATTERY_ADC_PIN CONFIG_INDICATOR_BATTERY_ADC_GPIO

static constexpr uint8_t LED_COUNT = 8;
static constexpr int LIDAR_DISCARDED_READINGS = 2;
static constexpr uint8_t LED_LEVEL = CONFIG_INDICATOR_LED_BRIGHTNESS;
static constexpr LEDColor COLOR_APPROACH = {0, LED_LEVEL, 0, 0};
static constexpr LEDColor COLOR_STOP = {LED_LEVEL, 0, 0, 0};

static_assert(CONFIG_INDICATOR_STOP_DISTANCE_MM < CONFIG_INDICATOR_GUIDE_DISTANCE_MM,
              "The stop distance must be closer than the distance where guidance starts");

// Battery voltage measured this wake, sent with every request; BATTERY_UNKNOWN if not measured
static uint16_t s_battery_mv = BATTERY_UNKNOWN;

// What the lidar last measured, sent to the detector with each request. The lidar is only on while
// guiding a car, so it is DISTANCE_NONE ("nan" on the web page) the rest of the time.
static uint16_t s_distance_mm = DISTANCE_NONE;
// Stop and guide distances of the LED bar: saved in flash, set by the detector, loaded at start-up
static RangeLimits s_limits = {CONFIG_INDICATOR_STOP_DISTANCE_MM, CONFIG_INDICATOR_GUIDE_DISTANCE_MM};

// Kept through deep sleep; cleared by a power-on or reset
RTC_DATA_ATTR static uint32_t s_wake_count;
RTC_DATA_ATTR static uint32_t s_failures_in_row;
// The car now in the stall has been guided in, so it is not guided again until the stall empties
RTC_DATA_ATTR static bool s_arrival_guided;

// Off while a PC is connected to the USB port, so the indicator can be flashed and logged
#if CONFIG_INDICATOR_DEEP_SLEEP
static bool s_deep_sleep = true;
#else
static bool s_deep_sleep = false;
#endif

// How long a cold boot waits for a USB host before deciding to deep sleep
static constexpr int64_t USB_HOST_WAIT_MS = 2000;

// True if a PC (not a charger or power bank) enumerates the USB port within USB_HOST_WAIT_MS
static bool usb_host_connected()
{
    const int64_t start_ms = esp_timer_get_time() / 1000;
    while (esp_timer_get_time() / 1000 - start_ms < USB_HOST_WAIT_MS) {
        if (usb_serial_jtag_is_connected()) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

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

/**
 * @brief Measure the battery through the divider on the battery ADC pin
 * @return uint16_t Battery voltage in millivolts, or BATTERY_UNKNOWN if the ADC is not available
 */
static uint16_t read_battery_mv()
{
    adc_unit_t unit;
    adc_channel_t channel;
    if (adc_oneshot_io_to_channel(BATTERY_ADC_PIN, &unit, &channel) != ESP_OK) {
        return BATTERY_UNKNOWN;
    }
    adc_oneshot_unit_handle_t adc = nullptr;
    adc_oneshot_unit_init_cfg_t unit_config = {};
    unit_config.unit_id = unit;
    if (adc_oneshot_new_unit(&unit_config, &adc) != ESP_OK) {
        return BATTERY_UNKNOWN;
    }
    // 12 dB covers about 0-2.5 V at the pin, a full cell (4.2 V) behind a 1:2 divider
    adc_oneshot_chan_cfg_t channel_config = {};
    channel_config.atten = ADC_ATTEN_DB_12;
    channel_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    adc_cali_handle_t cali = nullptr;
    adc_cali_curve_fitting_config_t cali_config = {};
    cali_config.unit_id = unit;
    cali_config.chan = channel;
    cali_config.atten = ADC_ATTEN_DB_12;
    cali_config.bitwidth = ADC_BITWIDTH_DEFAULT;

    uint16_t battery_mv = BATTERY_UNKNOWN;
    if (adc_oneshot_config_channel(adc, channel, &channel_config) == ESP_OK &&
        adc_cali_create_scheme_curve_fitting(&cali_config, &cali) == ESP_OK) {
        // Average a few samples; the capacitor on the pin keeps the divider's high impedance from loading them
        constexpr int SAMPLES = 8;
        int total_mv = 0;
        int samples = 0;
        for (int i = 0; i < SAMPLES; ++i) {
            int raw;
            int mv;
            if (adc_oneshot_read(adc, channel, &raw) == ESP_OK &&
                adc_cali_raw_to_voltage(cali, raw, &mv) == ESP_OK) {
                total_mv += mv;
                ++samples;
            }
        }
        if (samples > 0) {
            const int pin_mv = total_mv / samples;
            battery_mv = static_cast<uint16_t>(pin_mv * (CONFIG_INDICATOR_BATTERY_DIVIDER_TOP_KOHM +
                                                         CONFIG_INDICATOR_BATTERY_DIVIDER_BOTTOM_KOHM) /
                                               CONFIG_INDICATOR_BATTERY_DIVIDER_BOTTOM_KOHM);
        }
        adc_cali_delete_scheme_curve_fitting(cali);
    }
    adc_oneshot_del_unit(adc);
    return battery_mv;
}

// The LED data pin while the bar is unpowered: not driven and not pulled, so the LEDs, whose ground
// is switched off, cannot find a ground path through it
static void float_led_data_pin()
{
    gpio_reset_pin(LED_STRIP_PIN);
    gpio_set_direction(LED_STRIP_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LED_STRIP_PIN, GPIO_FLOATING);
}

static void set_led_power(bool on)
{
    gpio_set_direction(LED_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_POWER_PIN, on ? 1 : 0);
    if (on) {
        // Let the LEDs power up before the first frame
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

struct ExchangeResult {
    bool complete;
    ItemState state;
    FailureReason reason;
    int64_t elapsed_ms;
};

// Use the limits the detector asked for, and keep them through power loss. Ignored when they are not
// set (no change asked for), the same as now, or not usable.
static void apply_limits(const RangeLimits& requested)
{
    if (!requested.set() || requested == s_limits) {
        return;
    }
    const esp_err_t err = distance_limits::save(requested);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Ignoring distance limits %u to %u mm: %s", static_cast<unsigned>(requested.min_mm),
                 static_cast<unsigned>(requested.max_mm),
                 err == ESP_ERR_INVALID_ARG ? "out of range" : esp_err_to_name(err));
        return;
    }
    s_limits = requested;
    ESP_LOGI(TAG, "Distance limits set by the detector: stop %u mm, full bar %u mm",
             static_cast<unsigned>(s_limits.min_mm), static_cast<unsigned>(s_limits.max_mm));
}

// Run one request/response/acknowledge exchange with the detector. Bounded by the link's
// exchange deadline, so this always returns.
static ExchangeResult run_exchange(DetectionLink& link)
{
    const int64_t start_us = esp_timer_get_time();
    ExchangeResult result = {false, ItemState::Unknown, FailureReason::None, 0};
    if (!link.started() || link.request_status(s_battery_mv, s_limits, s_distance_mm) != ESP_OK) {
        result.reason = link.started() ? FailureReason::TransportFailure : FailureReason::InitializationFailure;
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
                apply_limits(event.limits);
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

static void log_result(const ExchangeResult& result)
{
    if (result.complete) {
        s_failures_in_row = 0;
        ESP_LOGI(TAG, "Wake %" PRIu32 ": stall %s (%" PRId64 " ms, battery %u mV)", s_wake_count,
                 to_string(result.state), result.elapsed_ms, static_cast<unsigned>(s_battery_mv));
    } else {
        ++s_failures_in_row;
        ESP_LOGW(TAG, "Wake %" PRIu32 ": exchange failed: %s (%" PRId64 " ms, %" PRIu32 " in a row)", s_wake_count,
                 to_string(result.reason), result.elapsed_ms, s_failures_in_row);
    }
}

// Show one lidar reading. Readings that are not a distance leave the bar as it is, except
// "out of range", which means nothing is within the sensor's reach yet.
static void show_distance(ParkingIndicator& bar, int32_t range_mm)
{
    if (range_mm == vl53l1x::RANGE_OUT_OF_RANGE) {
        // Nothing within reach: no car to report a distance for
        s_distance_mm = DISTANCE_NONE;
        range_mm = s_limits.max_mm;
    } else if (range_mm < 0) {
        return;
    } else {
        s_distance_mm = static_cast<uint16_t>(std::min<int32_t>(range_mm, DISTANCE_NONE - 1));
    }
    const GuidanceBar setting = guidance_bar(range_mm, s_limits.min_mm, s_limits.max_mm, LED_COUNT);
    LEDColor color = setting.too_close ? COLOR_STOP : COLOR_APPROACH;
    bar.set_indicator_color(&color);
    bar.write_indicator_level(setting.lit * 100.0f / LED_COUNT);
}

/**
 * @brief A car has arrived: show its distance on the bar for the guidance time
 *
 * Asks the detector again every request interval and stops early once the stall is empty, so a car
 * that backs out does not leave the bar lit.
 *
 * @return ItemState Absent if the stall emptied during guidance, otherwise Present
 */
static ItemState guide_car(DetectionLink& link)
{
    // The bar is powered and driven only while guiding
    set_led_power(true);
    std::optional<ParkingIndicator> led_bar;
    led_bar.emplace(LED_STRIP_PIN);
    ParkingIndicator& bar = *led_bar;
    bar.enable_indicator();
    bar.clear_indicator();

    // The board and the Adafruit breakout both have I2C pull-ups, so the internal ones stay off
    static I2cBus i2c_bus(I2C_NUM_0, I2C_SDA_PIN, I2C_SCL_PIN);
    static I2cMasterDevice range_device(i2c_bus, vl53l1x::DEFAULT_ADDRESS);
    static VL53L1X lidar(range_device, RANGE_DETECTOR_XSHUT_PIN);
    static bool bus_ready = false;
    if (!bus_ready) {
        bus_ready = i2c_bus.init() == ESP_OK && range_device.init() == ESP_OK;
    }

    vl53l1x::Config config;
    config.distance_mode = vl53l1x::DistanceMode::Long;
    config.timing_budget_ms = 50;
    config.inter_measurement_ms = 100;
    esp_err_t err = !bus_ready ? ESP_FAIL : (lidar.is_initialized() ? lidar.wake() : lidar.init(config));
    if (err == ESP_OK) {
        err = lidar.start_ranging();
    }
    if (err == ESP_OK) {
        // The first readings after starting are not reliable (31 and 92 mm seen with nothing that
        // close) and would flash the bar red, so they are not shown
        for (int i = 0; i < LIDAR_DISCARDED_READINGS; ++i) {
            lidar.read_range_mm(200);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Lidar not available (%s); no distance to show", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "Car arriving: guiding for %d s", CONFIG_INDICATOR_GUIDANCE_TIME_MS / 1000);
    const int64_t start_ms = esp_timer_get_time() / 1000;
    int64_t next_check_ms = start_ms + CONFIG_INDICATOR_REQUEST_INTERVAL_MS;
    ItemState state = ItemState::Present;
    int32_t last_logged_mm = -1000;
    while (esp_timer_get_time() / 1000 - start_ms < CONFIG_INDICATOR_GUIDANCE_TIME_MS) {
        if (lidar.is_ranging()) {
            const int32_t range_mm = lidar.read_range_mm(200);
            show_distance(bar, range_mm);
            // Log only noticeable changes, so the log follows the car without flooding
            if (range_mm < 0 || range_mm - last_logged_mm > 50 || last_logged_mm - range_mm > 50) {
                ESP_LOGI(TAG, "Distance %" PRId32 " mm", range_mm);
                last_logged_mm = range_mm;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (esp_timer_get_time() / 1000 >= next_check_ms) {
            next_check_ms += CONFIG_INDICATOR_REQUEST_INTERVAL_MS;
            ++s_wake_count;
            const ExchangeResult result = run_exchange(link);
            log_result(result);
            if (result.complete && result.state == ItemState::Absent) {
                ESP_LOGI(TAG, "Stall empty again; stopping guidance");
                state = ItemState::Absent;
                break;
            }
        }
    }

    s_distance_mm = DISTANCE_NONE;  // the lidar goes off, so there is nothing to report
    bar.clear_indicator();
    led_bar.reset();
    float_led_data_pin();
    set_led_power(false);
    if (lidar.is_initialized()) {
        if (s_deep_sleep) {
            lidar.prepare_for_deep_sleep();
        } else {
            lidar.stop_ranging();
        }
    }
    ESP_LOGI(TAG, "Guidance done");
    return state;
}

// One request to the detector, and guidance if a new car has arrived
static void serve(DetectionLink& link)
{
    const ExchangeResult result = run_exchange(link);
    log_result(result);
    if (!result.complete) {
        return;
    }
    if (result.state == ItemState::Absent) {
        s_arrival_guided = false;
    } else if (result.state == ItemState::Present && !s_arrival_guided) {
        s_arrival_guided = guide_car(link) == ItemState::Present;
    }
    // Unknown (the radar is not ready) changes nothing: the bar stays off and a car being guided stays guided
}

[[noreturn]] static void sleep_until_next_request(uint32_t delay_ms)
{
    // Keep the bar unpowered through sleep; its data pin stays floating
    set_led_power(false);
    gpio_hold_en(LED_POWER_PIN);
    gpio_deep_sleep_hold_en();
    esp_wifi_stop();
    esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(delay_ms) * 1000);
    esp_deep_sleep_start();
}

extern "C" void app_main(void)
{
#if CONFIG_INDICATOR_RANGE_BENCH
    run_range_bench();
#endif
    // Bar off and its data pin floating until a car needs guiding
    gpio_hold_dis(LED_POWER_PIN);
    gpio_hold_dis(LED_STRIP_PIN);
    set_led_power(false);
    float_led_data_pin();
    // Measured before Wi-Fi starts, so the radio's current does not pull the reading down
    s_battery_mv = read_battery_mv();
    const bool woke_from_sleep = esp_sleep_get_wakeup_causes() & BIT(ESP_SLEEP_WAKEUP_TIMER);
    ++s_wake_count;
    if (!woke_from_sleep) {
        s_failures_in_row = 0;
        s_arrival_guided = false;
    }

    start_wifi(CONFIG_INDICATOR_ESPNOW_CHANNEL);  // also initialises NVS
    s_limits = distance_limits::load();
    if (!woke_from_sleep) {
        uint8_t own_mac[6];
        ESP_ERROR_CHECK(esp_read_mac(own_mac, ESP_MAC_WIFI_STA));
        ESP_LOGI(TAG, "Indicator MAC " MACSTR " on channel %d; set this as a stall's indicator on the detector",
                 MAC2STR(own_mac), CONFIG_INDICATOR_ESPNOW_CHANNEL);
        // A brownout, watchdog or panic here means the previous run ended badly
        ESP_LOGI(TAG, "Started by reset reason %d", static_cast<int>(esp_reset_reason()));
        ESP_LOGI(TAG, "Distance limits: stop %u mm, full bar %u mm", static_cast<unsigned>(s_limits.min_mm),
                 static_cast<unsigned>(s_limits.max_mm));
        if (s_deep_sleep && usb_host_connected()) {
            s_deep_sleep = false;
            ESP_LOGI(TAG, "USB host connected: staying awake between requests until the next power-up");
        }
    }

    LinkConfig config;
    config.transport.channel = CONFIG_INDICATOR_ESPNOW_CHANNEL;
    config.exchange_id_source = retained_exchange_id;
    MacAddress detector;
    if (!parse_mac(CONFIG_INDICATOR_DETECTOR_MAC, detector)) {
        ESP_LOGE(TAG, "No detector MAC address configured (\"%s\"); set it in menuconfig",
                 CONFIG_INDICATOR_DETECTOR_MAC);
        while (true) {
            vTaskDelay(portMAX_DELAY);
        }
    }
    config.transport.add_peer(detector);

    static EspNowTransport transport;
    static DetectionLink link(transport);

    while (true) {
        if (!link.started()) {
            link.begin(Role::Receiver, config);
        }
        serve(link);
        if (s_deep_sleep) {
            link.end();
            sleep_until_next_request(CONFIG_INDICATOR_REQUEST_INTERVAL_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(CONFIG_INDICATOR_REQUEST_INTERVAL_MS));
        ++s_wake_count;
        s_battery_mv = read_battery_mv();
    }
}
