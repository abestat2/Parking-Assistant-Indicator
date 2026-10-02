#include <cinttypes>
#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_log.h"
#include "I2cBus.hpp"
#include "VL53L1X.hpp"
#include "MB1013.hpp"

// Qwiic connector on the SparkFun Pro Micro ESP32-C3
#define I2C_SDA_PIN GPIO_NUM_5
#define I2C_SCL_PIN GPIO_NUM_6
#define RANGE_DETECTOR_XSHUT_PIN GPIO_NUM_4
// MB1013 PW output
#define SONAR_PW_PIN GPIO_NUM_7

// How often to try again when the range sensor is missing or not responding
static constexpr TickType_t SENSOR_RETRY_DELAY = pdMS_TO_TICKS(5000);
// Consecutive I2C errors while ranging before the sensor is treated as disconnected
static constexpr int MAX_READ_ERRORS = 3;

// Initialize the sensor and start ranging. On failure the driver is released so this can be retried.
static bool start_range_sensor(VL53L1X& sensor, const vl53l1x::Config& config)
{
    esp_err_t ret = sensor.init(config);
    if (ret == ESP_OK) {
        ret = sensor.start_ranging();
    }
    if (ret != ESP_OK) {
        ESP_LOGW("RANGE", "Range sensor not available (%s), retrying", esp_err_to_name(ret));
        sensor.deinit();
        return false;
        }
    ESP_LOGI("RANGE", "Range sensor ranging");
    return true;
}

// Format a VL53L1X result: a distance, or why there is none
static void describe_lidar(char* text, size_t size, int32_t range)
{
    switch (range) {
    case vl53l1x::RANGE_TIMEOUT: snprintf(text, size, "timeout"); break;
    case vl53l1x::RANGE_OUT_OF_RANGE: snprintf(text, size, "out of range"); break;
    case vl53l1x::RANGE_INVALID: snprintf(text, size, "invalid"); break;
    case vl53l1x::RANGE_ERROR: snprintf(text, size, "not available"); break;
    default: snprintf(text, size, "%" PRId32 " mm", range); break;
    }
}

// Format an MB1013 result: a distance, or why there is none
static void describe_sonar(char* text, size_t size, int32_t range)
{
    switch (range) {
    case mb1013::RANGE_UNRESOLVED: snprintf(text, size, "no pulse"); break;
    case mb1013::RANGE_IN_PROGRESS: snprintf(text, size, "in progress"); break;
    case mb1013::RANGE_ERROR: snprintf(text, size, "error"); break;
    default: snprintf(text, size, "%" PRId32 " mm", range); break;
    }
}


extern "C" void app_main(void)
{
    // The board and the Adafruit breakout both have I2C pull-ups, so the internal ones stay off
    static I2cBus i2c_bus(I2C_NUM_0, I2C_SDA_PIN, I2C_SCL_PIN);
    static I2cMasterDevice range_device(i2c_bus, vl53l1x::DEFAULT_ADDRESS);
    static VL53L1X range_sensor(range_device, RANGE_DETECTOR_XSHUT_PIN);
    static MB1013 sonar(SONAR_PW_PIN);

    ESP_ERROR_CHECK(i2c_bus.init());
    ESP_ERROR_CHECK(range_device.init());
    // Only claims an RMT channel, so this works with or without the sonar connected
    ESP_ERROR_CHECK(sonar.init());

    vl53l1x::Config config;
    config.distance_mode = vl53l1x::DistanceMode::Long;
    config.timing_budget_ms = 50;
    config.inter_measurement_ms = 100;
    // Reduce toward 4x4 to narrow the field of view
    config.roi_width = 16;
    config.roi_height = 16;

    uint8_t baseMac[6];
    if(esp_read_mac(baseMac, ESP_MAC_WIFI_STA) == ESP_OK) {
        ESP_LOGI("MAC", "MAC address: %02x:%02x:%02x:%02x:%02x:%02x",
                 baseMac[0], baseMac[1], baseMac[2],
                 baseMac[3], baseMac[4], baseMac[5]);
    }
    else {
        ESP_LOGE("MAC", "Failed to read MAC address");
    }

    // Both sensors measure about every 100 ms, so each pass reads one result from each.
    // A missing or failed range sensor is retried rather than treated as fatal, and the
    // sonar keeps reporting in the meantime.
    int read_errors = 0;
    bool retry_pending = false;
    TickType_t last_attempt = 0;
    while (true) {
        if (!range_sensor.is_ranging() && (!retry_pending || xTaskGetTickCount() - last_attempt >= SENSOR_RETRY_DELAY)) {
            last_attempt = xTaskGetTickCount();
            retry_pending = !start_range_sensor(range_sensor, config);
            read_errors = 0;
        }

        int32_t lidar = vl53l1x::RANGE_ERROR;
        if (range_sensor.is_ranging()) {
            lidar = range_sensor.read_range_mm(200);
            if (lidar == vl53l1x::RANGE_ERROR) {
                if (++read_errors >= MAX_READ_ERRORS) {
                    ESP_LOGW("RANGE", "Range sensor stopped responding");
                    range_sensor.deinit();
                    retry_pending = false;
                }
            } else {
                read_errors = 0;
            }
        }

        int32_t sonar_range = sonar.read_range_mm();
        // The read can start partway through a pulse; the next one is complete
        if (sonar_range == mb1013::RANGE_IN_PROGRESS) {
            sonar_range = sonar.read_range_mm();
        }

        char lidar_text[24];
        char sonar_text[24];
        describe_lidar(lidar_text, sizeof(lidar_text), lidar);
        describe_sonar(sonar_text, sizeof(sonar_text), sonar_range);
        ESP_LOGI("RANGE", "Lidar: %s | Sonar: %s", lidar_text, sonar_text);
    }
}
