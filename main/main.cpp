#include <cinttypes>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_log.h"
#include "I2cBus.hpp"
#include "VL53L1X.hpp"

// Qwiic connector on the SparkFun Pro Micro ESP32-C3
#define I2C_SDA_PIN GPIO_NUM_5
#define I2C_SCL_PIN GPIO_NUM_6
#define RANGE_DETECTOR_XSHUT_PIN GPIO_NUM_4


extern "C" void app_main(void)
{
    // The board and the Adafruit breakout both have I2C pull-ups, so the internal ones stay off
    static I2cBus i2c_bus(I2C_NUM_0, I2C_SDA_PIN, I2C_SCL_PIN);
    static I2cMasterDevice range_device(i2c_bus, vl53l1x::DEFAULT_ADDRESS);
    static VL53L1X range_sensor(range_device, RANGE_DETECTOR_XSHUT_PIN);

    ESP_ERROR_CHECK(i2c_bus.init());
    ESP_ERROR_CHECK(range_device.init());

    vl53l1x::Config config;
    config.distance_mode = vl53l1x::DistanceMode::Long;
    config.timing_budget_ms = 50;
    config.inter_measurement_ms = 100;
    // Reduce toward 4x4 to narrow the field of view
    config.roi_width = 16;
    config.roi_height = 16;
    ESP_ERROR_CHECK(range_sensor.init(config));
    ESP_ERROR_CHECK(range_sensor.start_ranging());

    uint8_t baseMac[6];
    if(esp_read_mac(baseMac, ESP_MAC_WIFI_STA) == ESP_OK) {
        ESP_LOGI("MAC", "MAC address: %02x:%02x:%02x:%02x:%02x:%02x",
                 baseMac[0], baseMac[1], baseMac[2],
                 baseMac[3], baseMac[4], baseMac[5]);
    }
    else {
        ESP_LOGE("MAC", "Failed to read MAC address");
    }

    while (true) {
        int32_t range = range_sensor.read_range_mm(200);
        if (range >= 0) {
            ESP_LOGI("RANGE", "Range: %" PRId32 " mm", range);
        } else {
            ESP_LOGW("RANGE", "No range (%" PRId32 "), status %u", range, range_sensor.last_range_status());
        }
    }
}
