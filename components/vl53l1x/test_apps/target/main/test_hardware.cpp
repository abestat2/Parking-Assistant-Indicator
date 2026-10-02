// Tier 3: on-target tests with a real VL53L1X. Connect the sensor to the I2C pins and XSHUT pin
// set in menuconfig ("VL53L1X test app"), and point it at a surface 0.1 to 2 m away.

#include <cinttypes>
#include <cstdio>
#include "unity.h"
#include "unity_test_runner.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "I2cBus.hpp"
#include "VL53L1X.hpp"

using namespace vl53l1x;

static constexpr gpio_num_t SDA_GPIO = static_cast<gpio_num_t>(CONFIG_VL53L1X_TEST_SDA_GPIO);
static constexpr gpio_num_t SCL_GPIO = static_cast<gpio_num_t>(CONFIG_VL53L1X_TEST_SCL_GPIO);
static constexpr gpio_num_t XSHUT_GPIO = static_cast<gpio_num_t>(CONFIG_VL53L1X_TEST_XSHUT_GPIO);

// An address nothing on the test bus uses
static constexpr uint16_t UNUSED_ADDRESS = 0x08;

// Bus and sensor device, set up and torn down around each test so the leak check sees no change
struct Rig {
    I2cBus bus{ I2C_NUM_0, SDA_GPIO, SCL_GPIO };
    I2cMasterDevice device{ bus, DEFAULT_ADDRESS };

    Rig() {
        TEST_ESP_OK(bus.init());
        TEST_ESP_OK(device.init());
    }
};

static int64_t elapsed_us(int64_t start_us) {
    return esp_timer_get_time() - start_us;
}

// ---- I2cBus ----

TEST_CASE("i2c_bus: init twice is rejected and probe finds only the sensor", "[vl53l1x][hw]")
{
    Rig rig;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, rig.bus.init());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, rig.device.init());

    // Make sure the sensor is out of reset and booted
    VL53L1X sensor(rig.device, XSHUT_GPIO);
    TEST_ESP_OK(sensor.init());
    TEST_ESP_OK(rig.bus.probe(DEFAULT_ADDRESS));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, rig.bus.probe(UNUSED_ADDRESS));
}

TEST_CASE("i2c_bus: device before bus init and use after deinit are rejected", "[vl53l1x][hw]")
{
    I2cBus bus(I2C_NUM_0, SDA_GPIO, SCL_GPIO);
    I2cMasterDevice device(bus, DEFAULT_ADDRESS);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, device.init());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, bus.probe(DEFAULT_ADDRESS));

    uint8_t byte = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, device.write(&byte, 1));

    TEST_ESP_OK(bus.init());
    TEST_ESP_OK(device.init());
    device.deinit();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, device.write_read(&byte, 1, &byte, 1));
}

TEST_CASE("i2c_bus: two devices share one bus", "[vl53l1x][hw]")
{
    Rig rig;
    VL53L1X sensor(rig.device, XSHUT_GPIO);
    TEST_ESP_OK(sensor.init());
    I2cMasterDevice other(rig.bus, UNUSED_ADDRESS);
    TEST_ESP_OK(other.init());

    // Read the model ID register directly through each device
    uint8_t index[2] = { 0x01, 0x0F };
    uint8_t id[2] = {};
    TEST_ESP_OK(rig.device.write_read(index, sizeof(index), id, sizeof(id)));
    TEST_ASSERT_EQUAL_HEX16(MODEL_ID, id[0] << 8 | id[1]);
    // Nothing answers at the other address
    TEST_ASSERT_NOT_EQUAL(ESP_OK, other.write_read(index, sizeof(index), id, sizeof(id)));
}

// ---- VL53L1X ----

TEST_CASE("hw: init and measure a distance", "[vl53l1x][hw]")
{
    Rig rig;
    VL53L1X sensor(rig.device, XSHUT_GPIO);
    int64_t start = esp_timer_get_time();
    TEST_ESP_OK(sensor.init());
    printf("init took %" PRId64 " us\n", elapsed_us(start));

    TEST_ESP_OK(sensor.start_ranging());
    for (int i = 0; i < 10; i++) {
        int32_t range = sensor.read_range_mm(200);
        printf("range %" PRId32 " mm, status %u\n", range, sensor.last_range_status());
        TEST_ASSERT_GREATER_OR_EQUAL_INT32(0, range);
        TEST_ASSERT_LESS_THAN_INT32(4000, range);
    }
}

TEST_CASE("hw: shutdown keeps the sensor on the bus and wake resumes it quickly", "[vl53l1x][hw]")
{
    Rig rig;
    VL53L1X sensor(rig.device, XSHUT_GPIO);
    TEST_ESP_OK(sensor.init());
    TEST_ESP_OK(sensor.start_ranging());
    TEST_ESP_OK(sensor.shutdown());
    // Software standby still answers I2C
    TEST_ESP_OK(rig.bus.probe(DEFAULT_ADDRESS));

    int64_t start = esp_timer_get_time();
    TEST_ESP_OK(sensor.wake());
    int64_t wake_us = elapsed_us(start);
    TEST_ESP_OK(sensor.start_ranging());
    int32_t range = sensor.read_range_mm(200);
    int64_t first_range_us = elapsed_us(start);
    printf("wake took %" PRId64 " us, first range %" PRId32 " mm after %" PRId64 " us\n", wake_us, range, first_range_us);
    TEST_ASSERT_GREATER_OR_EQUAL_INT32(0, range);
    // Only a couple of I2C reads, no reinitialization
    TEST_ASSERT_LESS_THAN_INT32(5000, static_cast<int32_t>(wake_us));
}

TEST_CASE("hw: init reuses an initialized sensor and fully initializes a reset one", "[vl53l1x][hw]")
{
    Rig rig;
    {
        VL53L1X sensor(rig.device, XSHUT_GPIO);
        TEST_ESP_OK(sensor.init());
    }

    // Already initialized: only the configuration is written
    int64_t reuse_us;
    {
        VL53L1X sensor(rig.device, XSHUT_GPIO);
        int64_t start = esp_timer_get_time();
        TEST_ESP_OK(sensor.init());
        reuse_us = elapsed_us(start);
    }

    // Pulse XSHUT to reset the sensor. Its VHV registers return to reset values that differ from
    // the ones the ULD initialization leaves, so init() must run the full initialization.
    gpio_set_level(XSHUT_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(XSHUT_GPIO, 1);
    int64_t full_us;
    {
        VL53L1X sensor(rig.device, XSHUT_GPIO);
        int64_t start = esp_timer_get_time();
        TEST_ESP_OK(sensor.init());
        full_us = elapsed_us(start);
        TEST_ESP_OK(sensor.start_ranging());
        TEST_ASSERT_GREATER_OR_EQUAL_INT32(0, sensor.read_range_mm(200));
    }

    printf("init reusing the sensor took %" PRId64 " us, full init %" PRId64 " us\n", reuse_us, full_us);
    TEST_ASSERT_LESS_THAN_INT32(20000, static_cast<int32_t>(reuse_us));
    // Full initialization includes one measurement with the 100 ms default timing budget
    TEST_ASSERT_GREATER_THAN_INT32(50000, static_cast<int32_t>(full_us));
}

static void deep_sleep_stage_1()
{
    Rig rig;
    VL53L1X sensor(rig.device, XSHUT_GPIO);
    TEST_ESP_OK(sensor.init());
    TEST_ESP_OK(sensor.start_ranging());
    TEST_ASSERT_GREATER_OR_EQUAL_INT32(0, sensor.read_range_mm(200));
    TEST_ESP_OK(sensor.prepare_for_deep_sleep());
    esp_sleep_enable_timer_wakeup(1000 * 1000);
    esp_deep_sleep_start();
}

static void deep_sleep_stage_2()
{
    TEST_ASSERT_EQUAL(ESP_RST_DEEPSLEEP, esp_reset_reason());
    Rig rig;
    VL53L1X sensor(rig.device, XSHUT_GPIO);
    int64_t start = esp_timer_get_time();
    TEST_ESP_OK(sensor.init());
    int64_t init_us = elapsed_us(start);
    gpio_deep_sleep_hold_dis();

    TEST_ESP_OK(sensor.start_ranging());
    int32_t range = sensor.read_range_mm(200);
    printf("init after deep sleep took %" PRId64 " us, range %" PRId32 " mm\n", init_us, range);
    TEST_ASSERT_GREATER_OR_EQUAL_INT32(0, range);
    // XSHUT was held high, so the sensor kept its initialization through the sleep
    TEST_ASSERT_LESS_THAN_INT32(20000, static_cast<int32_t>(init_us));
}

TEST_CASE_MULTIPLE_STAGES("hw: sensor keeps its initialization through ESP32 deep sleep", "[vl53l1x][hw]",
                          deep_sleep_stage_1, deep_sleep_stage_2);

TEST_CASE("hw: narrow region of interest still measures", "[vl53l1x][hw]")
{
    Rig rig;
    VL53L1X sensor(rig.device, XSHUT_GPIO);
    Config config;
    config.roi_width = 4;
    config.roi_height = 4;
    TEST_ESP_OK(sensor.init(config));
    TEST_ESP_OK(sensor.start_ranging());
    int32_t range = sensor.read_range_mm(200);
    printf("4x4 ROI range %" PRId32 " mm, status %u\n", range, sensor.last_range_status());
    TEST_ASSERT_GREATER_OR_EQUAL_INT32(0, range);
}

TEST_CASE("hw: short mode with the fastest timing budget", "[vl53l1x][hw]")
{
    Rig rig;
    VL53L1X sensor(rig.device, XSHUT_GPIO);
    Config config;
    config.distance_mode = DistanceMode::Short;
    config.timing_budget_ms = 20;
    config.inter_measurement_ms = 25;
    TEST_ESP_OK(sensor.init(config));
    TEST_ESP_OK(sensor.start_ranging());

    int64_t start = esp_timer_get_time();
    for (int i = 0; i < 20; i++) {
        TEST_ASSERT_GREATER_OR_EQUAL_INT32(0, sensor.read_range_mm(100));
    }
    // 20 measurements at 25 ms, with room for the 10 ms tick on the default config
    int32_t total_ms = static_cast<int32_t>(elapsed_us(start) / 1000);
    printf("20 measurements took %" PRId32 " ms\n", total_ms);
    TEST_ASSERT_LESS_THAN_INT32(1000, total_ms);
}

TEST_CASE("hw: read after stop_ranging reports an error", "[vl53l1x][hw]")
{
    Rig rig;
    VL53L1X sensor(rig.device, XSHUT_GPIO);
    TEST_ESP_OK(sensor.init());
    TEST_ESP_OK(sensor.start_ranging());
    TEST_ASSERT_GREATER_OR_EQUAL_INT32(0, sensor.read_range_mm(200));
    TEST_ESP_OK(sensor.stop_ranging());
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm(200));
}

TEST_CASE("hw: wrong address fails init and leaves the sensor powered", "[vl53l1x][hw]")
{
    Rig rig;
    I2cMasterDevice wrong(rig.bus, UNUSED_ADDRESS);
    TEST_ESP_OK(wrong.init());
    VL53L1X sensor(wrong, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, sensor.init());
    TEST_ASSERT_FALSE(sensor.is_initialized());
    // The reset attempt released XSHUT again, so the real sensor is back on the bus
    vTaskDelay(pdMS_TO_TICKS(5));
    TEST_ESP_OK(rig.bus.probe(DEFAULT_ADDRESS));
}

TEST_CASE("hw: 20 shutdown and wake cycles without leaking", "[vl53l1x][hw]")
{
    Rig rig;
    VL53L1X sensor(rig.device, XSHUT_GPIO);
    TEST_ESP_OK(sensor.init());
    for (int i = 0; i < 20; i++) {
        TEST_ESP_OK(sensor.shutdown());
        TEST_ESP_OK(sensor.wake());
        TEST_ESP_OK(sensor.start_ranging());
        TEST_ASSERT_GREATER_OR_EQUAL_INT32(0, sensor.read_range_mm(200));
    }
    // The leak check runs in tearDown()
}
