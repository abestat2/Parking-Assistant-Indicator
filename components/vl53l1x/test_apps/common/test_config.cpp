// Tier 1: configuration and result decoding tests, shared by the host and target test apps

#include "unity.h"
#include "VL53L1X_config.hpp"

using namespace vl53l1x;

TEST_CASE("config: defaults are valid", "[vl53l1x][config]")
{
    TEST_ASSERT_EQUAL(ESP_OK, validate_config(Config{}));
}

TEST_CASE("config: every supported timing budget is accepted in long mode", "[vl53l1x][config]")
{
    const uint16_t budgets[] = { 20, 33, 50, 100, 200, 500 };
    for (uint16_t budget : budgets) {
        Config config;
        config.timing_budget_ms = budget;
        config.inter_measurement_ms = budget;
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, validate_config(config), "budget rejected");
    }
}

TEST_CASE("config: 15 ms budget is only accepted in short mode", "[vl53l1x][config]")
{
    Config config;
    config.timing_budget_ms = 15;
    config.inter_measurement_ms = 15;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, validate_config(config));
    config.distance_mode = DistanceMode::Short;
    TEST_ASSERT_EQUAL(ESP_OK, validate_config(config));
}

TEST_CASE("config: unsupported timing budgets are rejected", "[vl53l1x][config]")
{
    const uint16_t budgets[] = { 0, 10, 25, 34, 1000 };
    for (uint16_t budget : budgets) {
        Config config;
        config.timing_budget_ms = budget;
        config.inter_measurement_ms = 1000;
        TEST_ASSERT_EQUAL_MESSAGE(ESP_ERR_INVALID_ARG, validate_config(config), "budget accepted");
    }
}

TEST_CASE("config: inter-measurement period shorter than the budget is rejected", "[vl53l1x][config]")
{
    Config config;
    config.timing_budget_ms = 100;
    config.inter_measurement_ms = 99;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, validate_config(config));
    config.inter_measurement_ms = 100;
    TEST_ASSERT_EQUAL(ESP_OK, validate_config(config));
}

TEST_CASE("config: region of interest must be 4 to 16 SPADs each way", "[vl53l1x][config]")
{
    Config config;
    config.roi_width = 4;
    config.roi_height = 4;
    TEST_ASSERT_EQUAL(ESP_OK, validate_config(config));

    config.roi_width = 3;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, validate_config(config));
    config.roi_width = 17;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, validate_config(config));

    config.roi_width = 16;
    config.roi_height = 3;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, validate_config(config));
    config.roi_height = 17;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, validate_config(config));
}

TEST_CASE("config: unknown distance mode is rejected", "[vl53l1x][config]")
{
    Config config;
    config.distance_mode = static_cast<DistanceMode>(3);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, validate_config(config));
}

TEST_CASE("decode: valid status returns the distance", "[vl53l1x][config]")
{
    TEST_ASSERT_EQUAL_INT32(0, decode_range(0, 0));
    TEST_ASSERT_EQUAL_INT32(1234, decode_range(0, 1234));
    TEST_ASSERT_EQUAL_INT32(65535, decode_range(0, 65535));
}

TEST_CASE("decode: signal, bounds and wraparound failures mean out of range", "[vl53l1x][config]")
{
    TEST_ASSERT_EQUAL_INT32(RANGE_OUT_OF_RANGE, decode_range(2, 500));
    TEST_ASSERT_EQUAL_INT32(RANGE_OUT_OF_RANGE, decode_range(4, 500));
    TEST_ASSERT_EQUAL_INT32(RANGE_OUT_OF_RANGE, decode_range(7, 500));
}

TEST_CASE("decode: other statuses are invalid", "[vl53l1x][config]")
{
    const uint8_t statuses[] = { 1, 3, 5, 6, 8, 9, 10, 255 };
    for (uint8_t status : statuses) {
        TEST_ASSERT_EQUAL_INT32_MESSAGE(RANGE_INVALID, decode_range(status, 500), "status not invalid");
    }
}

TEST_CASE("decode: result codes are distinct and negative", "[vl53l1x][config]")
{
    const int32_t codes[] = { RANGE_TIMEOUT, RANGE_OUT_OF_RANGE, RANGE_INVALID, RANGE_ERROR };
    for (size_t i = 0; i < 4; i++) {
        TEST_ASSERT_LESS_THAN_INT32(0, codes[i]);
        for (size_t j = i + 1; j < 4; j++) {
            TEST_ASSERT_NOT_EQUAL(codes[i], codes[j]);
        }
    }
}
