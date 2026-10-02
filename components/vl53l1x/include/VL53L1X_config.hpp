#pragma once

#include "esp_err.h"
#include <cstdint>

namespace vl53l1x {

// Factory 7-bit I2C address. The sensor returns to it whenever XSHUT is pulled low.
constexpr uint16_t DEFAULT_ADDRESS = 0x29;
// Value of the IDENTIFICATION__MODEL_ID register on a VL53L1X
constexpr uint16_t MODEL_ID = 0xEACC;

enum class DistanceMode : uint16_t {
    Short = 1, // Up to about 1.3 m, better ambient light immunity
    Long = 2,  // Up to about 4 m in the dark
};

struct Config {
    DistanceMode distance_mode = DistanceMode::Long;
    // One of 15 (short mode only), 20, 33, 50, 100, 200 or 500. Longer budgets are more repeatable.
    uint16_t timing_budget_ms = 50;
    // Period between measurements while ranging; must be at least the timing budget
    uint32_t inter_measurement_ms = 50;
    // Region of interest in SPADs, 4 to 16 each. 16x16 is the full ~27 degree field of view;
    // 4x4 narrows it to about 15 degrees at the cost of range.
    uint8_t roi_width = 16;
    uint8_t roi_height = 16;
};

// Results returned in place of a distance. All are negative, so any value >= 0 is a valid distance in mm.

// No measurement completed within the timeout
constexpr int32_t RANGE_TIMEOUT = -1;
// The sensor found no target it could measure: the return signal was too weak or out of bounds
constexpr int32_t RANGE_OUT_OF_RANGE = -2;
// The sensor produced a measurement but flagged it unreliable, for example too noisy
constexpr int32_t RANGE_INVALID = -3;
// The driver is not initialized, the sensor is not ranging, or I2C communication failed
constexpr int32_t RANGE_ERROR = -4;

/**
 * @brief Check a configuration against the limits of the sensor and the ST driver
 * @return esp_err_t ESP_OK if valid, ESP_ERR_INVALID_ARG otherwise
 */
esp_err_t validate_config(const Config& config);

/**
 * @brief Convert a range status from the ST driver and the reported distance into a result
 *
 * The statuses are the ULD values (UM2510): 0 valid, 1 sigma failure, 2 signal failure,
 * 4 phase out of bounds, 7 wraparound, others (including 255) not usable.
 *
 * @return int32_t Distance in mm, or one of the RANGE_* codes above
 */
int32_t decode_range(uint8_t range_status, uint16_t distance_mm);

} // namespace vl53l1x
