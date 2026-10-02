#include "VL53L1X_config.hpp"

namespace vl53l1x {

esp_err_t validate_config(const Config& config) {
    if (config.distance_mode != DistanceMode::Short && config.distance_mode != DistanceMode::Long) {
        return ESP_ERR_INVALID_ARG;
    }

    // The ST driver only has register values for these budgets, and 15 ms only in short mode
    switch (config.timing_budget_ms) {
    case 15:
        if (config.distance_mode != DistanceMode::Short) {
            return ESP_ERR_INVALID_ARG;
        }
        break;
    case 20:
    case 33:
    case 50:
    case 100:
    case 200:
    case 500:
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    if (config.inter_measurement_ms < config.timing_budget_ms) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config.roi_width < 4 || config.roi_width > 16 || config.roi_height < 4 || config.roi_height > 16) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

int32_t decode_range(uint8_t range_status, uint16_t distance_mm) {
    switch (range_status) {
    case 0:
        return distance_mm;
    case 2: // Signal failure: return too weak, usually nothing within range
    case 4: // Phase out of bounds: target beyond the measurable range
    case 7: // Wraparound: a far, bright target aliased to a short distance
        return RANGE_OUT_OF_RANGE;
    default:
        return RANGE_INVALID;
    }
}

} // namespace vl53l1x
