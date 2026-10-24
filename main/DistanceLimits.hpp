#pragma once

#include "detection_link/Protocol.hpp"
#include "esp_err.h"
#include <cstdint>

/**
 * @brief The stop and guide distances the LED bar works between, kept in flash (NVS)
 *
 * min_mm is the stop distance (one LED lit, closer is all red) and max_mm is the distance where the
 * whole bar is lit. The detector can replace them over the link; the new values are saved so they
 * survive deep sleep and power loss. Until then the menuconfig values apply. NVS must already be
 * initialised.
 */
namespace distance_limits {

// What the lidar can usefully report
constexpr uint16_t LOWEST_MM = 30;
constexpr uint16_t HIGHEST_MM = 4000;

constexpr bool valid(detection_link::RangeLimits limits)
{
    return limits.min_mm >= LOWEST_MM && limits.max_mm <= HIGHEST_MM && limits.min_mm < limits.max_mm;
}

static_assert(valid({300, 2000}));
static_assert(!valid({}));
static_assert(!valid({2000, 300}));
static_assert(!valid({300, 300}));
static_assert(!valid({29, 2000}));
static_assert(!valid({300, 4001}));

// The saved limits, or the menuconfig ones if none are saved
detection_link::RangeLimits load();

// Save new limits. ESP_ERR_INVALID_ARG if they are not valid().
esp_err_t save(detection_link::RangeLimits limits);

} // namespace distance_limits
