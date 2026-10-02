#pragma once

// Glue between the ST ULD, which identifies a sensor by a uint16_t "dev" value, and I2cDevice.
// Each VL53L1X object registers its I2C device here and passes the returned id to the ULD.

#include "I2cDevice.hpp"
#include <cstdint>
#include <cstddef>

namespace vl53l1x::platform {

constexpr size_t MAX_SENSORS = 4;
constexpr uint16_t NO_DEVICE = UINT16_MAX;

/**
 * @brief Register an I2C device for use by the ULD
 * @return uint16_t The id to pass to ULD calls, or NO_DEVICE if all slots are taken
 */
uint16_t register_device(I2cDevice& device);

/**
 * @brief Release an id from register_device(). NO_DEVICE is ignored.
 */
void unregister_device(uint16_t dev);

} // namespace vl53l1x::platform
