#pragma once

#include "esp_err.h"
#include <cstdint>
#include <cstddef>

/**
 * @brief One device on an I2C bus, as seen by a sensor driver
 *
 * Drivers depend on this interface instead of the ESP-IDF I2C driver, so several drivers can share
 * one bus through I2cBus, and host tests can substitute a simulated device.
 */
class I2cDevice {
public:
    virtual ~I2cDevice() = default;

    /**
     * @brief Write bytes to the device in one transaction
     * @return esp_err_t ESP_OK on success, error code if the device did not acknowledge or the bus failed
     */
    virtual esp_err_t write(const uint8_t* data, size_t size) = 0;

    /**
     * @brief Write bytes, then read bytes after a repeated start, in one transaction
     * @return esp_err_t ESP_OK on success, error code if the device did not acknowledge or the bus failed
     */
    virtual esp_err_t write_read(const uint8_t* write_data, size_t write_size, uint8_t* read_data, size_t read_size) = 0;
};
