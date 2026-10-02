#pragma once

#include "driver/i2c_master.h"
#include "hal/gpio_types.h"
#include "I2cDevice.hpp"
#include <cstdint>

/**
 * @brief Owns one I2C master bus that several device drivers share
 *
 * Create one I2cBus per set of SDA/SCL pins, then one I2cMasterDevice per device on it. The ESP-IDF
 * master driver serializes transactions with a bus lock, so drivers on different tasks can use their
 * devices without extra locking. The bus must outlive every device added to it.
 */
class I2cBus {
public:
    /**
     * @brief Construct a new I2cBus object
     * @param port I2C controller to use, or -1 to let the driver pick a free one
     * @param sda_gpio SDA pin
     * @param scl_gpio SCL pin
     * @param enable_internal_pullups Enable the weak internal pull-ups. Leave off when the board has
     *        external pull-ups, as most breakout boards do.
     */
    I2cBus(i2c_port_num_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, bool enable_internal_pullups = false);

    /**
     * @brief Destroy the I2cBus object and release the bus
     */
    ~I2cBus();

    // Prevent copies to avoid releasing the same bus twice
    I2cBus(const I2cBus&) = delete;
    I2cBus& operator=(const I2cBus&) = delete;

    /**
     * @brief Create the I2C master bus
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if already initialized, other error code on failure
     */
    esp_err_t init();

    /**
     * @brief Release the bus. Every device on it must be deinitialized first.
     */
    void deinit();

    /**
     * @brief Check whether a device acknowledges its address
     * @return esp_err_t ESP_OK if it acknowledged, ESP_ERR_NOT_FOUND if not, ESP_ERR_INVALID_STATE if not initialized
     */
    esp_err_t probe(uint16_t address);

    /**
     * @brief The underlying ESP-IDF bus handle, or nullptr when not initialized
     */
    i2c_master_bus_handle_t handle() const { return m_bus; }

private:
    static constexpr const char* TAG = "I2cBus";
    static constexpr int PROBE_TIMEOUT_MS = 50;

    i2c_port_num_t m_port;
    gpio_num_t m_sda_gpio;
    gpio_num_t m_scl_gpio;
    bool m_enable_internal_pullups;
    i2c_master_bus_handle_t m_bus;
};

/**
 * @brief A device on an I2cBus, addressed with a 7-bit address
 */
class I2cMasterDevice : public I2cDevice {
public:
    /**
     * @brief Construct a new I2cMasterDevice object
     * @param bus Bus the device is on. It must be initialized before init() and outlive this object.
     * @param address 7-bit device address
     * @param scl_speed_hz Clock speed used for this device's transactions
     * @param timeout_ms Timeout for each transaction
     */
    I2cMasterDevice(I2cBus& bus, uint16_t address, uint32_t scl_speed_hz = 400000, int timeout_ms = 50);

    /**
     * @brief Destroy the I2cMasterDevice object and remove it from the bus
     */
    ~I2cMasterDevice() override;

    // Prevent copies to avoid removing the same device twice
    I2cMasterDevice(const I2cMasterDevice&) = delete;
    I2cMasterDevice& operator=(const I2cMasterDevice&) = delete;

    /**
     * @brief Add the device to the bus
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if already initialized or the bus is not,
     *         other error code on failure
     */
    esp_err_t init();

    /**
     * @brief Remove the device from the bus
     */
    void deinit();

    esp_err_t write(const uint8_t* data, size_t size) override;
    esp_err_t write_read(const uint8_t* write_data, size_t write_size, uint8_t* read_data, size_t read_size) override;

    uint16_t address() const { return m_address; }

private:
    static constexpr const char* TAG = "I2cMasterDevice";

    I2cBus& m_bus;
    uint16_t m_address;
    uint32_t m_scl_speed_hz;
    int m_timeout_ms;
    i2c_master_dev_handle_t m_device;
};
