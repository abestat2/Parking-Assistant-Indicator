#pragma once

#include "hal/gpio_types.h"
#include "I2cDevice.hpp"
#include "VL53L1X_config.hpp"
#include <cstdint>

/**
 * @brief VL53L1X time-of-flight distance sensor, using ST's Ultra Lite Driver (ULD)
 *
 * The sensor can be held in hardware standby through its XSHUT pin (about 5 uA). Standby clears
 * the sensor's configuration, so wake() boots it, runs the ULD initialization and reapplies the
 * configuration given to init(). The sensor also returns to its default I2C address in standby.
 *
 * Typical use:
 *   I2cBus bus(I2C_NUM_0, GPIO_NUM_5, GPIO_NUM_6);
 *   I2cMasterDevice device(bus, vl53l1x::DEFAULT_ADDRESS);
 *   VL53L1X sensor(device, GPIO_NUM_4);
 *   bus.init(); device.init(); sensor.init(config);
 *   sensor.start_ranging(); sensor.read_range_mm(100);
 *   sensor.shutdown(); ... sensor.wake(); sensor.start_ranging();
 */
class VL53L1X {
public:
    /**
     * @brief Construct a new VL53L1X object
     * @param i2c The sensor's device on a shared I2C bus. It must outlive this object.
     * @param xshut_gpio GPIO driving the sensor's XSHUT pin, or GPIO_NUM_NC if XSHUT is not
     *        connected. Without it the sensor stays powered and shutdown() is not supported.
     */
    VL53L1X(I2cDevice& i2c, gpio_num_t xshut_gpio = GPIO_NUM_NC);

    /**
     * @brief Destroy the VL53L1X object, leaving the sensor in standby when XSHUT is connected
     */
    ~VL53L1X();

    // Prevent copies so two objects never drive the same sensor and XSHUT pin
    VL53L1X(const VL53L1X&) = delete;
    VL53L1X& operator=(const VL53L1X&) = delete;

    /**
     * @brief Configure XSHUT, then boot, initialize and configure the sensor. It is left awake but not ranging.
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if already initialized,
     *         ESP_ERR_INVALID_ARG for an invalid config, ESP_ERR_NO_MEM if too many sensors exist,
     *         otherwise as for wake(). On failure the sensor is left in standby and init() can be retried.
     */
    esp_err_t init(const vl53l1x::Config& config = {});

    /**
     * @brief Put the sensor into hardware standby by pulling XSHUT low
     * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_SUPPORTED without an XSHUT pin,
     *         ESP_ERR_INVALID_STATE if not initialized
     */
    esp_err_t shutdown();

    /**
     * @brief Release XSHUT, wait for the sensor to boot, then initialize and configure it
     *
     * Initialization includes one internal calibration measurement, so this takes longer than the
     * 1.2 ms boot time alone. Calling it while the sensor is already awake reinitializes it.
     *
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized,
     *         ESP_ERR_TIMEOUT if the sensor did not boot or finish initialization in time,
     *         ESP_ERR_NOT_FOUND if the device is not a VL53L1X, ESP_FAIL on an I2C error.
     *         On failure the sensor is put back in standby when XSHUT is connected.
     */
    esp_err_t wake();

    /**
     * @brief Start continuous ranging with the configured timing budget and period
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if the sensor is not awake, ESP_FAIL on an I2C error
     */
    esp_err_t start_ranging();

    /**
     * @brief Stop ranging. The sensor stays awake and configured.
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if the sensor is not awake, ESP_FAIL on an I2C error
     */
    esp_err_t stop_ranging();

    /**
     * @brief Wait for the next measurement and return it
     * @param timeout_ms How long to wait for a measurement. Allow at least the inter-measurement period.
     * @return int32_t Distance in millimeters (>= 0), or one of the negative vl53l1x::RANGE_* codes
     */
    int32_t read_range_mm(uint32_t timeout_ms);

    /**
     * @brief The ULD range status of the last measurement read, or 255 if none was read
     */
    uint8_t last_range_status() const { return m_last_range_status; }

    /**
     * @brief Whether the sensor is booted and configured
     */
    bool is_awake() const { return m_awake; }

    /**
     * @brief Stop ranging, put the sensor in standby if XSHUT is connected, and release the driver slot
     */
    void deinit();

private:
    static constexpr const char* TAG = "VL53L1X";
    // The datasheet gives a 1.2 ms maximum boot time; allow plenty of margin for slow I2C
    static constexpr uint32_t BOOT_TIMEOUT_MS = 100;
    // Initialization waits for one measurement with the default 100 ms timing budget
    static constexpr uint32_t SENSOR_INIT_TIMEOUT_MS = 500;
    static constexpr uint8_t NO_RANGE_STATUS = 255;

    esp_err_t wait_for_boot();
    esp_err_t sensor_init();
    esp_err_t apply_config();
    esp_err_t wait_for_data_ready(uint32_t timeout_ms);
    void enter_standby();

    I2cDevice& m_i2c;
    gpio_num_t m_xshut_gpio;
    vl53l1x::Config m_config;
    uint16_t m_dev;
    bool m_awake;
    bool m_ranging;
    uint8_t m_last_range_status;
};
