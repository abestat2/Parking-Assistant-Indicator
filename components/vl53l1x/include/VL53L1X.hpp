#pragma once

#include "hal/gpio_types.h"
#include "I2cDevice.hpp"
#include "VL53L1X_config.hpp"
#include <cstdint>

/**
 * @brief VL53L1X time-of-flight distance sensor, using ST's Ultra Lite Driver (ULD)
 *
 * Low power uses the sensor's software standby (about 6 uA): it stops ranging but keeps its
 * configuration, so resuming only needs start_ranging(). XSHUT stays high and is pulsed low only
 * to hard-reset an unresponsive sensor. Hardware standby (XSHUT held low) is not used because on
 * breakouts with a pull-up on XSHUT, such as Adafruit's, holding it low draws far more current
 * through the pull-up (about 220 uA) than software standby does.
 *
 * The sensor keeps its configuration while the ESP32 is in deep sleep, so init() after a
 * deep-sleep wake reuses it instead of running the ~100 ms ULD initialization again. Call
 * prepare_for_deep_sleep() first so XSHUT is held high while the ESP32 sleeps.
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
     *        connected. Without it, recovery uses a software reset over I2C instead.
     */
    VL53L1X(I2cDevice& i2c, gpio_num_t xshut_gpio = GPIO_NUM_NC);

    /**
     * @brief Destroy the VL53L1X object, leaving the sensor in software standby
     */
    ~VL53L1X();

    // Prevent copies so two objects never drive the same sensor and XSHUT pin
    VL53L1X(const VL53L1X&) = delete;
    VL53L1X& operator=(const VL53L1X&) = delete;

    /**
     * @brief Drive XSHUT high, wait for the sensor, initialize it if needed, and apply the configuration.
     *        The sensor is left in software standby, not ranging.
     *
     * If the sensor was already initialized by the ULD (for example before an ESP32 deep sleep),
     * the ~100 ms initialization is skipped and only the configuration is written. If the sensor
     * does not respond, it is hard-reset once and initialization is retried.
     *
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if already initialized,
     *         ESP_ERR_INVALID_ARG for an invalid config, ESP_ERR_NO_MEM if too many sensors exist,
     *         ESP_ERR_TIMEOUT if the sensor did not boot or finish initialization in time,
     *         ESP_ERR_NOT_FOUND if the device is not a VL53L1X, ESP_FAIL on an I2C error.
     *         On failure init() can be retried.
     */
    esp_err_t init(const vl53l1x::Config& config = {});

    /**
     * @brief Put the sensor in software standby (about 6 uA). It keeps its configuration.
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized, ESP_FAIL on an I2C error
     */
    esp_err_t shutdown();

    /**
     * @brief Check that the sensor is still configured, ready for start_ranging()
     *
     * This normally takes a couple of I2C reads. If the sensor lost power or stopped responding,
     * it is reset and fully reinitialized, which takes about 100 ms.
     *
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized, otherwise as for init()
     */
    esp_err_t wake();

    /**
     * @brief Start continuous ranging with the configured timing budget and period
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized, ESP_FAIL on an I2C error
     */
    esp_err_t start_ranging();

    /**
     * @brief Stop ranging, leaving the sensor in software standby
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized, ESP_FAIL on an I2C error
     */
    esp_err_t stop_ranging();

    /**
     * @brief Wait for the next measurement and return it
     * @param timeout_ms How long to wait for a measurement. Allow at least the inter-measurement period.
     * @return int32_t Distance in millimeters (>= 0), or one of the negative vl53l1x::RANGE_* codes
     */
    int32_t read_range_mm(uint32_t timeout_ms);

    /**
     * @brief Stop ranging and hold XSHUT high through ESP32 deep sleep
     *
     * Without the hold, the pin returns to its reset state during deep sleep. The hold is released
     * by init() after waking. On the ESP32-C3 this also enables the deep-sleep hold for all held
     * digital pins (gpio_deep_sleep_hold_en()).
     *
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized,
     *         ESP_FAIL on an I2C error, or a GPIO driver error
     */
    esp_err_t prepare_for_deep_sleep();

    /**
     * @brief The ULD range status of the last measurement read, or 255 if none was read
     */
    uint8_t last_range_status() const { return m_last_range_status; }

    /**
     * @brief Whether init() has succeeded and deinit() has not been called
     */
    bool is_initialized() const { return m_dev != NO_DEVICE; }

    /**
     * @brief Whether the sensor is ranging
     */
    bool is_ranging() const { return m_ranging; }

    /**
     * @brief Stop ranging and release the driver slot. XSHUT stays driven high.
     */
    void deinit();

private:
    static constexpr const char* TAG = "VL53L1X";
    static constexpr uint16_t NO_DEVICE = UINT16_MAX;
    // The datasheet gives a 1.2 ms maximum boot time; allow plenty of margin for slow I2C
    static constexpr uint32_t BOOT_TIMEOUT_MS = 100;
    // Initialization waits for one measurement with the default 100 ms timing budget
    static constexpr uint32_t SENSOR_INIT_TIMEOUT_MS = 500;
    static constexpr uint8_t NO_RANGE_STATUS = 255;

    esp_err_t bring_up(bool allow_reuse);
    esp_err_t try_bring_up(bool allow_reuse);
    void hard_reset();
    esp_err_t wait_for_boot();
    esp_err_t check_model_id();
    bool is_uld_initialized();
    esp_err_t sensor_init();
    esp_err_t apply_config();
    esp_err_t wait_for_data_ready(uint32_t timeout_ms);

    I2cDevice& m_i2c;
    gpio_num_t m_xshut_gpio;
    vl53l1x::Config m_config;
    uint16_t m_dev;
    bool m_ranging;
    uint8_t m_last_range_status;
};
