#include "VL53L1X.hpp"
#include "vl53l1x_platform.hpp"
#include "vl53l1x_uld.h"
#include "driver/gpio.h"
#include "soc/soc_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include <cstring>

// The ST header has no C++ guards
extern "C" {
#include "VL53L1X_api.h"
}

using namespace vl53l1x;

// First register written by the ULD default configuration
static constexpr uint16_t DEFAULT_CONFIGURATION_START = 0x2D;
// VHV (temperature calibration) register set after the first measurement, as in VL53L1X_SensorInit()
static constexpr uint16_t VHV_CONFIG__INIT = 0x0B;
// Values VL53L1X_SensorInit() leaves in the VHV registers. Nothing else in the ULD writes them,
// so finding them means the sensor was initialized and has not been reset or power-cycled since.
static constexpr uint8_t VHV_LOOP_BOUND_AFTER_INIT = 0x09;
static constexpr uint8_t VHV_INIT_AFTER_INIT = 0x00;

// Wait at least one tick, so polling loops always yield to other tasks
static void delay_ms(uint32_t ms) {
    TickType_t ticks = pdMS_TO_TICKS(ms);
    vTaskDelay(ticks > 0 ? ticks : 1);
}

static bool expired(TickType_t start, uint32_t timeout_ms) {
    return xTaskGetTickCount() - start >= pdMS_TO_TICKS(timeout_ms);
}

VL53L1X::VL53L1X(I2cDevice& i2c, gpio_num_t xshut_gpio)
    : m_i2c(i2c), m_xshut_gpio(xshut_gpio), m_config(), m_dev(NO_DEVICE), m_ranging(false),
      m_last_range_status(NO_RANGE_STATUS) {
    static_assert(NO_DEVICE == platform::NO_DEVICE, "Driver and platform must agree on the unregistered id");
}

VL53L1X::~VL53L1X() {
    deinit();
}

esp_err_t VL53L1X::init(const Config& config) {
    // Refuse to re-initialize so an existing registration is never overwritten or leaked
    ESP_RETURN_ON_FALSE(m_dev == NO_DEVICE, ESP_ERR_INVALID_STATE, TAG, "Already initialized");
    ESP_RETURN_ON_ERROR(validate_config(config), TAG, "Invalid configuration");

    m_dev = platform::register_device(m_i2c);
    ESP_RETURN_ON_FALSE(m_dev != NO_DEVICE, ESP_ERR_NO_MEM, TAG, "Too many sensors");
    m_config = config;

    if (m_xshut_gpio != GPIO_NUM_NC) {
        ESP_LOGI(TAG, "Initializing VL53L1X with XSHUT on GPIO %d", m_xshut_gpio);
        // Set the level before enabling the output, and both before releasing a hold from
        // prepare_for_deep_sleep(), so XSHUT never glitches low and resets the sensor
        gpio_set_level(m_xshut_gpio, 1);
        gpio_config_t io_config = {};
        io_config.pin_bit_mask = 1ULL << m_xshut_gpio;
        io_config.mode = GPIO_MODE_OUTPUT;
        esp_err_t ret = gpio_config(&io_config);
        if (ret == ESP_OK) {
            ret = gpio_hold_dis(m_xshut_gpio);
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure XSHUT: %s", esp_err_to_name(ret));
            platform::unregister_device(m_dev);
            m_dev = NO_DEVICE;
            return ret;
        }
    } else {
        ESP_LOGI(TAG, "Initializing VL53L1X without XSHUT");
    }

    esp_err_t ret = bring_up(true);
    if (ret != ESP_OK) {
        platform::unregister_device(m_dev);
        m_dev = NO_DEVICE;
    }
    return ret;
}

esp_err_t VL53L1X::shutdown() {
    ESP_RETURN_ON_FALSE(m_dev != NO_DEVICE, ESP_ERR_INVALID_STATE, TAG, "Not initialized");
    // A sensor that is not ranging is already in software standby
    if (!m_ranging) {
        return ESP_OK;
    }
    return stop_ranging();
}

esp_err_t VL53L1X::wake() {
    ESP_RETURN_ON_FALSE(m_dev != NO_DEVICE, ESP_ERR_INVALID_STATE, TAG, "Not initialized");
    if (m_ranging) {
        return ESP_OK;
    }

    uint8_t state = 0;
    if (VL53L1X_BootState(m_dev, &state) == 0 && (state & 0x01) && is_uld_initialized()) {
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Sensor lost its configuration, reinitializing");
    return bring_up(true);
}

esp_err_t VL53L1X::start_ranging() {
    ESP_RETURN_ON_FALSE(m_dev != NO_DEVICE, ESP_ERR_INVALID_STATE, TAG, "Not initialized");
    // Discard any measurement left over from before
    int8_t status = VL53L1X_ClearInterrupt(m_dev);
    status |= VL53L1X_StartRanging(m_dev);
    ESP_RETURN_ON_FALSE(status == 0, ESP_FAIL, TAG, "Failed to start ranging");
    m_ranging = true;
    return ESP_OK;
}

esp_err_t VL53L1X::stop_ranging() {
    ESP_RETURN_ON_FALSE(m_dev != NO_DEVICE, ESP_ERR_INVALID_STATE, TAG, "Not initialized");
    ESP_RETURN_ON_FALSE(VL53L1X_StopRanging(m_dev) == 0, ESP_FAIL, TAG, "Failed to stop ranging");
    m_ranging = false;
    return ESP_OK;
}

int32_t VL53L1X::read_range_mm(uint32_t timeout_ms) {
    if (!m_ranging) {
        ESP_LOGE(TAG, "Sensor not ranging");
        return RANGE_ERROR;
    }

    esp_err_t ret = wait_for_data_ready(timeout_ms);
    if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Timed out waiting for a measurement");
        return RANGE_TIMEOUT;
    }
    if (ret != ESP_OK) {
        return RANGE_ERROR;
    }

    VL53L1X_Result_t result = {};
    int8_t status = VL53L1X_GetResult(m_dev, &result);
    // Clearing the interrupt lets the sensor report the next measurement
    status |= VL53L1X_ClearInterrupt(m_dev);
    if (status != 0) {
        ESP_LOGE(TAG, "Failed to read the measurement");
        return RANGE_ERROR;
    }

    m_last_range_status = result.Status;
    return decode_range(result.Status, result.Distance);
}

esp_err_t VL53L1X::prepare_for_deep_sleep() {
    ESP_RETURN_ON_ERROR(shutdown(), TAG, "Failed to enter standby");
    if (m_xshut_gpio == GPIO_NUM_NC) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(gpio_hold_en(m_xshut_gpio), TAG, "Failed to hold XSHUT");
#if !SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP
    // On chips such as the ESP32-C3, gpio_hold_en() alone does not survive deep sleep
    gpio_deep_sleep_hold_en();
#endif
    return ESP_OK;
}

void VL53L1X::deinit() {
    if (m_dev == NO_DEVICE) {
        return;
    }
    if (m_ranging) {
        VL53L1X_StopRanging(m_dev);
    }
    platform::unregister_device(m_dev);
    m_dev = NO_DEVICE;
    m_ranging = false;
    ESP_LOGI(TAG, "VL53L1X driver released");
}

esp_err_t VL53L1X::bring_up(bool allow_reuse) {
    esp_err_t ret = try_bring_up(allow_reuse);
    if (ret == ESP_OK) {
        return ESP_OK;
    }

    // The sensor may be stuck or half-configured; reset it and start from scratch once
    ESP_LOGW(TAG, "Sensor setup failed (%s), resetting it", esp_err_to_name(ret));
    hard_reset();
    ret = try_bring_up(false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set up sensor: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t VL53L1X::try_bring_up(bool allow_reuse) {
    m_ranging = false;
    ESP_RETURN_ON_ERROR(wait_for_boot(), TAG, "Sensor did not boot");
    ESP_RETURN_ON_ERROR(check_model_id(), TAG, "Not a VL53L1X");

    if (allow_reuse && is_uld_initialized()) {
        // Still initialized from before, for example an ESP32 deep sleep. It may have been left
        // ranging, so stop it and drop any pending measurement.
        ESP_LOGI(TAG, "Sensor already initialized, reusing it");
        int8_t status = VL53L1X_StopRanging(m_dev);
        status |= VL53L1X_ClearInterrupt(m_dev);
        ESP_RETURN_ON_FALSE(status == 0, ESP_FAIL, TAG, "Failed to stop ranging");
    } else {
        ESP_RETURN_ON_ERROR(sensor_init(), TAG, "Initialization failed");
    }

    return apply_config();
}

esp_err_t VL53L1X::hard_reset() {
    m_ranging = false;
    if (m_xshut_gpio != GPIO_NUM_NC) {
        gpio_set_level(m_xshut_gpio, 0);
        delay_ms(1);
        gpio_set_level(m_xshut_gpio, 1);
        return ESP_OK;
    }
    // Without XSHUT, use the software reset. It fails harmlessly if the sensor is not responding.
    int8_t status = VL53L1_WrByte(m_dev, SOFT_RESET, 0x00);
    delay_ms(1);
    status |= VL53L1_WrByte(m_dev, SOFT_RESET, 0x01);
    return status == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t VL53L1X::wait_for_boot() {
    // The sensor does not acknowledge I2C until it has booted, so read errors are expected at first
    TickType_t start = xTaskGetTickCount();
    while (true) {
        uint8_t state = 0;
        if (VL53L1X_BootState(m_dev, &state) == 0 && (state & 0x01)) {
            return ESP_OK;
        }
        if (expired(start, BOOT_TIMEOUT_MS)) {
            return ESP_ERR_TIMEOUT;
        }
        delay_ms(1);
    }
}

esp_err_t VL53L1X::check_model_id() {
    uint16_t model_id = 0;
    ESP_RETURN_ON_FALSE(VL53L1X_GetSensorId(m_dev, &model_id) == 0, ESP_FAIL, TAG, "Failed to read model ID");
    ESP_RETURN_ON_FALSE(model_id == MODEL_ID, ESP_ERR_NOT_FOUND, TAG, "Unexpected model ID 0x%04x", model_id);
    return ESP_OK;
}

bool VL53L1X::is_uld_initialized() {
    uint8_t loop_bound = 0;
    uint8_t vhv_init = 0;
    int8_t status = VL53L1_RdByte(m_dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, &loop_bound);
    status |= VL53L1_RdByte(m_dev, VHV_CONFIG__INIT, &vhv_init);
    return status == 0 && loop_bound == VHV_LOOP_BOUND_AFTER_INIT && vhv_init == VHV_INIT_AFTER_INIT;
}

esp_err_t VL53L1X::sensor_init() {
    // Same sequence as VL53L1X_SensorInit(), but with the configuration written in one transaction
    // and a timeout on the first measurement
    size_t size = 0;
    const uint8_t* defaults = vl53l1x_uld_default_configuration(&size);
    uint8_t configuration[128];
    ESP_RETURN_ON_FALSE(size <= sizeof(configuration), ESP_ERR_INVALID_SIZE, TAG, "Configuration too large");
    // The ULD write functions take a non-const buffer
    std::memcpy(configuration, defaults, size);

    int8_t status = VL53L1_WriteMulti(m_dev, DEFAULT_CONFIGURATION_START, configuration, size);
    status |= VL53L1X_StartRanging(m_dev);
    ESP_RETURN_ON_FALSE(status == 0, ESP_FAIL, TAG, "Failed to write the default configuration");

    ESP_RETURN_ON_ERROR(wait_for_data_ready(SENSOR_INIT_TIMEOUT_MS), TAG, "No initial measurement");

    status = VL53L1X_ClearInterrupt(m_dev);
    status |= VL53L1X_StopRanging(m_dev);
    status |= VL53L1_WrByte(m_dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, VHV_LOOP_BOUND_AFTER_INIT); // Two bounds VHV
    status |= VL53L1_WrByte(m_dev, VHV_CONFIG__INIT, VHV_INIT_AFTER_INIT); // Start VHV from the previous temperature
    ESP_RETURN_ON_FALSE(status == 0, ESP_FAIL, TAG, "Failed to finish initialization");
    return ESP_OK;
}

esp_err_t VL53L1X::apply_config() {
    // The distance mode must be set before the timing budget, whose register values depend on it
    int8_t status = VL53L1X_SetDistanceMode(m_dev, static_cast<uint16_t>(m_config.distance_mode));
    status |= VL53L1X_SetTimingBudgetInMs(m_dev, m_config.timing_budget_ms);
    status |= VL53L1X_SetInterMeasurementInMs(m_dev, m_config.inter_measurement_ms);
    status |= VL53L1X_SetROI(m_dev, m_config.roi_width, m_config.roi_height);
    ESP_RETURN_ON_FALSE(status == 0, ESP_FAIL, TAG, "Failed to apply configuration");
    return ESP_OK;
}

esp_err_t VL53L1X::wait_for_data_ready(uint32_t timeout_ms) {
    TickType_t start = xTaskGetTickCount();
    while (true) {
        uint8_t ready = 0;
        if (VL53L1X_CheckForDataReady(m_dev, &ready) != 0) {
            ESP_LOGE(TAG, "Failed to check for data");
            return ESP_FAIL;
        }
        if (ready) {
            return ESP_OK;
        }
        if (expired(start, timeout_ms)) {
            return ESP_ERR_TIMEOUT;
        }
        delay_ms(1);
    }
}
