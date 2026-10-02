#include "VL53L1X.hpp"
#include "vl53l1x_platform.hpp"
#include "driver/gpio.h"
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
// VHV (temperature calibration) registers set after the first measurement, as in VL53L1X_SensorInit()
static constexpr uint16_t VHV_CONFIG__INIT = 0x0B;

// Wait at least one tick, so polling loops always yield to other tasks
static void delay_ms(uint32_t ms) {
    TickType_t ticks = pdMS_TO_TICKS(ms);
    vTaskDelay(ticks > 0 ? ticks : 1);
}

static bool expired(TickType_t start, uint32_t timeout_ms) {
    return xTaskGetTickCount() - start >= pdMS_TO_TICKS(timeout_ms);
}

VL53L1X::VL53L1X(I2cDevice& i2c, gpio_num_t xshut_gpio)
    : m_i2c(i2c), m_xshut_gpio(xshut_gpio), m_config(), m_dev(platform::NO_DEVICE), m_awake(false), m_ranging(false),
      m_last_range_status(NO_RANGE_STATUS) {}

VL53L1X::~VL53L1X() {
    deinit();
}

esp_err_t VL53L1X::init(const Config& config) {
    // Refuse to re-initialize so an existing registration is never overwritten or leaked
    ESP_RETURN_ON_FALSE(m_dev == platform::NO_DEVICE, ESP_ERR_INVALID_STATE, TAG, "Already initialized");
    ESP_RETURN_ON_ERROR(validate_config(config), TAG, "Invalid configuration");

    m_dev = platform::register_device(m_i2c);
    ESP_RETURN_ON_FALSE(m_dev != platform::NO_DEVICE, ESP_ERR_NO_MEM, TAG, "Too many sensors");
    m_config = config;

    if (m_xshut_gpio != GPIO_NUM_NC) {
        ESP_LOGI(TAG, "Initializing VL53L1X with XSHUT on GPIO %d", m_xshut_gpio);
        // Set the level before enabling the output so the pin never glitches high.
        // Starting in standby also resets a sensor left running by a previous boot.
        gpio_set_level(m_xshut_gpio, 0);
        gpio_config_t io_config = {};
        io_config.pin_bit_mask = 1ULL << m_xshut_gpio;
        io_config.mode = GPIO_MODE_OUTPUT;
        esp_err_t ret = gpio_config(&io_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure XSHUT: %s", esp_err_to_name(ret));
            platform::unregister_device(m_dev);
            m_dev = platform::NO_DEVICE;
            return ret;
        }
    } else {
        ESP_LOGI(TAG, "Initializing VL53L1X without XSHUT");
    }

    esp_err_t ret = wake();
    if (ret != ESP_OK) {
        // wake() already put the sensor back in standby
        platform::unregister_device(m_dev);
        m_dev = platform::NO_DEVICE;
    }
    return ret;
}

esp_err_t VL53L1X::shutdown() {
    ESP_RETURN_ON_FALSE(m_dev != platform::NO_DEVICE, ESP_ERR_INVALID_STATE, TAG, "Not initialized");
    ESP_RETURN_ON_FALSE(m_xshut_gpio != GPIO_NUM_NC, ESP_ERR_NOT_SUPPORTED, TAG, "No XSHUT pin");
    enter_standby();
    return ESP_OK;
}

esp_err_t VL53L1X::wake() {
    ESP_RETURN_ON_FALSE(m_dev != platform::NO_DEVICE, ESP_ERR_INVALID_STATE, TAG, "Not initialized");

    m_awake = false;
    m_ranging = false;
    if (m_xshut_gpio != GPIO_NUM_NC) {
        gpio_set_level(m_xshut_gpio, 1);
    }

    esp_err_t ret = wait_for_boot();
    if (ret == ESP_OK) {
        uint16_t model_id = 0;
        if (VL53L1X_GetSensorId(m_dev, &model_id) != 0) {
            ret = ESP_FAIL;
        } else if (model_id != MODEL_ID) {
            ESP_LOGE(TAG, "Unexpected model ID 0x%04x", model_id);
            ret = ESP_ERR_NOT_FOUND;
        }
    }
    if (ret == ESP_OK) {
        ret = sensor_init();
    }
    if (ret == ESP_OK) {
        ret = apply_config();
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake sensor: %s", esp_err_to_name(ret));
        if (m_xshut_gpio != GPIO_NUM_NC) {
            enter_standby();
        }
        return ret;
    }

    m_awake = true;
    return ESP_OK;
}

esp_err_t VL53L1X::start_ranging() {
    ESP_RETURN_ON_FALSE(m_awake, ESP_ERR_INVALID_STATE, TAG, "Sensor not awake");
    // Discard any measurement left over from before
    int8_t status = VL53L1X_ClearInterrupt(m_dev);
    status |= VL53L1X_StartRanging(m_dev);
    ESP_RETURN_ON_FALSE(status == 0, ESP_FAIL, TAG, "Failed to start ranging");
    m_ranging = true;
    return ESP_OK;
}

esp_err_t VL53L1X::stop_ranging() {
    ESP_RETURN_ON_FALSE(m_awake, ESP_ERR_INVALID_STATE, TAG, "Sensor not awake");
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

void VL53L1X::deinit() {
    if (m_dev == platform::NO_DEVICE) {
        return;
    }
    if (m_xshut_gpio != GPIO_NUM_NC) {
        // Leave XSHUT driven low: a released pin may float or be pulled up, waking the sensor
        enter_standby();
    } else if (m_ranging) {
        VL53L1X_StopRanging(m_dev);
    }
    platform::unregister_device(m_dev);
    m_dev = platform::NO_DEVICE;
    m_awake = false;
    m_ranging = false;
    ESP_LOGI(TAG, "VL53L1X driver released");
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
            ESP_LOGE(TAG, "Sensor did not boot");
            return ESP_ERR_TIMEOUT;
        }
        delay_ms(1);
    }
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
    status |= VL53L1_WrByte(m_dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); // Two bounds VHV
    status |= VL53L1_WrByte(m_dev, VHV_CONFIG__INIT, 0); // Start VHV from the previous temperature
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

void VL53L1X::enter_standby() {
    gpio_set_level(m_xshut_gpio, 0);
    m_awake = false;
    m_ranging = false;
}
