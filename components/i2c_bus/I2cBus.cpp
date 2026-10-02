#include "I2cBus.hpp"
#include "esp_log.h"
#include "esp_check.h"

I2cBus::I2cBus(i2c_port_num_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, bool enable_internal_pullups)
    : m_port(port), m_sda_gpio(sda_gpio), m_scl_gpio(scl_gpio), m_enable_internal_pullups(enable_internal_pullups), m_bus(nullptr) {}

I2cBus::~I2cBus() {
    deinit();
}

esp_err_t I2cBus::init() {
    ESP_RETURN_ON_FALSE(!m_bus, ESP_ERR_INVALID_STATE, TAG, "Already initialized");

    ESP_LOGI(TAG, "Initializing I2C bus on SDA %d, SCL %d", m_sda_gpio, m_scl_gpio);

    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = m_port;
    bus_config.sda_io_num = m_sda_gpio;
    bus_config.scl_io_num = m_scl_gpio;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = m_enable_internal_pullups;

    esp_err_t ret = i2c_new_master_bus(&bus_config, &m_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        m_bus = nullptr;
    }
    return ret;
}

void I2cBus::deinit() {
    if (m_bus) {
        esp_err_t ret = i2c_del_master_bus(m_bus);
        if (ret != ESP_OK) {
            // Fails while devices are still attached; keep the handle so they stay usable
            ESP_LOGE(TAG, "Failed to release I2C bus: %s", esp_err_to_name(ret));
            return;
        }
        m_bus = nullptr;
    }
}

esp_err_t I2cBus::probe(uint16_t address) {
    ESP_RETURN_ON_FALSE(m_bus, ESP_ERR_INVALID_STATE, TAG, "Not initialized");
    return i2c_master_probe(m_bus, address, PROBE_TIMEOUT_MS);
}

I2cMasterDevice::I2cMasterDevice(I2cBus& bus, uint16_t address, uint32_t scl_speed_hz, int timeout_ms)
    : m_bus(bus), m_address(address), m_scl_speed_hz(scl_speed_hz), m_timeout_ms(timeout_ms), m_device(nullptr) {}

I2cMasterDevice::~I2cMasterDevice() {
    deinit();
}

esp_err_t I2cMasterDevice::init() {
    ESP_RETURN_ON_FALSE(!m_device, ESP_ERR_INVALID_STATE, TAG, "Already initialized");
    ESP_RETURN_ON_FALSE(m_bus.handle(), ESP_ERR_INVALID_STATE, TAG, "Bus not initialized");

    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = m_address;
    dev_config.scl_speed_hz = m_scl_speed_hz;

    esp_err_t ret = i2c_master_bus_add_device(m_bus.handle(), &dev_config, &m_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device 0x%02x: %s", m_address, esp_err_to_name(ret));
        m_device = nullptr;
    }
    return ret;
}

void I2cMasterDevice::deinit() {
    if (m_device) {
        i2c_master_bus_rm_device(m_device);
        m_device = nullptr;
    }
}

esp_err_t I2cMasterDevice::write(const uint8_t* data, size_t size) {
    if (!m_device) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit(m_device, data, size, m_timeout_ms);
}

esp_err_t I2cMasterDevice::write_read(const uint8_t* write_data, size_t write_size, uint8_t* read_data, size_t read_size) {
    if (!m_device) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(m_device, write_data, write_size, read_data, read_size, m_timeout_ms);
}
