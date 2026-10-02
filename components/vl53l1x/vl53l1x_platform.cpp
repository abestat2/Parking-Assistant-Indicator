// Implementation of the platform functions declared by the ST ULD in vl53l1_platform.h.
// Registers are 16-bit indexes followed by big-endian data, and the sensor auto-increments the
// index, so every access is a single I2C transaction.

#include "vl53l1x_platform.hpp"
#include "vl53l1_platform.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <array>
#include <cstring>

namespace vl53l1x::platform {

static std::array<I2cDevice*, MAX_SENSORS> s_devices = {};

uint16_t register_device(I2cDevice& device) {
    for (size_t i = 0; i < s_devices.size(); i++) {
        if (!s_devices[i]) {
            s_devices[i] = &device;
            return static_cast<uint16_t>(i);
        }
    }
    return NO_DEVICE;
}

void unregister_device(uint16_t dev) {
    if (dev < s_devices.size()) {
        s_devices[dev] = nullptr;
    }
}

static I2cDevice* lookup(uint16_t dev) {
    return dev < s_devices.size() ? s_devices[dev] : nullptr;
}

} // namespace vl53l1x::platform

using vl53l1x::platform::lookup;

static constexpr int8_t PLATFORM_OK = 0;
static constexpr int8_t PLATFORM_ERROR = -1;

// Large enough for the 91-byte default configuration in one transaction
static constexpr size_t MAX_WRITE_CHUNK = 128;

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t* pdata, uint32_t count) {
    I2cDevice* device = lookup(dev);
    if (!device || (!pdata && count > 0)) {
        return PLATFORM_ERROR;
    }

    uint8_t buffer[2 + MAX_WRITE_CHUNK];
    do {
        size_t chunk = count < MAX_WRITE_CHUNK ? count : MAX_WRITE_CHUNK;
        buffer[0] = static_cast<uint8_t>(index >> 8);
        buffer[1] = static_cast<uint8_t>(index & 0xFF);
        if (chunk > 0) {
            std::memcpy(buffer + 2, pdata, chunk);
        }
        if (device->write(buffer, 2 + chunk) != ESP_OK) {
            return PLATFORM_ERROR;
        }
        pdata += chunk;
        index += chunk;
        count -= chunk;
    } while (count > 0);

    return PLATFORM_OK;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t* pdata, uint32_t count) {
    I2cDevice* device = lookup(dev);
    if (!device || !pdata) {
        return PLATFORM_ERROR;
    }

    const uint8_t index_bytes[2] = { static_cast<uint8_t>(index >> 8), static_cast<uint8_t>(index & 0xFF) };
    return device->write_read(index_bytes, sizeof(index_bytes), pdata, count) == ESP_OK ? PLATFORM_OK : PLATFORM_ERROR;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
    return VL53L1_WriteMulti(dev, index, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
    uint8_t bytes[2] = { static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF) };
    return VL53L1_WriteMulti(dev, index, bytes, sizeof(bytes));
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
    uint8_t bytes[4] = {
        static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16),
        static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF),
    };
    return VL53L1_WriteMulti(dev, index, bytes, sizeof(bytes));
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t* pdata) {
    return VL53L1_ReadMulti(dev, index, pdata, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t* pdata) {
    uint8_t bytes[2] = {};
    int8_t status = VL53L1_ReadMulti(dev, index, bytes, sizeof(bytes));
    if (status == PLATFORM_OK) {
        *pdata = static_cast<uint16_t>(bytes[0] << 8 | bytes[1]);
    }
    return status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t* pdata) {
    uint8_t bytes[4] = {};
    int8_t status = VL53L1_ReadMulti(dev, index, bytes, sizeof(bytes));
    if (status == PLATFORM_OK) {
        *pdata = static_cast<uint32_t>(bytes[0]) << 24 | static_cast<uint32_t>(bytes[1]) << 16 |
                 static_cast<uint32_t>(bytes[2]) << 8 | bytes[3];
    }
    return status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms) {
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    return PLATFORM_OK;
}
