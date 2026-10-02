// Tier 2: the ULD platform layer (vl53l1x_platform.cpp) against a recording I2C device.
// These are white-box tests of the register access the ST driver relies on.

#include <vector>
#include "unity.h"
#include "vl53l1x_platform.hpp"
#include "vl53l1_platform.h"

extern "C" {
#include "Mocktask.h"
}

using namespace vl53l1x;

// Records every transaction and answers reads with a fixed byte pattern
class RecordingDevice : public I2cDevice {
public:
    struct Transaction {
        std::vector<uint8_t> written;
        size_t read_size;
    };

    std::vector<Transaction> transactions;
    int fail_after = -1; // transactions that succeed before every later one fails; -1 never fails
    std::vector<uint8_t> read_data;

    esp_err_t write(const uint8_t* data, size_t size) override {
        transactions.push_back({ std::vector<uint8_t>(data, data + size), 0 });
        return result();
    }

    esp_err_t write_read(const uint8_t* write_data, size_t write_size, uint8_t* out, size_t read_size) override {
        transactions.push_back({ std::vector<uint8_t>(write_data, write_data + write_size), read_size });
        esp_err_t ret = result();
        if (ret == ESP_OK) {
            for (size_t i = 0; i < read_size; i++) {
                out[i] = i < read_data.size() ? read_data[i] : 0;
            }
        }
        return ret;
    }

private:
    esp_err_t result() const {
        return fail_after >= 0 && static_cast<int>(transactions.size()) > fail_after ? ESP_FAIL : ESP_OK;
    }
};

// Registers a device for the duration of a test and always releases its slot
struct Registered {
    RecordingDevice device;
    uint16_t dev;
    Registered() : dev(platform::register_device(device)) { TEST_ASSERT_NOT_EQUAL(platform::NO_DEVICE, dev); }
    ~Registered() { platform::unregister_device(dev); }
};

static void assert_bytes(const std::vector<uint8_t>& expected, const std::vector<uint8_t>& actual)
{
    TEST_ASSERT_EQUAL(expected.size(), actual.size());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected.data(), actual.data(), expected.size());
}

// ---- registry ----

TEST_CASE("platform: registry hands out each slot once and reuses released ones", "[vl53l1x][platform]")
{
    RecordingDevice devices[platform::MAX_SENSORS + 1];
    uint16_t ids[platform::MAX_SENSORS];
    for (size_t i = 0; i < platform::MAX_SENSORS; i++) {
        ids[i] = platform::register_device(devices[i]);
        TEST_ASSERT_NOT_EQUAL(platform::NO_DEVICE, ids[i]);
        for (size_t j = 0; j < i; j++) {
            TEST_ASSERT_NOT_EQUAL(ids[j], ids[i]);
        }
    }
    TEST_ASSERT_EQUAL(platform::NO_DEVICE, platform::register_device(devices[platform::MAX_SENSORS]));

    // Out-of-range ids are ignored and leave the real registrations alone
    platform::unregister_device(platform::NO_DEVICE);
    platform::unregister_device(static_cast<uint16_t>(platform::MAX_SENSORS));
    TEST_ASSERT_EQUAL(platform::NO_DEVICE, platform::register_device(devices[platform::MAX_SENSORS]));

    platform::unregister_device(ids[1]);
    TEST_ASSERT_EQUAL(ids[1], platform::register_device(devices[platform::MAX_SENSORS]));

    for (uint16_t id : ids) {
        platform::unregister_device(id);
    }
}

TEST_CASE("platform: each id reaches its own device", "[vl53l1x][platform]")
{
    Registered a, b;
    TEST_ASSERT_EQUAL(0, VL53L1_WrByte(b.dev, 0x0001, 0x22));
    TEST_ASSERT_EQUAL(0, VL53L1_WrByte(a.dev, 0x0001, 0x11));
    TEST_ASSERT_EQUAL(1, a.device.transactions.size());
    TEST_ASSERT_EQUAL(1, b.device.transactions.size());
    TEST_ASSERT_EQUAL_HEX8(0x11, a.device.transactions[0].written[2]);
    TEST_ASSERT_EQUAL_HEX8(0x22, b.device.transactions[0].written[2]);
}

TEST_CASE("platform: unregistered ids fail every access without output", "[vl53l1x][platform]")
{
    const uint16_t ids[] = { platform::NO_DEVICE, static_cast<uint16_t>(platform::MAX_SENSORS), 0 };
    for (uint16_t dev : ids) {
        uint8_t data[4] = { 0xAA, 0xAA, 0xAA, 0xAA };
        uint8_t byte = 0xAA;
        uint16_t word = 0xAAAA;
        uint32_t dword = 0xAAAAAAAA;
        TEST_ASSERT_NOT_EQUAL(0, VL53L1_WriteMulti(dev, 0x10, data, sizeof(data)));
        TEST_ASSERT_NOT_EQUAL(0, VL53L1_ReadMulti(dev, 0x10, data, sizeof(data)));
        TEST_ASSERT_NOT_EQUAL(0, VL53L1_WrByte(dev, 0x10, 1));
        TEST_ASSERT_NOT_EQUAL(0, VL53L1_WrWord(dev, 0x10, 1));
        TEST_ASSERT_NOT_EQUAL(0, VL53L1_WrDWord(dev, 0x10, 1));
        TEST_ASSERT_NOT_EQUAL(0, VL53L1_RdByte(dev, 0x10, &byte));
        TEST_ASSERT_NOT_EQUAL(0, VL53L1_RdWord(dev, 0x10, &word));
        TEST_ASSERT_NOT_EQUAL(0, VL53L1_RdDWord(dev, 0x10, &dword));
        TEST_ASSERT_EQUAL_HEX8(0xAA, byte);
        TEST_ASSERT_EQUAL_HEX16(0xAAAA, word);
        TEST_ASSERT_EQUAL_HEX32(0xAAAAAAAA, dword);
    }
}

// ---- writes ----

TEST_CASE("platform: writes send a big-endian index followed by big-endian data", "[vl53l1x][platform]")
{
    Registered r;
    TEST_ASSERT_EQUAL(0, VL53L1_WrByte(r.dev, 0x0123, 0x45));
    TEST_ASSERT_EQUAL(0, VL53L1_WrWord(r.dev, 0x0456, 0x789A));
    TEST_ASSERT_EQUAL(0, VL53L1_WrDWord(r.dev, 0x0789, 0x12345678));

    TEST_ASSERT_EQUAL(3, r.device.transactions.size());
    assert_bytes({ 0x01, 0x23, 0x45 }, r.device.transactions[0].written);
    assert_bytes({ 0x04, 0x56, 0x78, 0x9A }, r.device.transactions[1].written);
    assert_bytes({ 0x07, 0x89, 0x12, 0x34, 0x56, 0x78 }, r.device.transactions[2].written);
}

TEST_CASE("platform: a multi-byte write is one transaction up to 128 data bytes", "[vl53l1x][platform]")
{
    Registered r;
    uint8_t data[128];
    for (size_t i = 0; i < sizeof(data); i++) {
        data[i] = static_cast<uint8_t>(i);
    }
    TEST_ASSERT_EQUAL(0, VL53L1_WriteMulti(r.dev, 0x002D, data, sizeof(data)));
    TEST_ASSERT_EQUAL(1, r.device.transactions.size());
    TEST_ASSERT_EQUAL(130, r.device.transactions[0].written.size());
    TEST_ASSERT_EQUAL_HEX8(0x2D, r.device.transactions[0].written[1]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(data, r.device.transactions[0].written.data() + 2, sizeof(data));
}

TEST_CASE("platform: longer writes are split with the index advanced for each chunk", "[vl53l1x][platform]")
{
    Registered r;
    uint8_t data[300];
    for (size_t i = 0; i < sizeof(data); i++) {
        data[i] = static_cast<uint8_t>(i * 7);
    }
    TEST_ASSERT_EQUAL(0, VL53L1_WriteMulti(r.dev, 0x0100, data, sizeof(data)));

    const uint16_t indexes[] = { 0x0100, 0x0180, 0x0200 };
    const size_t sizes[] = { 128, 128, 44 };
    TEST_ASSERT_EQUAL(3, r.device.transactions.size());
    size_t offset = 0;
    for (size_t i = 0; i < 3; i++) {
        const std::vector<uint8_t>& written = r.device.transactions[i].written;
        TEST_ASSERT_EQUAL(2 + sizes[i], written.size());
        TEST_ASSERT_EQUAL_HEX16(indexes[i], written[0] << 8 | written[1]);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(data + offset, written.data() + 2, sizes[i]);
        offset += sizes[i];
    }
}

TEST_CASE("platform: a split write stops at the first failed chunk", "[vl53l1x][platform]")
{
    Registered r;
    r.device.fail_after = 1;
    uint8_t data[300] = {};
    TEST_ASSERT_NOT_EQUAL(0, VL53L1_WriteMulti(r.dev, 0x0100, data, sizeof(data)));
    TEST_ASSERT_EQUAL(2, r.device.transactions.size());
}

TEST_CASE("platform: an empty write sends only the index, and a missing buffer is rejected", "[vl53l1x][platform]")
{
    Registered r;
    TEST_ASSERT_EQUAL(0, VL53L1_WriteMulti(r.dev, 0x0042, nullptr, 0));
    TEST_ASSERT_EQUAL(1, r.device.transactions.size());
    assert_bytes({ 0x00, 0x42 }, r.device.transactions[0].written);

    TEST_ASSERT_NOT_EQUAL(0, VL53L1_WriteMulti(r.dev, 0x0042, nullptr, 4));
    TEST_ASSERT_EQUAL(1, r.device.transactions.size());
}

TEST_CASE("platform: a failed write is reported", "[vl53l1x][platform]")
{
    Registered r;
    r.device.fail_after = 0;
    TEST_ASSERT_NOT_EQUAL(0, VL53L1_WrByte(r.dev, 0x10, 1));
    TEST_ASSERT_NOT_EQUAL(0, VL53L1_WrWord(r.dev, 0x10, 1));
    TEST_ASSERT_NOT_EQUAL(0, VL53L1_WrDWord(r.dev, 0x10, 1));
}

// ---- reads ----

TEST_CASE("platform: reads send a big-endian index and decode big-endian data", "[vl53l1x][platform]")
{
    Registered r;
    r.device.read_data = { 0x12, 0x34, 0x56, 0x78 };
    uint8_t byte = 0;
    uint16_t word = 0;
    uint32_t dword = 0;
    TEST_ASSERT_EQUAL(0, VL53L1_RdByte(r.dev, 0x00E5, &byte));
    TEST_ASSERT_EQUAL(0, VL53L1_RdWord(r.dev, 0x010F, &word));
    TEST_ASSERT_EQUAL(0, VL53L1_RdDWord(r.dev, 0x006C, &dword));
    TEST_ASSERT_EQUAL_HEX8(0x12, byte);
    TEST_ASSERT_EQUAL_HEX16(0x1234, word);
    TEST_ASSERT_EQUAL_HEX32(0x12345678, dword);

    TEST_ASSERT_EQUAL(3, r.device.transactions.size());
    assert_bytes({ 0x00, 0xE5 }, r.device.transactions[0].written);
    assert_bytes({ 0x01, 0x0F }, r.device.transactions[1].written);
    assert_bytes({ 0x00, 0x6C }, r.device.transactions[2].written);
    TEST_ASSERT_EQUAL(1, r.device.transactions[0].read_size);
    TEST_ASSERT_EQUAL(2, r.device.transactions[1].read_size);
    TEST_ASSERT_EQUAL(4, r.device.transactions[2].read_size);
}

TEST_CASE("platform: a multi-byte read is a single transaction", "[vl53l1x][platform]")
{
    Registered r;
    r.device.read_data = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17 };
    uint8_t out[17] = {};
    TEST_ASSERT_EQUAL(0, VL53L1_ReadMulti(r.dev, 0x0089, out, sizeof(out)));
    TEST_ASSERT_EQUAL(1, r.device.transactions.size());
    TEST_ASSERT_EQUAL(17, r.device.transactions[0].read_size);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(r.device.read_data.data(), out, sizeof(out));
}

TEST_CASE("platform: a failed read is reported and leaves the output unchanged", "[vl53l1x][platform]")
{
    Registered r;
    r.device.fail_after = 0;
    r.device.read_data = { 0x12, 0x34, 0x56, 0x78 };
    uint8_t byte = 0xAA;
    uint16_t word = 0xAAAA;
    uint32_t dword = 0xAAAAAAAA;
    TEST_ASSERT_NOT_EQUAL(0, VL53L1_RdByte(r.dev, 0x10, &byte));
    TEST_ASSERT_NOT_EQUAL(0, VL53L1_RdWord(r.dev, 0x10, &word));
    TEST_ASSERT_NOT_EQUAL(0, VL53L1_RdDWord(r.dev, 0x10, &dword));
    TEST_ASSERT_EQUAL_HEX8(0xAA, byte);
    TEST_ASSERT_EQUAL_HEX16(0xAAAA, word);
    TEST_ASSERT_EQUAL_HEX32(0xAAAAAAAA, dword);
}

TEST_CASE("platform: a read into a missing buffer is rejected without a transaction", "[vl53l1x][platform]")
{
    Registered r;
    TEST_ASSERT_NOT_EQUAL(0, VL53L1_ReadMulti(r.dev, 0x10, nullptr, 4));
    TEST_ASSERT_TRUE(r.device.transactions.empty());
}

// ---- delay ----

static TickType_t s_delayed_ticks;

static void record_delay(const TickType_t ticks, int calls)
{
    s_delayed_ticks += ticks;
}

TEST_CASE("platform: WaitMs delays for the requested time", "[vl53l1x][platform]")
{
    s_delayed_ticks = 0;
    vTaskDelay_Stub(record_delay);
    TEST_ASSERT_EQUAL(0, VL53L1_WaitMs(0, 250));
    TEST_ASSERT_EQUAL_UINT32(pdMS_TO_TICKS(250), s_delayed_ticks);
}
