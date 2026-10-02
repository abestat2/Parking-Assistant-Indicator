// Tier 2: driver logic, including the unmodified ST ULD, against a simulated sensor.
// SimulatedSensor implements I2cDevice with a register map and the few behaviours the ULD relies
// on (boot status, data-ready interrupt, ranging start/stop). XSHUT goes through the GPIO mock,
// and time is simulated through the FreeRTOS mock.

#include <array>
#include <cstring>
#include <type_traits>
#include <vector>
#include "unity.h"
#include "VL53L1X.hpp"

extern "C" {
#include "Mockgpio.h"
#include "Mocktask.h"
}

using namespace vl53l1x;

static_assert(!std::is_copy_constructible_v<VL53L1X>, "VL53L1X must not be copyable");
static_assert(!std::is_copy_assignable_v<VL53L1X>, "VL53L1X must not be copy-assignable");

static constexpr gpio_num_t XSHUT_GPIO = GPIO_NUM_4;

// Registers used by the ULD (see VL53L1X_api.h)
static constexpr uint16_t REG_VHV_LOOP_BOUND = 0x0008;
static constexpr uint16_t REG_VHV_INIT = 0x000B;
static constexpr uint16_t REG_GPIO_HV_MUX_CTRL = 0x0030;
static constexpr uint16_t REG_GPIO_TIO_HV_STATUS = 0x0031;
static constexpr uint16_t REG_PHASECAL_TIMEOUT = 0x004B;
static constexpr uint16_t REG_TIMEOUT_MACROP_A_HI = 0x005E;
static constexpr uint16_t REG_INTERMEASUREMENT_PERIOD = 0x006C;
static constexpr uint16_t REG_ROI_XY_SIZE = 0x0080;
static constexpr uint16_t REG_INTERRUPT_CLEAR = 0x0086;
static constexpr uint16_t REG_MODE_START = 0x0087;
static constexpr uint16_t REG_RANGE_STATUS = 0x0089;
static constexpr uint16_t REG_RANGE_MM = 0x0096;
static constexpr uint16_t REG_OSC_CALIBRATE_VAL = 0x00DE;
static constexpr uint16_t REG_FIRMWARE_SYSTEM_STATUS = 0x00E5;
static constexpr uint16_t REG_MODEL_ID = 0x010F;

// Raw RESULT__RANGE_STATUS values and the ULD statuses they map to
static constexpr uint8_t RAW_VALID = 9;          // -> 0
static constexpr uint8_t RAW_SIGMA_FAIL = 6;     // -> 1
static constexpr uint8_t RAW_SIGNAL_FAIL = 4;    // -> 2
static constexpr uint8_t RAW_OUT_OF_BOUNDS = 5;  // -> 4

static constexpr int NEVER = 1000000;

struct Write {
    uint16_t index;
    std::vector<uint8_t> data;
};

class SimulatedSensor : public I2cDevice {
public:
    SimulatedSensor() { power_on(); }

    // Behaviour knobs, applied from the next power-up or measurement
    int boot_polls = 0;          // status reads that report "not booted" after power-up
    int polls_until_ready = 0;   // data-ready reads that report "not ready" for each measurement
    uint16_t model_id = MODEL_ID;
    bool fail_all = false;
    int fail_writes_to = -1;     // register whose writes are not acknowledged

    bool powered = false;
    bool ranging = false;
    std::vector<Write> writes;
    std::array<uint8_t, 0x10000> regs = {};

    void power_on() {
        powered = true;
        ranging = false;
        m_ready = false;
        m_boot_polls_left = boot_polls;
        regs.fill(0);
        regs[REG_MODEL_ID] = model_id >> 8;
        regs[REG_MODEL_ID + 1] = model_id & 0xFF;
        // Oscillator calibration read by SetInterMeasurementInMs()
        regs[REG_OSC_CALIBRATE_VAL] = 0x01;
        regs[REG_OSC_CALIBRATE_VAL + 1] = 0xF4;
        // Reset value: active-high interrupt
        regs[REG_GPIO_HV_MUX_CTRL] = 0x01;
    }

    void power_off() {
        powered = false;
        ranging = false;
    }

    void set_result(uint8_t raw_status, uint16_t distance_mm) {
        regs[REG_RANGE_STATUS] = raw_status;
        regs[REG_RANGE_MM] = distance_mm >> 8;
        regs[REG_RANGE_MM + 1] = distance_mm & 0xFF;
    }

    uint16_t reg16(uint16_t index) const { return static_cast<uint16_t>(regs[index] << 8 | regs[index + 1]); }
    uint32_t reg32(uint16_t index) const {
        return static_cast<uint32_t>(regs[index]) << 24 | static_cast<uint32_t>(regs[index + 1]) << 16 |
               static_cast<uint32_t>(regs[index + 2]) << 8 | regs[index + 3];
    }

    bool data_ready() const { return m_ready; }

    esp_err_t write(const uint8_t* data, size_t size) override {
        if (!powered || fail_all || size < 2) {
            return ESP_FAIL;
        }
        uint16_t index = static_cast<uint16_t>(data[0] << 8 | data[1]);
        if (index == fail_writes_to) {
            return ESP_FAIL;
        }
        writes.push_back({ index, std::vector<uint8_t>(data + 2, data + size) });
        for (size_t i = 2; i < size; i++) {
            write_register(static_cast<uint16_t>(index + i - 2), data[i]);
        }
        return ESP_OK;
    }

    esp_err_t write_read(const uint8_t* write_data, size_t write_size, uint8_t* read_data, size_t read_size) override {
        if (!powered || fail_all || write_size != 2) {
            return ESP_FAIL;
        }
        uint16_t index = static_cast<uint16_t>(write_data[0] << 8 | write_data[1]);
        for (size_t i = 0; i < read_size; i++) {
            read_data[i] = read_register(static_cast<uint16_t>(index + i));
        }
        return ESP_OK;
    }

private:
    void start_measurement() {
        m_ready = false;
        m_polls_left = polls_until_ready;
    }

    void write_register(uint16_t index, uint8_t value) {
        regs[index] = value;
        if (index == REG_MODE_START) {
            ranging = value == 0x40;
            if (ranging) {
                start_measurement();
            } else {
                m_ready = false;
            }
        } else if (index == REG_INTERRUPT_CLEAR && (value & 0x01)) {
            m_ready = false;
            if (ranging) {
                start_measurement();
            }
        }
    }

    uint8_t read_register(uint16_t index) {
        if (index == REG_FIRMWARE_SYSTEM_STATUS) {
            if (m_boot_polls_left > 0) {
                m_boot_polls_left--;
                return 0;
            }
            return 1;
        }
        if (index == REG_GPIO_TIO_HV_STATUS) {
            if (ranging && !m_ready) {
                if (m_polls_left > 0) {
                    m_polls_left--;
                } else {
                    m_ready = true;
                }
            }
            // Bit 4 of GPIO_HV_MUX__CTRL set means active-low
            uint8_t active_level = (regs[REG_GPIO_HV_MUX_CTRL] & 0x10) ? 0 : 1;
            uint8_t level = m_ready ? active_level : !active_level;
            return static_cast<uint8_t>((regs[index] & ~0x01) | level);
        }
        return regs[index];
    }

    int m_boot_polls_left = 0;
    int m_polls_left = 0;
    bool m_ready = false;
};

// State shared with the mock callbacks
static SimulatedSensor* s_sensor;
static TickType_t s_ticks;
static int s_delay_calls;
static std::vector<int> s_xshut_levels;
static gpio_config_t s_gpio_config;
static int s_gpio_config_calls;
static esp_err_t s_gpio_config_result;

static esp_err_t stub_gpio_set_level(gpio_num_t gpio, uint32_t level, int calls)
{
    TEST_ASSERT_EQUAL(XSHUT_GPIO, gpio);
    s_xshut_levels.push_back(static_cast<int>(level));
    if (level) {
        s_sensor->power_on();
    } else {
        s_sensor->power_off();
    }
    return ESP_OK;
}

static esp_err_t stub_gpio_config(const gpio_config_t* config, int calls)
{
    s_gpio_config = *config;
    s_gpio_config_calls++;
    return s_gpio_config_result;
}

static TickType_t stub_tick_count(int calls)
{
    return s_ticks;
}

static void stub_task_delay(const TickType_t ticks, int calls)
{
    TEST_ASSERT_GREATER_THAN_UINT32(0, ticks);
    s_ticks += ticks;
    s_delay_calls++;
}

extern "C" void setUp(void)
{
    Mockgpio_Init();
    Mocktask_Init();
    s_sensor = nullptr;
    s_ticks = 0;
    s_delay_calls = 0;
    s_xshut_levels.clear();
    s_gpio_config = {};
    s_gpio_config_calls = 0;
    s_gpio_config_result = ESP_OK;
    xTaskGetTickCount_Stub(stub_tick_count);
    vTaskDelay_Stub(stub_task_delay);
}

extern "C" void tearDown(void)
{
    Mockgpio_Verify();
    Mocktask_Verify();
    Mockgpio_Destroy();
    Mocktask_Destroy();
}

// Use a simulated sensor whose XSHUT is driven through the GPIO mock. It starts unpowered, as
// the driver holds it in standby before booting it.
static void attach_xshut(SimulatedSensor& sensor)
{
    s_sensor = &sensor;
    sensor.power_off();
    gpio_set_level_Stub(stub_gpio_set_level);
    gpio_config_Stub(stub_gpio_config);
}

static Config test_config()
{
    Config config;
    config.distance_mode = DistanceMode::Short;
    config.timing_budget_ms = 33;
    config.inter_measurement_ms = 100;
    config.roi_width = 8;
    config.roi_height = 6;
    return config;
}

static bool wrote(const SimulatedSensor& sensor, uint16_t index)
{
    for (const Write& w : sensor.writes) {
        if (w.index == index) {
            return true;
        }
    }
    return false;
}

// ---- init() ----

TEST_CASE("simulated: init holds XSHUT low, then boots and configures the sensor", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init(test_config()));

    // XSHUT driven low before the pin becomes an output, then released to boot
    TEST_ASSERT_EQUAL(1, s_gpio_config_calls);
    TEST_ASSERT_EQUAL_UINT64(1ULL << XSHUT_GPIO, s_gpio_config.pin_bit_mask);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, s_gpio_config.mode);
    TEST_ASSERT_EQUAL(2, s_xshut_levels.size());
    TEST_ASSERT_EQUAL(0, s_xshut_levels[0]);
    TEST_ASSERT_EQUAL(1, s_xshut_levels[1]);

    TEST_ASSERT_TRUE(sensor.is_awake());
    TEST_ASSERT_FALSE(sim.ranging);
    TEST_ASSERT_EQUAL_UINT8(255, sensor.last_range_status());
    TEST_ASSERT_EQUAL_UINT8(0x09, sim.regs[REG_VHV_LOOP_BOUND]);
    TEST_ASSERT_EQUAL_UINT8(0x00, sim.regs[REG_VHV_INIT]);

    // Short mode, 33 ms budget (short-mode register value), 8x6 ROI
    TEST_ASSERT_EQUAL_UINT8(0x14, sim.regs[REG_PHASECAL_TIMEOUT]);
    TEST_ASSERT_EQUAL_UINT16(0x00D6, sim.reg16(REG_TIMEOUT_MACROP_A_HI));
    TEST_ASSERT_EQUAL_UINT8((6 - 1) << 4 | (8 - 1), sim.regs[REG_ROI_XY_SIZE]);
    // 0x1F4 oscillator value x 100 ms x 1.075
    TEST_ASSERT_UINT32_WITHIN(1, 53750, sim.reg32(REG_INTERMEASUREMENT_PERIOD));
}

TEST_CASE("simulated: default configuration is written in one transaction", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());

    // 0x2D through 0x87, ending with ranging stopped
    TEST_ASSERT_GREATER_THAN(0, sim.writes.size());
    const Write& first = sim.writes[0];
    TEST_ASSERT_EQUAL_HEX16(0x2D, first.index);
    TEST_ASSERT_EQUAL(0x87 - 0x2D + 1, first.data.size());
    TEST_ASSERT_EQUAL_HEX8(0x00, first.data.back());

    // Default config is long mode with a 50 ms budget
    TEST_ASSERT_EQUAL_UINT8(0x0A, sim.regs[REG_PHASECAL_TIMEOUT]);
    TEST_ASSERT_EQUAL_UINT16(0x00AD, sim.reg16(REG_TIMEOUT_MACROP_A_HI));
    TEST_ASSERT_EQUAL_UINT8(0xFF, sim.regs[REG_ROI_XY_SIZE]);
}

TEST_CASE("simulated: invalid config is rejected without touching the hardware", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    VL53L1X sensor(sim, XSHUT_GPIO);
    Config config;
    config.roi_width = 2;
    // No GPIO stubs are installed, so any GPIO call fails the test
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, sensor.init(config));
    TEST_ASSERT_TRUE(sim.writes.empty());
    TEST_ASSERT_FALSE(sensor.is_awake());
}

TEST_CASE("simulated: second init returns invalid state and keeps the sensor working", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    size_t writes = sim.writes.size();

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, sensor.init());
    TEST_ASSERT_EQUAL(writes, sim.writes.size());

    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());
    sim.set_result(RAW_VALID, 777);
    TEST_ASSERT_EQUAL_INT32(777, sensor.read_range_mm(100));
}

TEST_CASE("simulated: init waits for the sensor to finish booting", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.boot_polls = 3;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_TRUE(sensor.is_awake());
    TEST_ASSERT_EQUAL(3, s_delay_calls);
}

TEST_CASE("simulated: sensor that never boots times out and is left in standby", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.boot_polls = NEVER;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, sensor.init());

    TEST_ASSERT_FALSE(sensor.is_awake());
    TEST_ASSERT_FALSE(sim.powered);
    TEST_ASSERT_EQUAL(0, s_xshut_levels.back());
    // Gave up after about the 100 ms boot timeout
    TEST_ASSERT_UINT32_WITHIN(pdMS_TO_TICKS(20) + 1, pdMS_TO_TICKS(100), s_ticks);

    // The failed init released everything, so it can be retried
    sim.boot_polls = 0;
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_TRUE(sensor.is_awake());
}

TEST_CASE("simulated: device with the wrong model ID is rejected", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.model_id = 0xEEAA;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, sensor.init());
    TEST_ASSERT_FALSE(sensor.is_awake());
    TEST_ASSERT_FALSE(sim.powered);
}

TEST_CASE("simulated: init times out instead of hanging when the first measurement never arrives", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.polls_until_ready = NEVER;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, sensor.init());
    TEST_ASSERT_FALSE(sensor.is_awake());
    TEST_ASSERT_FALSE(sim.powered);
    TEST_ASSERT_UINT32_WITHIN(pdMS_TO_TICKS(20) + 1, pdMS_TO_TICKS(500), s_ticks);
}

TEST_CASE("simulated: I2C failure while writing the configuration fails init", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.fail_writes_to = 0x2D;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_FAIL, sensor.init());
    TEST_ASSERT_FALSE(sensor.is_awake());
    TEST_ASSERT_FALSE(sim.powered);
}

TEST_CASE("simulated: I2C failure while applying the configuration fails init", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    sim.fail_writes_to = REG_ROI_XY_SIZE;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_FAIL, sensor.init());
    TEST_ASSERT_FALSE(sensor.is_awake());
}

TEST_CASE("simulated: GPIO configuration failure is passed through", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    s_gpio_config_result = ESP_ERR_INVALID_ARG;
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, sensor.init());
    TEST_ASSERT_TRUE(sim.writes.empty());

    s_gpio_config_result = ESP_OK;
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
}

TEST_CASE("simulated: without XSHUT the sensor is used as already powered", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    // No GPIO stubs are installed, so any GPIO call fails the test
    VL53L1X sensor(sim);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_TRUE(sensor.is_awake());
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, sensor.shutdown());
    TEST_ASSERT_TRUE(sensor.is_awake());

    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());
    sensor.deinit();
    // deinit stops ranging when it cannot power the sensor down
    TEST_ASSERT_FALSE(sim.ranging);
}

// ---- ranging ----

TEST_CASE("simulated: read returns the measured distance and clears the interrupt", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());
    TEST_ASSERT_TRUE(sim.ranging);

    sim.polls_until_ready = 2;
    sim.set_result(RAW_VALID, 1234);
    TEST_ASSERT_EQUAL_INT32(1234, sensor.read_range_mm(100));
    TEST_ASSERT_EQUAL_UINT8(0, sensor.last_range_status());
    TEST_ASSERT_FALSE(sim.data_ready());

    sim.set_result(RAW_VALID, 2500);
    TEST_ASSERT_EQUAL_INT32(2500, sensor.read_range_mm(100));
}

TEST_CASE("simulated: failed measurements are reported with their status", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());

    sim.set_result(RAW_SIGNAL_FAIL, 4000);
    TEST_ASSERT_EQUAL_INT32(RANGE_OUT_OF_RANGE, sensor.read_range_mm(100));
    TEST_ASSERT_EQUAL_UINT8(2, sensor.last_range_status());

    sim.set_result(RAW_OUT_OF_BOUNDS, 4000);
    TEST_ASSERT_EQUAL_INT32(RANGE_OUT_OF_RANGE, sensor.read_range_mm(100));
    TEST_ASSERT_EQUAL_UINT8(4, sensor.last_range_status());

    sim.set_result(RAW_SIGMA_FAIL, 900);
    TEST_ASSERT_EQUAL_INT32(RANGE_INVALID, sensor.read_range_mm(100));
    TEST_ASSERT_EQUAL_UINT8(1, sensor.last_range_status());
}

TEST_CASE("simulated: read times out when no measurement arrives", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    sim.polls_until_ready = NEVER;
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());

    TickType_t start = s_ticks;
    TEST_ASSERT_EQUAL_INT32(RANGE_TIMEOUT, sensor.read_range_mm(200));
    TEST_ASSERT_UINT32_WITHIN(pdMS_TO_TICKS(20) + 1, pdMS_TO_TICKS(200), s_ticks - start);

    // A zero timeout checks once without waiting
    start = s_ticks;
    TEST_ASSERT_EQUAL_INT32(RANGE_TIMEOUT, sensor.read_range_mm(0));
    TEST_ASSERT_EQUAL_UINT32(start, s_ticks);
}

TEST_CASE("simulated: read before ranging or init reports an error", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm(100));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, sensor.start_ranging());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, sensor.stop_ranging());

    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm(100));

    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.stop_ranging());
    TEST_ASSERT_FALSE(sim.ranging);
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm(100));
}

TEST_CASE("simulated: I2C failure during a read reports an error", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());

    sim.fail_all = true;
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm(100));

    sim.fail_all = false;
    sim.set_result(RAW_VALID, 640);
    TEST_ASSERT_EQUAL_INT32(640, sensor.read_range_mm(100));
}

// ---- shutdown() and wake() ----

TEST_CASE("simulated: shutdown pulls XSHUT low and wake restores the configuration", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init(test_config()));
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());

    TEST_ASSERT_EQUAL(ESP_OK, sensor.shutdown());
    TEST_ASSERT_EQUAL(0, s_xshut_levels.back());
    TEST_ASSERT_FALSE(sim.powered);
    TEST_ASSERT_FALSE(sensor.is_awake());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, sensor.start_ranging());
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm(100));

    // Power-up cleared the simulated registers, so the configuration must be written again
    sim.writes.clear();
    TEST_ASSERT_EQUAL(ESP_OK, sensor.wake());
    TEST_ASSERT_TRUE(sensor.is_awake());
    TEST_ASSERT_TRUE(wrote(sim, 0x2D));
    TEST_ASSERT_EQUAL_UINT8(0x14, sim.regs[REG_PHASECAL_TIMEOUT]);
    TEST_ASSERT_EQUAL_UINT8((6 - 1) << 4 | (8 - 1), sim.regs[REG_ROI_XY_SIZE]);

    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());
    sim.set_result(RAW_VALID, 321);
    TEST_ASSERT_EQUAL_INT32(321, sensor.read_range_mm(100));
}

TEST_CASE("simulated: shutdown and wake before init report invalid state", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, sensor.shutdown());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, sensor.wake());
}

TEST_CASE("simulated: failed wake leaves the sensor in standby and a later wake recovers", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.shutdown());

    sim.boot_polls = NEVER;
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, sensor.wake());
    TEST_ASSERT_FALSE(sensor.is_awake());
    TEST_ASSERT_FALSE(sim.powered);

    sim.boot_polls = 0;
    TEST_ASSERT_EQUAL(ESP_OK, sensor.wake());
    TEST_ASSERT_TRUE(sensor.is_awake());
}

// ---- lifecycle ----

TEST_CASE("simulated: deinit leaves XSHUT low and can be repeated", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    VL53L1X sensor(sim, XSHUT_GPIO);
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
    TEST_ASSERT_EQUAL(ESP_OK, sensor.start_ranging());

    sensor.deinit();
    TEST_ASSERT_FALSE(sim.powered);
    TEST_ASSERT_FALSE(sensor.is_awake());
    size_t levels = s_xshut_levels.size();
    sensor.deinit();
    TEST_ASSERT_EQUAL(levels, s_xshut_levels.size());
    TEST_ASSERT_EQUAL_INT32(RANGE_ERROR, sensor.read_range_mm(100));

    // A deinitialized sensor can be initialized again
    TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
}

TEST_CASE("simulated: destructor puts the sensor in standby", "[vl53l1x][host]")
{
    SimulatedSensor sim;
    attach_xshut(sim);
    {
        VL53L1X sensor(sim, XSHUT_GPIO);
        TEST_ASSERT_EQUAL(ESP_OK, sensor.init());
        TEST_ASSERT_TRUE(sim.powered);
    }
    TEST_ASSERT_FALSE(sim.powered);
}

TEST_CASE("simulated: several sensors work side by side up to the limit", "[vl53l1x][host]")
{
    // Without XSHUT so no GPIO calls are made
    SimulatedSensor sims[5];
    VL53L1X s0(sims[0]), s1(sims[1]), s2(sims[2]), s3(sims[3]), s4(sims[4]);
    VL53L1X* sensors[] = { &s0, &s1, &s2, &s3 };
    for (VL53L1X* s : sensors) {
        TEST_ASSERT_EQUAL(ESP_OK, s->init());
        TEST_ASSERT_EQUAL(ESP_OK, s->start_ranging());
    }
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, s4.init());
    TEST_ASSERT_TRUE(sims[4].writes.empty());

    // Each sensor talks only to its own device
    for (int i = 0; i < 4; i++) {
        sims[i].set_result(RAW_VALID, static_cast<uint16_t>(100 * (i + 1)));
    }
    TEST_ASSERT_EQUAL_INT32(300, s2.read_range_mm(100));
    TEST_ASSERT_EQUAL_INT32(100, s0.read_range_mm(100));
    TEST_ASSERT_EQUAL_INT32(400, s3.read_range_mm(100));

    // Releasing one frees a slot
    s1.deinit();
    TEST_ASSERT_EQUAL(ESP_OK, s4.init());
}
